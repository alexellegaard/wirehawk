/* a6_step_test.c — step-and-hold test for TWO daisy-chained A6 drives.
 *
 * Usage: sudo ./a6_step_test <ifname>
 *
 * Sends the IDENTICAL command to both motors. Sequence:
 *   settle 1 s  ->  +1 rev -> dwell 2 s  ->  0 -> dwell 2 s  ->
 *   -1 rev -> dwell 2 s  ->  0 -> dwell 2 s   (then repeats until Ctrl-C).
 *
 * The dwell is the point of the test: after the step, the following error
 * (0x60F4) shows whether the drive overshoots and how long it takes to settle.
 * Each step prints a 200 ms trace plus a summary (peak |ferr| and settle time)
 * for M0 (slave 1) and M1 (slave 2), so any difference is immediately visible.
 *
 * FREE SHAFTS — no load. Steps are 1 rev.
 */
#define _GNU_SOURCE

#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <stdint.h>
#include <sched.h>
#include <sys/mman.h>
#include <math.h>

#define CYCLE_NS      1000000L
#define ENCODER_RES   131072L
#define RT_CPU        3
#define MAX_MOTORS    2
#define STEP_COUNTS   131072L   /* 1 rev */
#define SETTLE_THRESH 200       /* counts: |ferr| below this = settled */

typedef struct __attribute__((packed))
{
    uint16_t control_word;
    int32_t  target_position;
    uint16_t touch_probe;
    uint32_t physical_outputs;
} A6_RxPDO;                     /* 12 bytes */

typedef struct __attribute__((packed))
{
    uint16_t error_code;
    uint16_t status_word;
    int32_t  position_actual;
    int16_t  torque_actual;
    int32_t  following_error;
    uint16_t touch_status;
    int32_t  touch_pos1;
    int32_t  touch_pos2;
    uint32_t digital_inputs;
} A6_TxPDO;                     /* 28 bytes */

static ecx_contextt ctx;
static A6_RxPDO *rx[MAX_MOTORS];
static A6_TxPDO *tx[MAX_MOTORS];
static volatile int running = 1;

static void sighandler(int sig) { (void)sig; running = 0; }

static int a6_setup(ecx_contextt *context, uint16_t slave)
{
    int8_t   mode = 8;
    uint16_t sync_mode = 2;
    uint32_t cycle_ns = CYCLE_NS;
    ecx_SDOwrite(context, slave, 0x6060, 0, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C32, 1, FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C32, 2, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C32, 10, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C33, 1, FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C33, 2, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C33, 10, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTSAFE);
    ecx_dcsync0(context, slave, TRUE, CYCLE_NS, -97000000);
    return 1;
}

static void exchange(void) { ecx_send_processdata(&ctx); ecx_receive_processdata(&ctx, EC_TIMEOUTRET); }

static float pgain = 0.00002f, igain = 0.00002f;
static int64 syncoffset = 500000;
static void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta = (reftime - syncoffset) % cycletime;
    if (delta > (cycletime / 2)) delta -= cycletime;
    integral += -delta;
    *offsettime = (int64)((-delta * pgain) + (integral * igain));
}

/* ---- step sequence ---- */
typedef struct { int32_t target; int dwell_ms; } Seg;
static const Seg SEGS[] = {
    { 0,                1000 },
    { +STEP_COUNTS,     2000 },
    { 0,                2000 },
    { -STEP_COUNTS,     2000 },
    { 0,                2000 },
    { +STEP_COUNTS,     2000 },
    { 0,                2000 },
    { -STEP_COUNTS,     2000 },
    { 0,                2000 },
};
#define N_SEGS ((int)(sizeof(SEGS)/sizeof(SEGS[0])))

int main(int argc, char *argv[])
{
    ec_groupt *grp;
    uint8_t io_map[4096] = {0};
    int wkc = 0, n;

    if (argc != 2) { printf("Usage: sudo ./a6_step_test <ifname>\n"); return 1; }
    signal(SIGINT, sighandler);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) perror("mlockall (continuing)");
    cpu_set_t cpuset; CPU_ZERO(&cpuset); CPU_SET(RT_CPU, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == -1) perror("sched_setaffinity (continuing)");
    struct sched_param sp = { .sched_priority = 80 };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1) perror("sched_setscheduler (continuing)");

    printf("Init on %s ... ", argv[1]);
    if (!ecx_init(&ctx, argv[1])) { printf("no socket\n"); return 1; }
    if (ecx_config_init(&ctx) <= 0) { printf("no slaves\n"); ecx_close(&ctx); return 1; }
    printf("%d slave(s)\n", ctx.slavecount);
    n = ctx.slavecount;
    if (n < 2) { printf("need 2 slaves (found %d)\n", n); ecx_close(&ctx); return 1; }

    ecx_configdc(&ctx);
    osal_usleep(500000);
    for (int s = 1; s <= n; s++) ctx.slavelist[s].PO2SOconfig = a6_setup;
    ecx_config_map_group(&ctx, io_map, 0);
    grp = ctx.grouplist;
    for (int m = 0; m < n; m++) {
        rx[m] = (A6_RxPDO *)(grp->outputs + m * sizeof(A6_RxPDO));
        tx[m] = (A6_TxPDO *)(grp->inputs  + m * sizeof(A6_TxPDO));
    }

    for (int s = 1; s <= n; s++) {
        uint16_t v = 2;  ecx_SDOwrite(&ctx, s, 0x2013, 6,  FALSE, sizeof(v), &v, EC_TIMEOUTSAFE);
        v = 10;          ecx_SDOwrite(&ctx, s, 0x2013, 18, FALSE, sizeof(v), &v, EC_TIMEOUTSAFE);
        v = 20;          ecx_SDOwrite(&ctx, s, 0x2013, 3,  FALSE, sizeof(v), &v, EC_TIMEOUTSAFE);
    }
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n");

    for (int m = 0; m < n; m++) { rx[m]->control_word = 0x0000; rx[m]->target_position = tx[m]->position_actual; }
    for (int k = 0; k < 50; k++) { exchange(); osal_usleep(1000); }

    ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    printf("OPERATIONAL reached\n");
    exchange();

    int32_t start_pos[MAX_MOTORS];
    int     enabled[MAX_MOTORS] = {0, 0};
    int     fault_reset[MAX_MOTORS] = {0, 0};
    for (int m = 0; m < n; m++) start_pos[m] = tx[m]->position_actual;

    int64 toff = 0;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    next.tv_nsec = ((next.tv_nsec / 1000000) + 1) * 1000000;

    int seg = 0, seg_cycle = 0;
    int64_t max_ferr[MAX_MOTORS];
    int     last_big[MAX_MOTORS];

    printf("Beginning step-and-hold test (1 rev steps, 2 s dwells)...\n");
    while (running && seg < N_SEGS) {
        next.tv_nsec += CYCLE_NS + toff;
        if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
        if (wkc > 0) ec_sync(ctx.DCtime, CYCLE_NS, &toff);

        /* first time entering a segment: set target + reset measurement */
        if (seg_cycle == 0) {
            for (int m = 0; m < n; m++) { max_ferr[m] = 0; last_big[m] = -1; }
            printf("\n== SEG %d: target %+d counts, dwell %d ms ==\n",
                   seg, SEGS[seg].target, SEGS[seg].dwell_ms);
        }

        for (int m = 0; m < n; m++) {
            uint16_t sw = tx[m]->status_word;
            uint16_t state = sw & 0x006F;

            if (sw & 0x0008) {
                fault_reset[m] = !fault_reset[m];
                rx[m]->control_word = fault_reset[m] ? 0x0080 : 0x0000;
                enabled[m] = 0;
                continue;
            }
            if (!enabled[m]) {
                if ((sw & 0x004F) == 0x0040)      rx[m]->control_word = 0x0006;
                else if (state == 0x0021)         rx[m]->control_word = 0x0007;
                else if (state == 0x0023)         rx[m]->control_word = 0x000F;
                else if (state == 0x0027) {       enabled[m] = 1; start_pos[m] = tx[m]->position_actual; }
                continue;
            }
            /* enabled -> command the segment target (identical for both motors) */
            rx[m]->target_position = start_pos[m] + SEGS[seg].target;
            rx[m]->control_word = 0x000F;
        }

        /* measure following error */
        for (int m = 0; m < n; m++) {
            int32_t f = tx[m]->following_error;
            int64_t af = (f < 0) ? -(int64_t)f : (int64_t)f;
            if (af > max_ferr[m]) max_ferr[m] = af;
            if (af >= SETTLE_THRESH) last_big[m] = seg_cycle;
        }

        ecx_send_processdata(&ctx);

        if (seg_cycle % 200 == 0)
            printf("  +%4dms: M0 ferr=%+7d   M1 ferr=%+7d\n",
                   seg_cycle, tx[0]->following_error, tx[1]->following_error);

        seg_cycle++;
        if (seg_cycle >= SEGS[seg].dwell_ms) {
            printf("  => M0 max_ferr=%-6lld settle=%-4dms | M1 max_ferr=%-6lld settle=%-4dms\n",
                   (long long)max_ferr[0], last_big[0],
                   (long long)max_ferr[1], last_big[1]);
            seg++;
            seg_cycle = 0;
        }
    }

    printf("disabling ...\n");
    for (int m = 0; m < n; m++) rx[m]->control_word = 0x0007;
    exchange(); osal_usleep(10000);
    for (int m = 0; m < n; m++) rx[m]->control_word = 0x0000;
    exchange(); osal_usleep(10000);
    ctx.slavelist[0].state = EC_STATE_SAFE_OP; ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    ctx.slavelist[0].state = EC_STATE_PRE_OP; ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    ctx.slavelist[0].state = EC_STATE_INIT; ecx_writestate(&ctx, 0);
    ecx_close(&ctx);
    printf("done.\n");
    return 0;
}
