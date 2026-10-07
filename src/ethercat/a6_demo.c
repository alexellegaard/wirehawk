/* a6_demo.c — motor identification + synchronized sine-wave demo.
 *
 * Usage: sudo ./a6_demo <ifname>
 *
 * Two phases, both on free shafts (no load):
 *
 *   Phase 1 — identify: each motor does one revolution (out and back) in
 *             order, motor 1 -> 4, while the others hold still. Use this to
 *             map which physical motor is which slave (handy when the cables
 *             are tangled).
 *
 *   Phase 2 — synchronized wave: a smooth phase-offset sine across all motors
 *             (a travelling wave), +/- 0.5 rev at 0.5 Hz, 1 s cosine ramp-in.
 *
 * Both phases use a sync barrier first (wait until EVERY drive is "operation
 * enabled") so the motion starts for all motors on the same cycle. Runs until
 * Ctrl-C. Same DC ordering as a6_step_test.
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

#define CYCLE_NS      4000000L   /* 4 ms = 250 Hz, matches the bridge */
#define CYCLE_MS      (CYCLE_NS / 1000000L)   /* 4 ms/cycle */
#define ENCODER_RES   131072L
#define RT_CPU        3
#define MAX_MOTORS    4

#define DEMO_AMP      65536.0    /* +/- 0.5 rev (wave phase) */
#define DEMO_FREQ_HZ  0.5
#define RAMP_IN_S     1.0        /* cosine ramp-in over 1 s */

/* identification phase timing (per motor) */
#define IDENT_MS      1000       /* ramp 1 rev over 1 s */
#define IDENT_HOLD    800        /* hold at +1 rev */
#define IDENT_PAUSE   400        /* pause before the next motor */

/* ---- A6-EC predefined PDO layout (0x1701 / 0x1B01) ---- */
typedef struct __attribute__((packed))
{
    uint16_t control_word;
    int32_t  target_position;
    uint16_t touch_probe;
    uint32_t physical_outputs;
} A6_RxPDO;

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
} A6_TxPDO;

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
    ecx_SDOwrite(context, slave, 0x1C32, 1,  FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C32, 2,  FALSE, sizeof(cycle_ns),  &cycle_ns,  EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C32, 10, FALSE, sizeof(cycle_ns),  &cycle_ns,  EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C33, 1,  FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C33, 2,  FALSE, sizeof(cycle_ns),  &cycle_ns,  EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C33, 10, FALSE, sizeof(cycle_ns),  &cycle_ns,  EC_TIMEOUTSAFE);
    ecx_dcsync0(context, slave, TRUE, CYCLE_NS, -97000000);
    return 1;
}

static int exchange(void)
{
    ecx_send_processdata(&ctx);
    return ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
}

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

/* Per-cycle drive status handling. Returns 1 if the drive is enabled and ready
 * for a motion target (caller then sets target + cw=0x000F); otherwise writes
 * the enable/fault handshake with the target pinned to actual and returns 0.
 * start_pos is (re)latched to the live actual on the cycle the drive enables. */
static int drive_ready(int m, int *enabled, int32_t *start_pos)
{
    uint16_t sw = tx[m]->status_word, state = sw & 0x006F;
    if (sw & 0x0008)
    {
        rx[m]->control_word = 0x0080;          /* hold fault reset steady */
        *enabled = 0;
        rx[m]->target_position = tx[m]->position_actual;
        return 0;
    }
    if (!*enabled)
    {
        rx[m]->target_position = tx[m]->position_actual;   /* pin: no jump on enable */
        if ((sw & 0x004F) == 0x0040)      rx[m]->control_word = 0x0006;
        else if (state == 0x0021)         rx[m]->control_word = 0x0007;
        else if (state == 0x0023)         rx[m]->control_word = 0x000F;
        else if (state == 0x0027)         { *enabled = 1; *start_pos = tx[m]->position_actual; rx[m]->control_word = 0x000F; }
        return 0;
    }
    return 1;
}

int main(int argc, char *argv[])
{
    ec_groupt *grp;
    uint8_t io_map[4096] = {0};
    int wkc = 0, n;

    if (argc != 2) { printf("Usage: sudo ./a6_demo <ifname>\n"); return 1; }
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
    n = (ctx.slavecount > MAX_MOTORS) ? MAX_MOTORS : ctx.slavecount;

    ecx_configdc(&ctx);
    osal_usleep(500000);
    for (int s = 1; s <= n; s++) ctx.slavelist[s].PO2SOconfig = a6_setup;
    ecx_config_map_group(&ctx, io_map, 0);
    grp = ctx.grouplist;
    for (int m = 0; m < n; m++)
    {
        rx[m] = (A6_RxPDO *)(grp->outputs + m * sizeof(A6_RxPDO));
        tx[m] = (A6_TxPDO *)(grp->inputs  + m * sizeof(A6_TxPDO));
    }

    for (int s = 1; s <= n; s++)
    {
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

    int     enabled[MAX_MOTORS] = {0};
    int32_t start_pos[MAX_MOTORS] = {0};
    double  phase[MAX_MOTORS];
    int64 toff = 0;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    next.tv_nsec = ((next.tv_nsec / 1000000) + 1) * 1000000;

    /* ---- enable barrier: wait until ALL drives are operation-enabled ---- */
    printf("Enabling %d motor(s)...\n", n);
    int all_enabled = 0;
    {
        int watchdog = 0;
        while (running && !all_enabled && watchdog < 5000)
        {
            next.tv_nsec += CYCLE_NS + toff;
            if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
            wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
            if (wkc > 0) ec_sync(ctx.DCtime, CYCLE_NS, &toff);

            all_enabled = 1;
            for (int m = 0; m < n; m++)
            {
                if (drive_ready(m, &enabled[m], &start_pos[m]))
                {
                    rx[m]->target_position = start_pos[m];   /* hold until all enabled */
                    rx[m]->control_word = 0x000F;
                }
                if (!enabled[m]) all_enabled = 0;
            }
            ecx_send_processdata(&ctx);
            watchdog++;
        }
        if (!all_enabled) printf("enable timeout\n");
    }

    /* ---- Phase 1: identify each motor — one revolution, motor 1 -> 4 ---- */
    if (all_enabled)
    {
        long id_cycles = (IDENT_MS + IDENT_HOLD + IDENT_MS + IDENT_PAUSE) / CYCLE_MS;
        for (int m = 0; m < n && running; m++)
        {
            int32_t base = start_pos[m];
            printf(">>> identify motor %d (slave %d): one revolution\n", m + 1, m + 1);
            for (long c = 0; c < id_cycles && running; c++)
            {
                next.tv_nsec += CYCLE_NS + toff;
                if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
                wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
                if (wkc > 0) ec_sync(ctx.DCtime, CYCLE_NS, &toff);

                long ms = c * CYCLE_MS;
                double off;
                if      (ms < IDENT_MS)                 off = (double)ms / IDENT_MS;
                else if (ms < IDENT_MS + IDENT_HOLD)   off = 1.0;
                else if (ms < 2*IDENT_MS + IDENT_HOLD) off = 1.0 - (double)(ms - IDENT_MS - IDENT_HOLD) / IDENT_MS;
                else                                   off = 0.0;

                for (int k = 0; k < n; k++)
                {
                    if (!drive_ready(k, &enabled[k], &start_pos[k])) continue;
                    if (k == m) rx[k]->target_position = base + (int32_t)(off * ENCODER_RES);
                    else        rx[k]->target_position = start_pos[k];
                    rx[k]->control_word = 0x000F;
                }
                ecx_send_processdata(&ctx);
            }
        }
        printf("identification done\n");
    }

    /* ---- Phase 2: synchronized travelling wave (quarter-cycle per motor) ---- */
    if (all_enabled && running)
    {
        for (int m = 0; m < n; m++)
        {
            start_pos[m] = tx[m]->position_actual;
            phase[m] = m * (M_PI / 2.0);
        }
        printf("running synchronized wave (Ctrl-C to stop)\n");

        long t = 0;
        while (running)
        {
            next.tv_nsec += CYCLE_NS + toff;
            if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
            wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
            if (wkc > 0) ec_sync(ctx.DCtime, CYCLE_NS, &toff);

            double sec  = t * (CYCLE_NS / 1e9);
            double ramp = (sec < RAMP_IN_S) ? (1.0 - cos(M_PI * sec / RAMP_IN_S)) / 2.0 : 1.0;
            double amp  = DEMO_AMP * ramp;

            for (int m = 0; m < n; m++)
            {
                if (!drive_ready(m, &enabled[m], &start_pos[m])) continue;
                rx[m]->target_position = start_pos[m] + (int32_t)(amp * sin(2.0 * M_PI * DEMO_FREQ_HZ * sec - phase[m]));
                rx[m]->control_word = 0x000F;
            }
            ecx_send_processdata(&ctx);

            if (t % 500 == 0)
            {
                printf("  t=%5.1fs:", sec);
                for (int m = 0; m < n; m++)
                    printf("  M%d pos=%+8d ferr=%+7d", m, (int)(tx[m]->position_actual - start_pos[m]), tx[m]->following_error);
                printf("\n");
            }
            t++;
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
