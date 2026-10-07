/* a6_profile_test.c — trapezoidal move at CONFIGURABLE speed/accel, to test whether
 * the bench wobble follows SPEED/ACCEL (drive/topology) or the FRAMEWORK (profile shape).
 *
 * Usage: sudo ./a6_profile_test <ifname> <speed_rev_s> <accel_rev_s2> [coast_ms] [dwell_ms]
 *   gentle    : sudo ./a6_profile_test eth0 1 10
 *   aggressive: sudo ./a6_profile_test eth0 45 200
 *
 * Profile (same shape as the ROS bridge trapezoid): ramp 0->speed at accel,
 * coast at speed, ramp speed->0 at accel, dwell. Cycle = 4 ms (250 Hz, matching
 * the bridge). CSP + DC sync, BOTH slaves commanded identically.
 *
 * During the COAST it accumulates velocity (delta-position/cycle) mean/std/min/max
 * for M0 (slave 1) and M1 (slave 2) — the std is the constant-motion chatter.
 * During the DWELL it reports following error (settle ringing). A live trace
 * prints every ~200 ms so you can watch it.
 *
 * FREE SHAFTS — no load. Ramp is bounded by <accel> (no instant steps).
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

#define CYCLE_NS      4000000L   /* 4 ms = 250 Hz, matches the ROS bridge */
#define ENCODER_RES   131072L    /* 17-bit encoder, counts/rev */
#define RT_CPU        3
#define MAX_MOTORS    4
#define HALF_WRAP     0x00800000 /* guard: half of +/-160 rev wrap */

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

/* Welford accumulators for velocity (counts/cycle) during coast */
typedef struct { double mean, m2; int64_t min, max; long n; } VStat;
static void vstat_add(VStat *s, double v)
{
    s->n++;
    double d = v - s->mean;
    s->mean += d / s->n;
    s->m2 += d * (v - s->mean);
    if (s->n == 1) { s->min = (int64_t)v; s->max = (int64_t)v; }
    else { if (v < s->min) s->min = (int64_t)v; if (v > s->max) s->max = (int64_t)v; }
}

int main(int argc, char *argv[])
{
    ec_groupt *grp;
    uint8_t io_map[4096] = {0};
    int wkc = 0, n;

    if (argc < 4 || argc > 6)
    {
        printf("Usage: sudo ./a6_profile_test <ifname> <speed_rev_s> <accel_rev_s2> [coast_ms] [dwell_ms]\n");
        return 1;
    }
    double speed_rev_s = atof(argv[2]);
    double accel_rev_s2 = atof(argv[3]);
    int    coast_ms   = (argc >= 5) ? atoi(argv[4]) : 2000;
    int    dwell_ms   = (argc >= 6) ? atoi(argv[5]) : 2000;
    if (speed_rev_s <= 0 || accel_rev_s2 <= 0)
    {
        printf("speed and accel must be > 0\n");
        return 1;
    }
    signal(SIGINT, sighandler);

    /* convert to counts/cycle and counts/cycle^2 */
    double cyc_s  = CYCLE_NS / 1e9;
    double speed_cpc  = speed_rev_s  * ENCODER_RES * cyc_s;      /* counts/cycle */
    double accel_cpc2 = accel_rev_s2 * ENCODER_RES * cyc_s * cyc_s; /* counts/cycle^2 */
    long   ramp_cycles = (long)ceil(speed_cpc / accel_cpc2);
    long   coast_cycles = coast_ms / 4;
    long   down_cycles  = ramp_cycles;
    long   dwell_cycles = dwell_ms / 4;
    long   total = ramp_cycles + coast_cycles + down_cycles + dwell_cycles;

    printf("profile: %g rev/s (%.1f rpm), %g rev/s^2 | cycle 4 ms | ramp %ld, coast %ld, dwell %ld cycles\n",
           speed_rev_s, speed_rev_s * 60.0, accel_rev_s2, ramp_cycles, coast_cycles, dwell_cycles);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) perror("mlockall (continuing)");
    cpu_set_t cpuset; CPU_ZERO(&cpuset); CPU_SET(RT_CPU, &cpuset);
    sched_setaffinity(0, sizeof(cpuset), &cpuset);
    struct sched_param sp = { .sched_priority = 80 };
    sched_setscheduler(0, SCHED_FIFO, &sp);

    printf("Init on %s ... ", argv[1]);
    if (!ecx_init(&ctx, argv[1])) { printf("no socket\n"); return 1; }
    if (ecx_config_init(&ctx) <= 0) { printf("no slaves\n"); ecx_close(&ctx); return 1; }
    printf("%d slave(s)\n", ctx.slavecount);
    n = ctx.slavecount;
    if (n > MAX_MOTORS) n = MAX_MOTORS;

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
    int64_t cmd_pos[MAX_MOTORS];       /* running commanded position */
    int32_t prev_pos[MAX_MOTORS];
    VStat   vst[MAX_MOTORS] = { {0} };
    int64_t max_ferr[MAX_MOTORS] = {0};
    int32_t last_vel[MAX_MOTORS] = {0};

    int64 toff = 0;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    next.tv_nsec = ((next.tv_nsec / 1000000) + 1) * 1000000;

    /* Enable phase: bring EVERY drive to "operation enabled" while pinning its
     * target to the live actual. The motion clock does NOT start until all
     * drives are enabled — otherwise the first drive to enable joins the
     * velocity profile mid-ramp and starts earlier than the rest (and they
     * finish at different points). */
    printf("Enabling %d motor(s)...\n", n);
    int all_enabled = 0;
    {
        int enable_watchdog = 0;
        while (running && !all_enabled && enable_watchdog < 5000)
        {
            next.tv_nsec += CYCLE_NS + toff;
            if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

            wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
            if (wkc > 0) ec_sync(ctx.DCtime, CYCLE_NS, &toff);

            all_enabled = 1;
            for (int m = 0; m < n; m++)
            {
                uint16_t sw = tx[m]->status_word, state = sw & 0x006F;
                if (sw & 0x0008)
                {
                    rx[m]->control_word = 0x0080;   /* hold fault reset steady */
                    enabled[m] = 0;
                    all_enabled = 0;
                    continue;
                }
                rx[m]->target_position = tx[m]->position_actual;   /* pin: no jump on enable */
                if ((sw & 0x004F) == 0x0040)      rx[m]->control_word = 0x0006;
                else if (state == 0x0021)         rx[m]->control_word = 0x0007;
                else if (state == 0x0023)         rx[m]->control_word = 0x000F;
                else if (state == 0x0027) { enabled[m] = 1; cmd_pos[m] = tx[m]->position_actual; prev_pos[m] = tx[m]->position_actual; rx[m]->control_word = 0x000F; }
                if (!enabled[m]) all_enabled = 0;
            }
            ecx_send_processdata(&ctx);
            enable_watchdog++;
        }
        if (!all_enabled)
            printf("enable timeout — some drive never reached operation enabled\n");
        else
            printf("all %d motor(s) enabled — starting synchronized move\n", n);
    }

    long i = 0;
    if (all_enabled)
        while (running && i < total)
    {
        next.tv_nsec += CYCLE_NS + toff;
        if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
        if (wkc > 0) ec_sync(ctx.DCtime, CYCLE_NS, &toff);

        /* phase + desired velocity this cycle */
        double vdes = 0;
        if      (i < ramp_cycles)                         vdes = accel_cpc2 * (i + 1);
        else if (i < ramp_cycles + coast_cycles)          vdes = speed_cpc;
        else if (i < ramp_cycles + coast_cycles + down_cycles) vdes = speed_cpc - accel_cpc2 * (i - ramp_cycles - coast_cycles + 1);
        else                                              vdes = 0;
        if (vdes < 0) vdes = 0;

        for (int m = 0; m < n; m++)
        {
            uint16_t sw = tx[m]->status_word, state = sw & 0x006F;
            if (sw & 0x0008)
            {
                rx[m]->control_word = 0x0080;   /* hold fault reset steady */
                enabled[m] = 0;
                continue;
            }
            if (!enabled[m])
            {
                rx[m]->target_position = tx[m]->position_actual;
                if ((sw & 0x004F) == 0x0040) rx[m]->control_word = 0x0006;
                else if (state == 0x0021)    rx[m]->control_word = 0x0007;
                else if (state == 0x0023)    rx[m]->control_word = 0x000F;
                else if (state == 0x0027) { enabled[m] = 1; cmd_pos[m] = tx[m]->position_actual; prev_pos[m] = tx[m]->position_actual; }
                continue;
            }

            cmd_pos[m] += (int64_t)llround(vdes);
            rx[m]->target_position = (int32_t)cmd_pos[m];
            rx[m]->control_word = 0x000F;

            /* velocity = delta-position/cycle, wrap-guarded */
            int32_t pa = tx[m]->position_actual;
            int32_t dp = pa - prev_pos[m];
            if (dp > HALF_WRAP) dp -= 2 * HALF_WRAP;
            else if (dp < -HALF_WRAP) dp += 2 * HALF_WRAP;
            prev_pos[m] = pa;
            last_vel[m] = dp;

            if (i >= ramp_cycles && i < ramp_cycles + coast_cycles)
                vstat_add(&vst[m], (double)dp);

            if (i >= ramp_cycles + coast_cycles + down_cycles)
            {
                int64_t af = (tx[m]->following_error < 0) ? -(int64_t)tx[m]->following_error : tx[m]->following_error;
                if (af > max_ferr[m]) max_ferr[m] = af;
            }
        }
        ecx_send_processdata(&ctx);

        if (i >= ramp_cycles && i < ramp_cycles + coast_cycles && (i % 50) == 0)
        {
            printf("  coast +%ld ms:", (i - ramp_cycles) * 4);
            for (int m = 0; m < n; m++)
                printf("  M%d vel=%+8d ferr=%+7d", m, last_vel[m], tx[m]->following_error);
            printf("\n");
        }

        i++;
    }

    printf("\n== summary (velocity = counts/cycle during coast; expect ~%.0f) ==\n", speed_cpc);
    for (int m = 0; m < n; m++)
    {
        double std = (vst[m].n > 1) ? sqrt(vst[m].m2 / (vst[m].n - 1)) : 0;
        double rel = (vst[m].mean != 0) ? std / fabs(vst[m].mean) : 0;
        printf("  M%d: mean=%.1f std=%.1f min=%lld max=%lld (n=%ld) rel=%.2f | dwell peak ferr=%lld\n",
               m, vst[m].mean, std, (long long)vst[m].min, (long long)vst[m].max, vst[m].n, rel, (long long)max_ferr[m]);
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
