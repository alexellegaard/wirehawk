/* a6_sync.c — DC synchronization diagnostic for TWO daisy-chained A6 drives.
 *
 * Usage: sudo ./a6_sync <ifname>        (e.g. eth0)
 *
 * Brings both drives to OPERATIONAL with DC SYNC0 armed (motors stay DISABLED,
 * no motion), then:
 *   1. dumps the ESC DC registers of BOTH slaves (system-time offset,
 *      propagation delay, time filter, SYNC0 activation);
 *   2. samples the "System Time Difference" (0x092C, ns) of BOTH slaves at
 *      1 kHz for 5 s and reports mean / std / min / max per slave.
 *
 * 0x092C (ECT_REG_DCSYSDIFF) is the live clock error vs the reference clock.
 * A well-locked slave reads small (~tens of ns) and stable; the daisy-chained
 * slave (slave 2) is expected to show MORE jitter if its relayed DC clock sync
 * is degraded — that is the number we compare for the "slave 2 rings" issue.
 *
 * Register reads use FPRD (frame-level, no SDO), so they work from OPERATIONAL
 * exactly like the bridge. Drives are never enabled (control word stays 0,
 * target pinned to actual), so this is safe on a free bench.
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

#define CYCLE_NS   1000000L   /* 1 ms cycle */
#define RT_CPU     3          /* pair with isolcpus=3 */
#define MAX_MOTORS 2
#define NSAMPLES   5000       /* 5 s at 1 kHz */

/* ---- A6-EC predefined PDO layout (0x1701 / 0x1B01) — needed to map IO ---- */
typedef struct __attribute__((packed))
{
    uint16_t control_word;
    int32_t  target_position;
    uint16_t touch_probe;
    uint32_t physical_outputs;
} A6_RxPDO;                    /* 12 bytes */

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
} A6_TxPDO;                    /* 28 bytes */

static ecx_contextt ctx;
static A6_RxPDO *rx[MAX_MOTORS];
static A6_TxPDO *tx[MAX_MOTORS];
static volatile int running = 1;

static void sighandler(int sig)
{
    (void)sig;
    running = 0;
}

/* Pre-OP -> Safe-OP hook: CSP mode + SM-sync + arm DC SYNC0 (per slave). */
static int a6_setup(ecx_contextt *context, uint16_t slave)
{
    int8_t   mode      = 8;
    uint16_t sync_mode = 2;
    uint32_t cycle_ns  = CYCLE_NS;

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

/* DC sync PI (mirrors SOEM ec_sync) — keeps master loop aligned to reference. */
static float pgain = 0.00002f;
static float igain = 0.00002f;
static int64 syncoffset = 500000;

static void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta = (reftime - syncoffset) % cycletime;
    if (delta > (cycletime / 2))
        delta -= cycletime;
    integral += -delta;
    *offsettime = (int64)((-delta * pgain) + (integral * igain));
}

static uint32_t rdreg(uint16_t slave, uint16_t addr)
{
    uint32_t v = 0;
    ecx_FPRD(&ctx.port, ctx.slavelist[slave].configadr, addr, sizeof(v), &v, EC_TIMEOUTRET);
    return etohl(v);
}

static uint64_t rdsystime(uint16_t slave)
{
    uint64_t v = 0;
    ecx_FPRD(&ctx.port, ctx.slavelist[slave].configadr, 0x0910, sizeof(v), &v, EC_TIMEOUTRET);
    return etohll(v);
}

static void wrreg(uint16_t slave, uint16_t addr, uint32_t v)
{
    uint32_t be = htoel(v);
    ecx_FPWR(&ctx.port, ctx.slavelist[slave].configadr, addr, sizeof(be), &be, EC_TIMEOUTRET);
}

int main(int argc, char *argv[])
{
    ec_groupt *grp;
    uint8_t io_map[4096] = {0};
    int wkc = 0;
    int n;

    if (argc < 2 || argc > 3)
    {
        printf("Usage: sudo ./a6_sync <ifname> [timefilter_ns]\n");
        printf("  timefilter_ns: optional. If given, writes DC time filter (0x0934)\n");
        printf("  on BOTH slaves before measuring (default A6 value is 3072 ns).\n");
        return 1;
    }
    uint32_t timefilter = 0;
    int have_timefilter = (argc == 3);
    if (have_timefilter)
        timefilter = (uint32_t)strtoul(argv[2], NULL, 0);
    signal(SIGINT, sighandler);

    /* realtime setup */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        perror("mlockall (continuing)");
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(RT_CPU, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == -1)
        perror("sched_setaffinity (continuing)");
    struct sched_param sp = { .sched_priority = 80 };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1)
        perror("sched_setscheduler (continuing)");

    printf("Init on %s ...\n", argv[1]);
    if (!ecx_init(&ctx, argv[1])) { printf("no socket\n"); return 1; }
    if (ecx_config_init(&ctx) <= 0) { printf("no slaves\n"); ecx_close(&ctx); return 1; }
    printf("%d slave(s)\n", ctx.slavecount);
    n = ctx.slavecount;
    if (n > MAX_MOTORS) n = MAX_MOTORS;
    if (n < 2)
        printf("WARNING: only %d slave(s) — is the second drive powered and daisy-chained?\n", n);
    for (int s = 1; s <= n; s++)
        printf("  slave %d: name='%s'\n", s, ctx.slavelist[s].name);

    ecx_configdc(&ctx);
    osal_usleep(500000);

    for (int s = 1; s <= n; s++)
        ctx.slavelist[s].PO2SOconfig = a6_setup;
    ecx_config_map_group(&ctx, io_map, 0);

    grp = ctx.grouplist;
    for (int m = 0; m < n; m++)
    {
        rx[m] = (A6_RxPDO *)(grp->outputs + m * sizeof(A6_RxPDO));
        tx[m] = (A6_TxPDO *)(grp->inputs  + m * sizeof(A6_TxPDO));
    }

    for (int s = 1; s <= n; s++)
    {
        uint16_t sync_mode = 2, irq_thr = 10, sync_lost = 20;
        ecx_SDOwrite(&ctx, s, 0x2013, 6,  FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE);
        ecx_SDOwrite(&ctx, s, 0x2013, 18, FALSE, sizeof(irq_thr),   &irq_thr,   EC_TIMEOUTSAFE);
        ecx_SDOwrite(&ctx, s, 0x2013, 3,  FALSE, sizeof(sync_lost), &sync_lost, EC_TIMEOUTSAFE);
        if (have_timefilter)
        {
            wrreg(s, 0x0934, timefilter);
            printf("slave %d: DC time filter (0x0934) set to %u ns\n", s, timefilter);
        }
    }
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n");

    for (int m = 0; m < n; m++)
    {
        rx[m]->control_word    = 0x0000;
        rx[m]->target_position = tx[m]->position_actual;
    }
    for (int k = 0; k < 50; k++) { exchange(); osal_usleep(1000); }

    ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    printf("OPERATIONAL reached\n");
    exchange();

    /* ---- static DC register dump, both slaves ---- */
    printf("\n== ESC DC registers (both slaves) ==\n");
    for (int s = 1; s <= n; s++)
    {
        uint32_t offset = rdreg(s, 0x0920);   /* system time offset (signed) */
        uint32_t delay  = rdreg(s, 0x0928);   /* propagation delay (unsigned) */
        uint32_t filt   = rdreg(s, 0x0934);   /* time filter */
        uint8_t  syncact = 0;
        ecx_FPRD(&ctx.port, ctx.slavelist[s].configadr, 0x0981, sizeof(syncact), &syncact, EC_TIMEOUTRET);
        printf("  slave %d: offset=%d ns  delay=%u ns  timefilter=%u ns  syncactivate=0x%02X (SYNC0=bit0)\n",
               s, (int32_t)offset, delay, filt, syncact);
    }

    /* ---- sample 64-bit system time (0x0910) EVERY cycle; measure jitter ---- */
    printf("\nSampling system time (0x0910) at 1 kHz for %d s...\n", NSAMPLES / 1000);
    {
        int64 toff = 0;
        /* cycle-to-cycle delta stats per slave (Welford, numerically stable) */
        double dmean[MAX_MOTORS] = {0}, dm2[MAX_MOTORS] = {0};
        int64_t dmin[MAX_MOTORS], dmax[MAX_MOTORS];
        long long dcnt[MAX_MOTORS] = {0};
        /* slave2 - slave1 difference stats */
        double xmean = 0, xm2 = 0;
        int64_t xmin = 0, xmax = 0;
        long long xcnt = 0;
        uint64_t prev[MAX_MOTORS] = {0};
        int have_prev = 0;
        struct timespec next;
        clock_gettime(CLOCK_MONOTONIC, &next);
        next.tv_nsec = ((next.tv_nsec / 1000000) + 1) * 1000000;

        for (int i = 0; i < NSAMPLES && running; i++)
        {
            next.tv_nsec += CYCLE_NS + toff;
            if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

            wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
            if (wkc > 0)
                ec_sync(ctx.DCtime, CYCLE_NS, &toff);

            uint64_t now[MAX_MOTORS];
            for (int s = 1; s <= n; s++)
                now[s - 1] = rdsystime(s);

            if (have_prev)
            {
                for (int s = 1; s <= n; s++)
                {
                    int idx = s - 1;
                    int64_t delta = (int64_t)(now[idx] - prev[idx]);
                    dcnt[idx]++;
                    if (dcnt[idx] == 1) { dmin[idx] = dmax[idx] = delta; }
                    else { if (delta < dmin[idx]) dmin[idx] = delta; if (delta > dmax[idx]) dmax[idx] = delta; }
                    double dv = (double)delta;
                    double dd = dv - dmean[idx];
                    dmean[idx] += dd / (double)dcnt[idx];
                    dm2[idx] += dd * (dv - dmean[idx]);
                }
                if (n >= 2)
                {
                    int64_t diff = (int64_t)(now[1] - now[0]);
                    xcnt++;
                    if (xcnt == 1) { xmin = xmax = diff; }
                    else { if (diff < xmin) xmin = diff; if (diff > xmax) xmax = diff; }
                    double dv = (double)diff;
                    double dd = dv - xmean;
                    xmean += dd / (double)xcnt;
                    xm2 += dd * (dv - xmean);
                }
            }
            have_prev = 1;
            for (int s = 1; s <= n; s++) prev[s - 1] = now[s - 1];

            /* keep drives disabled and holding (no motion) */
            for (int m = 0; m < n; m++)
            {
                rx[m]->control_word    = 0x0000;
                rx[m]->target_position = tx[m]->position_actual;
            }
            ecx_send_processdata(&ctx);
        }

        printf("\n== Cycle-to-cycle system-time delta (expect ~1,000,000 ns/cycle) ==\n");
        printf("  slave |    mean |     std |     min |     max  (ns)\n");
        for (int s = 1; s <= n; s++)
        {
            int idx = s - 1;
            if (dcnt[idx] == 0) { printf("  %5d | (no samples)\n", s); continue; }
            double var = dm2[idx] / (double)dcnt[idx];
            if (var < 0) var = 0;
            printf("  %5d | %9.0f | %9.0f | %9lld | %9lld\n",
                   s, dmean[idx], sqrt(var), (long long)dmin[idx], (long long)dmax[idx]);
        }

        if (n >= 2)
        {
            double var = xm2 / (double)xcnt;
            if (var < 0) var = 0;
            printf("\n== slave2 - slave1 system-time difference ==\n");
            printf("  mean=%9.0f  std=%9.0f  min=%9lld  max=%9lld  (ns)\n",
                   xmean, sqrt(var), (long long)xmin, (long long)xmax);
        }
    }

    /* ---- clean shutdown (drive never enabled, so just walk states down) ---- */
    printf("\ndisabling ...\n");
    for (int m = 0; m < n; m++) rx[m]->control_word = 0x0000;
    exchange();
    osal_usleep(10000);
    ctx.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    ctx.slavelist[0].state = EC_STATE_PRE_OP;
    ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    ctx.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&ctx, 0);
    ecx_close(&ctx);
    printf("done.\n");
    return 0;
}
