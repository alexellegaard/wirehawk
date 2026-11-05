/* a6_temp_stresstest.c — heat stress test for the drive enclosure.
 *
 * Primary heat source is the ENABLED drive itself: the fan, the power-stage
 * idle/switching losses, and the small holding current — all present even at
 * zero motor speed (on the order of 10–20 W before the motor turns). The
 * back-and-forth motion is a gentle add-on (accel-torque I²R) that also keeps
 * the drive clearly active. So the real job here is: enable the drive, keep it
 * active, and log the temperature rise.
 *
 * Motion: a repeating triangle-VELOCITY profile (accelerate 0→+V, decelerate
 * +V→0, accelerate 0→−V, decelerate −V→0, repeat). |torque| ≈ constant at
 * J·accel. Keep accel in the gentle range — ~20–40 rev/s²; >45 rev/s² is where
 * a free shaft starts to clank/shake the table. Speed amplitude sets the swing
 * (V²/(2a) rev per half-cycle); keep it low on a spool so the rope barely pays out.
 *
 * Logging:
 *   - every 10 s: append one row to a CSV (elapsed_s, temp per motor in °C)
 *   - every 60 s: print a line to the terminal
 *
 * Usage:
 *   sudo ./a6_temp_stresstest <ifname> <speed_rev_s> <accel_rev_s2> [duration_s]
 *     duration_s 0 (or omitted) = run until Ctrl-C
 *
 * Example (gentle compromise):
 *   sudo ./a6_temp_stresstest eth0 10 30
 *   sudo ./a6_temp_stresstest eth0 10 30 1800   # 30 min then stop
 *
 * The CSV lands in the current directory as temp_stresstest_YYYYmmdd_HHMMSS.csv.
 *
 * CSP + DC sync, all slaves commanded identically.
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

static int sdo_read16(uint16_t slave, uint16_t idx, uint8_t sub, uint32_t *v)
{
    int sz = 2;
    *v = 0;
    return ecx_SDOread(&ctx, slave, idx, sub, FALSE, &sz, v, EC_TIMEOUTSAFE) > 0;
}

int main(int argc, char *argv[])
{
    ec_groupt *grp;
    uint8_t io_map[4096] = {0};
    int wkc = 0, n;
    FILE *logfile = NULL;

    if (argc < 4 || argc > 5)
    {
        printf("Usage: sudo ./a6_temp_stresstest <ifname> <speed_rev_s> <accel_rev_s2> [duration_s]\n");
        return 1;
    }
    double speed_rev_s  = atof(argv[2]);
    double accel_rev_s2 = atof(argv[3]);
    long   duration_s   = (argc >= 5) ? atol(argv[4]) : 0;
    if (speed_rev_s <= 0 || accel_rev_s2 <= 0)
    {
        printf("speed and accel must be > 0\n");
        return 1;
    }
    signal(SIGINT, sighandler);

    /* convert to counts/cycle and counts/cycle^2 */
    double cyc_s      = CYCLE_NS / 1e9;
    double speed_cpc  = speed_rev_s  * ENCODER_RES * cyc_s;            /* counts/cycle */
    double accel_cpc2 = accel_rev_s2 * ENCODER_RES * cyc_s * cyc_s;    /* counts/cycle^2 */
    long   ramp_cycles   = (long)ceil(speed_cpc / accel_cpc2);         /* V/a per phase */
    long   period_cycles = 4 * ramp_cycles;
    long   read_every    = (long)lround(1.0 / cyc_s);                  /* 1 s */
    long   log_every     = (long)lround(10.0 / cyc_s);                 /* 10 s */
    long   print_every   = (long)lround(60.0 / cyc_s);                 /* 60 s */

    printf("profile: triangle velocity +/-%g rev/s (%.1f rpm) at %g rev/s^2\n",
           speed_rev_s, speed_rev_s * 60.0, accel_rev_s2);
    printf("         swing %.2f rev per half-cycle | period %.2f s | read %ld s / log %ld s / print %ld s\n",
           speed_rev_s * speed_rev_s / (2.0 * accel_rev_s2),
           period_cycles * cyc_s, read_every, log_every, print_every);
    if (duration_s > 0)
        printf("         duration %ld s (%.1f min)\n", duration_s, duration_s / 60.0);
    else
        printf("         run until Ctrl-C\n");

    /* open the temperature log */
    {
        char logpath[256];
        time_t now = time(NULL);
        struct tm lt;
        localtime_r(&now, &lt);
        strftime(logpath, sizeof(logpath), "temp_stresstest_%Y%m%d_%H%M%S.csv", &lt);
        logfile = fopen(logpath, "w");
        if (!logfile) { printf("WARNING: cannot open log file '%s' — logging disabled\n", logpath); }
        else { printf("logging to %s\n", logpath); }
    }

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

    /* relax sync-loss tolerance so the periodic SDO reads don't fault */
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
    int64_t cmd_pos[MAX_MOTORS];

    int64 toff = 0;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    next.tv_nsec = ((next.tv_nsec / 1000000) + 1) * 1000000;

    /* Enable: pin target to live actual, step to operation enabled. */
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
                    rx[m]->control_word = 0x0080;
                    enabled[m] = 0;
                    all_enabled = 0;
                    continue;
                }
                rx[m]->target_position = tx[m]->position_actual;
                if ((sw & 0x004F) == 0x0040)      rx[m]->control_word = 0x0006;
                else if (state == 0x0021)         rx[m]->control_word = 0x0007;
                else if (state == 0x0023)         rx[m]->control_word = 0x000F;
                else if (state == 0x0027) { enabled[m] = 1; cmd_pos[m] = tx[m]->position_actual; rx[m]->control_word = 0x000F; }
                if (!enabled[m]) all_enabled = 0;
            }
            ecx_send_processdata(&ctx);
            enable_watchdog++;
        }
        if (!all_enabled)
            printf("enable timeout — some drive never reached operation enabled\n");
        else
            printf("all %d motor(s) enabled — heat run started\n", n);
    }

    /* CSV header */
    if (logfile)
    {
        fprintf(logfile, "elapsed_s");
        for (int m = 0; m < n; m++) fprintf(logfile, ",motor%d_temp_C", m);
        fprintf(logfile, "\n");
        fflush(logfile);
    }

    double last_temp[MAX_MOTORS] = {0};
    long i = 0;
    long total_cycles = (duration_s > 0) ? (long)(duration_s / cyc_s) : 0;
    if (all_enabled)
        while (running && (duration_s <= 0 || i < total_cycles))
    {
        next.tv_nsec += CYCLE_NS + toff;
        if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
        if (wkc > 0) ec_sync(ctx.DCtime, CYCLE_NS, &toff);

        /* triangle velocity this cycle: 0→+V → +V→0 → 0→−V → −V→0 */
        long ph = i % period_cycles;
        double vdes = 0;
        if      (ph < ramp_cycles)                 vdes =  accel_cpc2 * (ph + 1);
        else if (ph < 2 * ramp_cycles)             vdes =  speed_cpc - accel_cpc2 * (ph - ramp_cycles + 1);
        else if (ph < 3 * ramp_cycles)             vdes = -accel_cpc2 * (ph - 2 * ramp_cycles + 1);
        else                                       vdes = -speed_cpc + accel_cpc2 * (ph - 3 * ramp_cycles + 1);

        for (int m = 0; m < n; m++)
        {
            uint16_t sw = tx[m]->status_word, state = sw & 0x006F;
            if (sw & 0x0008)
            {
                rx[m]->control_word = 0x0080;
                enabled[m] = 0;
                continue;
            }
            if (!enabled[m])
            {
                rx[m]->target_position = tx[m]->position_actual;
                if ((sw & 0x004F) == 0x0040) rx[m]->control_word = 0x0006;
                else if (state == 0x0021)    rx[m]->control_word = 0x0007;
                else if (state == 0x0023)    rx[m]->control_word = 0x000F;
                else if (state == 0x0027) { enabled[m] = 1; cmd_pos[m] = tx[m]->position_actual; }
                continue;
            }

            cmd_pos[m] += (int64_t)llround(vdes);
            rx[m]->target_position = (int32_t)cmd_pos[m];
            rx[m]->control_word = 0x000F;
        }
        ecx_send_processdata(&ctx);

        /* temperature read (1 s) → log (10 s) → print (60 s) */
        if (i % read_every == 0)
        {
            for (int m = 0; m < n; m++)
            {
                uint32_t raw = 0;
                if (sdo_read16((uint16_t)(m + 1), 0x2040, 0x31, &raw))
                {
                    int16_t r = (int16_t)(raw & 0xFFFF);
                    last_temp[m] = r / 10.0;
                }
            }

            if (i % log_every == 0 && logfile)
            {
                fprintf(logfile, "%.1f", i * cyc_s);
                for (int m = 0; m < n; m++) fprintf(logfile, ",%.1f", last_temp[m]);
                fprintf(logfile, "\n");
                fflush(logfile);
            }

            if (i % print_every == 0)
            {
                printf("t=%4.0fs |", i * cyc_s);
                for (int m = 0; m < n; m++) printf(" M%d=%5.1fC", m, last_temp[m]);
                printf("\n");
                fflush(stdout);
            }
        }

        i++;
    }

    printf("\nstopping ...\n");
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
    if (logfile) fclose(logfile);
    printf("done.\n");
    return 0;
}
