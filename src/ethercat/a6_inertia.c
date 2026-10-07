/* a6_inertia.c — read/set the A6 load-inertia ratio, and trigger the offline
 * inertia auto-tune (the "tell the drive it has no payload" tool).
 *
 * Modes:
 *   sudo ./a6_inertia <ifname>              # read C00.04/05/06 + C07.01-04 (Safe-OP, no motion)
 *   sudo ./a6_inertia <ifname> set <ratio>  # write C00.06 = <ratio> 0..12000 (Safe-OP, no motion)
 *   sudo ./a6_inertia <ifname> tune         # offline inertia auto-tune (OP; MOVES THE MOTOR)
 *
 * Parameter map (manual Cxx.yy = SDO 0x20xx:(yy+1), A6-EC):
 *   C00.04 = 0x2000:05  Auto-tuning mode        0:manual 1:standard 2:positioning (def 1)
 *   C00.05 = 0x2000:06  Stiffness level         1..31 (def 12)
 *   C00.06 = 0x2000:07  Load inertia ratio      0..12000 % (def 100 = 1:1 load; ~0 = free shaft)
 *   C07.00 = 0x2007:01  Offline auto-tune mode  0..785 (def 769)
 *   C07.01 = 0x2007:02  Auto-tune speed ref     50..8000 rpm (def 500)
 *   C07.02 = 0x2007:03  Auto-tune accel time    0..65535 ms (def 100)
 *   C07.03 = 0x2007:04  Auto-tune target torque 1..1500 (0.1%) (def 150)
 *   C07.04 = 0x2007:05  Auto-tune revolutions   10..65535 (0.01 rev) (def 200)
 *   F30.10 = 0x2030:11  Inertia auto-tune select 0:disabled 1:enable (trigger)
 *
 * WHY: on a bare shaft the factory inertia ratio 100% assumes a 1:1 load, so the
 * drive's auto-computed gains are ~2x too stiff -> the underdamped "wobble" seen
 * on BOTH motors. Setting C00.06 ~= 0 (or letting the drive measure it via the
 * auto-tune) tells it there is no payload. In Standard mode (C00.04=1) the drive
 * re-computes Kp/Kv itself; you do NOT hand-tune the gains.
 *
 * The `tune` mode enables the servo and the DRIVE runs its own motion profile
 * (forward/reverse a few revs at C07.01/C07.02) — gentle and controlled, but it
 * DOES spin the shafts. Alex runs this himself.
 */
#define _GNU_SOURCE

#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>

#define CYCLE_NS      1000000L
#define MAX_MOTORS    4

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

/* ---- SDO helpers (U16) ---- */
static int rd16(uint16_t slave, uint16_t idx, uint8_t sub, uint16_t *v)
{
    int sz = sizeof(*v); *v = 0;
    return ecx_SDOread(&ctx, slave, idx, sub, FALSE, &sz, v, EC_TIMEOUTSAFE) > 0;
}
static int wr16(uint16_t slave, uint16_t idx, uint8_t sub, uint16_t v)
{
    int sz = sizeof(v);
    return ecx_SDOwrite(&ctx, slave, idx, sub, FALSE, sz, &v, EC_TIMEOUTSAFE) > 0;
}

/* ---- Pre-OP hook (tune mode only): CSP + SM-sync + arm DC SYNC0 ---- */
static int a6_setup(ecx_contextt *context, uint16_t slave)
{
    int8_t   mode      = 8;
    uint16_t sync_mode = 2;
    uint32_t cycle_ns  = CYCLE_NS;
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

/* ---- Safe-OP modes: read / set (no motion) ---- */
static void print_param(const char *name, uint16_t idx, uint8_t sub)
{
    printf("  %-32s", name);
    for (int s = 1; s <= ctx.slavecount; s++)
    {
        uint16_t v = 0;
        rd16(s, idx, sub, &v);
        printf("  S%d=%u", s, v);
    }
    printf("\n");
}

static int do_read(const char *ifname)
{
    printf("Init on %s ...\n", ifname);
    if (!ecx_init(&ctx, ifname)) { printf("no socket\n"); return 1; }
    if (ecx_config_init(&ctx) <= 0) { printf("no slaves\n"); ecx_close(&ctx); return 1; }
    printf("%d slave(s)\n", ctx.slavecount);
    uint8_t io_map[4096] = {0};
    ecx_config_map_group(&ctx, io_map, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n\n");

    printf("== gain auto-tuning (C00 group) ==\n");
    print_param("Auto-tuning mode (C00.04)", 0x2000, 5);
    print_param("Stiffness level (C00.05)",   0x2000, 6);
    print_param("Load inertia ratio % (C00.06)", 0x2000, 7);

    printf("\n== offline inertia auto-tune config (C07 group) ==\n");
    print_param("Auto-tune mode (C07.00)",      0x2007, 1);
    print_param("Auto-tune speed rpm (C07.01)", 0x2007, 2);
    print_param("Auto-tune accel ms (C07.02)",  0x2007, 3);
    print_param("Auto-tune torque 0.1%% (C07.03)", 0x2007, 4);
    print_param("Auto-tune revs 0.01r (C07.04)", 0x2007, 5);

    ecx_close(&ctx);
    return 0;
}

static int do_write(const char *ifname, uint16_t idx, uint8_t sub, const char *name, uint16_t value)
{
    printf("Init on %s ...\n", ifname);
    if (!ecx_init(&ctx, ifname)) { printf("no socket\n"); return 1; }
    if (ecx_config_init(&ctx) <= 0) { printf("no slaves\n"); ecx_close(&ctx); return 1; }
    printf("%d slave(s)\n", ctx.slavecount);
    uint8_t io_map[4096] = {0};
    ecx_config_map_group(&ctx, io_map, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n\n");

    printf("Writing %s = %u to ALL slaves ...\n", name, value);
    for (int s = 1; s <= ctx.slavecount; s++)
    {
        uint16_t before = 0, after = 0;
        rd16(s, idx, sub, &before);
        int ok = wr16(s, idx, sub, value);
        rd16(s, idx, sub, &after);
        printf("  slave %d: before=%u write=%s after=%u\n", s, before, ok ? "OK" : "FAIL", after);
    }

    ecx_close(&ctx);
    return 0;
}

/* stiffness level (C00.05) is the gain-aggressiveness knob in Standard auto-tune
 * mode — lower it to soften the gains and damp free-shaft vibration. */
static int do_stiff(const char *ifname, uint16_t level)
{
    return do_write(ifname, 0x2000, 6, "stiffness level C00.05", level);
}

/* ---- tune mode: OP + enable + trigger F30.10 + wait + readback + disable ---- */
static int do_tune(const char *ifname)
{
    ec_groupt *grp;
    uint8_t io_map[4096] = {0};
    int wkc = 0, n;

    signal(SIGINT, sighandler);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) perror("mlockall");
    cpu_set_t cpuset; CPU_ZERO(&cpuset); CPU_SET(3, &cpuset);
    sched_setaffinity(0, sizeof(cpuset), &cpuset);
    struct sched_param sp = { .sched_priority = 80 };
    sched_setscheduler(0, SCHED_FIFO, &sp);

    printf("Init on %s ... ", ifname);
    if (!ecx_init(&ctx, ifname)) { printf("no socket\n"); return 1; }
    if (ecx_config_init(&ctx) <= 0) { printf("no slaves\n"); ecx_close(&ctx); return 1; }
    n = (ctx.slavecount > MAX_MOTORS) ? MAX_MOTORS : ctx.slavecount;
    printf("%d slave(s)\n", ctx.slavecount);

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
    printf("mapped O=%d I=%d bytes\n", grp->Obytes, grp->Ibytes);

    /* Safe-OP manufacturer params (same as the other bench scripts) */
    for (int s = 1; s <= n; s++)
    {
        uint16_t sync_mode = 2, irq_thr = 10, sync_lost = 20;
        ecx_SDOwrite(&ctx, s, 0x2013, 6,  FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE);
        ecx_SDOwrite(&ctx, s, 0x2013, 18, FALSE, sizeof(irq_thr),   &irq_thr,   EC_TIMEOUTSAFE);
        ecx_SDOwrite(&ctx, s, 0x2013, 3,  FALSE, sizeof(sync_lost), &sync_lost, EC_TIMEOUTSAFE);
    }
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n");

    /* pre-charge cyclic frames, pin target to actual */
    for (int m = 0; m < n; m++) { rx[m]->control_word = 0; rx[m]->target_position = tx[m]->position_actual; }
    for (int k = 0; k < 50; k++) { exchange(); osal_usleep(1000); }

    ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    printf("OPERATIONAL reached\n");

    /* read the current inertia ratio BEFORE tuning */
    uint16_t pre[MAX_MOTORS] = {0};
    for (int m = 0; m < n; m++) rd16(m + 1, 0x2000, 7, &pre[m]);

    /* enable servos, then trigger F30.10 = 1 once each */
    int     triggered[MAX_MOTORS] = {0};
    int64   toff = 0;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    next.tv_nsec = ((next.tv_nsec / 1000000) + 1) * 1000000;

    printf("Enabling servos and triggering offline inertia auto-tune (MOVES THE SHAFTS)...\n");
    for (int i = 0; i < 8000 && running; i++)   /* up to 8 s @ 1 ms */
    {
        next.tv_nsec += CYCLE_NS + toff;
        if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
        if (wkc > 0) ec_sync(ctx.DCtime, CYCLE_NS, &toff);

        for (int m = 0; m < n; m++)
        {
            uint16_t sw = tx[m]->status_word, state = sw & 0x006F;
            if (sw & 0x0008) { rx[m]->control_word = 0x0080; continue; }  /* hold fault reset steady */
            if ((sw & 0x004F) == 0x0040)      rx[m]->control_word = 0x0006;
            else if (state == 0x0021)         rx[m]->control_word = 0x0007;
            else if (state == 0x0023)       { rx[m]->control_word = 0x000F; rx[m]->target_position = tx[m]->position_actual; }
            else if (state == 0x0027)       { rx[m]->control_word = 0x000F; rx[m]->target_position = tx[m]->position_actual;
                                              if (!triggered[m]) { wr16(m + 1, 0x2030, 11, 1); triggered[m] = 1;
                                                                   printf("  slave %d: F30.10 auto-tune triggered\n", m + 1); } }
        }
        ecx_send_processdata(&ctx);

        if (i % 1000 == 999)
        {
            int all_trig = 1;
            for (int m = 0; m < n; m++) if (!triggered[m]) all_trig = 0;
            if (all_trig) printf("  ... auto-tune in progress (t=%ds)\n", (i + 1) / 1000);
        }
    }

    /* read back the result */
    printf("\n== inertia auto-tune result ==\n");
    for (int m = 0; m < n; m++)
    {
        uint16_t post = 0, f30 = 0;
        rd16(m + 1, 0x2000, 7, &post);
        rd16(m + 1, 0x2030, 11, &f30);
        printf("  slave %d: C00.06 before=%u%%  after=%u%%  (F30.10=%u, 0=idle/done)\n",
               m + 1, pre[m], post, f30);
    }

    /* disable + walk state machine down */
    printf("\ndisabling ...\n");
    for (int m = 0; m < n; m++) rx[m]->control_word = 0x0007;
    exchange(); osal_usleep(10000);
    for (int m = 0; m < n; m++) rx[m]->control_word = 0x0000;
    exchange(); osal_usleep(10000);

    ctx.slavelist[0].state = EC_STATE_SAFE_OP; ecx_writestate(&ctx, 0); ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    ctx.slavelist[0].state = EC_STATE_PRE_OP;  ecx_writestate(&ctx, 0); ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    ctx.slavelist[0].state = EC_STATE_INIT;    ecx_writestate(&ctx, 0);
    ecx_close(&ctx);
    printf("done.\n");
    return 0;
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Usage:\n");
        printf("  sudo ./a6_inertia <ifname>                read params\n");
        printf("  sudo ./a6_inertia <ifname> set <ratio>    set load inertia ratio %% (0..12000)\n");
        printf("  sudo ./a6_inertia <ifname> stiff <level>  set stiffness level (1..31, def 12)\n");
        printf("  sudo ./a6_inertia <ifname> tune           offline inertia auto-tune (moves the motor)\n");
        return 1;
    }
    if (argc == 2)
        return do_read(argv[1]);
    if (argc == 4 && strcmp(argv[2], "set") == 0)
        return do_write(argv[1], 0x2000, 7, "load inertia ratio C00.06", (uint16_t)strtoul(argv[3], NULL, 0));
    if (argc == 4 && strcmp(argv[2], "stiff") == 0)
        return do_stiff(argv[1], (uint16_t)strtoul(argv[3], NULL, 0));
    if (argc == 3 && strcmp(argv[2], "tune") == 0)
        return do_tune(argv[1]);
    printf("bad arguments\n");
    return 1;
}
