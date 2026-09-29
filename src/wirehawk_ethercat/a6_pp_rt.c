/* a6_pp_rt.c — SOEM EtherCAT master for the STEPPERONLINE A6-EC servo (PP mode),
 * real-time variant: mlockall + SCHED_FIFO + core pinning for a PREEMPT_RT kernel.
 *
 * Usage: sudo ./a6_pp_rt <ifname>     (e.g. enx00e06c399ebd)
 *
 * Brings the drive to OPERATIONAL, sets CiA 402 PP mode (0x6060 = 1),
 * runs the enable sequence, then jogs the target position one revolution
 * (131072 counts) back and forth.
 *
 * Uses the A6-EC predefined PDO mapping (0x1701 / 0x1B01) from
 * STEPPERONLINE_A6_Servo_V0.02.xml. NO load assumed — test with the shaft free.
 *
 * Key ordering: the SM-sync objects 0x1C32/0x1C33 are declared
 * WriteRestrictions="PreOP" in the ESI, so they are written in PRE-OP *before*
 * ec_config_map_group (which transitions to Safe-OP). Writing them in Safe-OP
 * is rejected and leaves the drive in Free Run -> Er74.1 "No sync signal".
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

#define CYCLE_NS     1000000L  /* 1 ms cycle (fine for PP mode) */
#define ENCODER_RES  131072L   /* 17-bit encoder: 131072 counts/rev */
#define RT_CPU       3          /* core the loop is pinned to; pair with isolcpus=3 in cmdline.txt */

/* ---- A6-EC predefined PDO layout (0x1701 / 0x1B01) ---- */
typedef struct __attribute__((packed))
{
    uint16_t control_word;     /* 0x6040 Controlword       */
    int32_t  target_position;  /* 0x607A Target position   */
    uint16_t touch_probe;      /* 0x60B8 Touch probe func. */
    uint32_t physical_outputs; /* 0x60FE Physical outputs  */
} A6_RxPDO;                    /* 12 bytes */

typedef struct __attribute__((packed))
{
    uint16_t error_code;       /* 0x603F Error code        */
    uint16_t status_word;      /* 0x6041 Statusword        */
    int32_t  position_actual;  /* 0x6064 Position actual   */
    int16_t  torque_actual;    /* 0x6077 Torque actual     */
    int32_t  following_error;  /* 0x60F4 Following error   */
    uint16_t touch_status;     /* 0x60B9 Touch probe status*/
    int32_t  touch_pos1;       /* 0x60BA Touch probe pos1  */
    int32_t  touch_pos2;       /* 0x60BC Touch probe pos2  */
    uint32_t digital_inputs;   /* 0x60FD Digital inputs    */
} A6_TxPDO;                    /* 28 bytes */

static ecx_contextt ctx;
static A6_RxPDO *rx;
static A6_TxPDO *tx;
static volatile int running = 1;

static void sighandler(int sig)
{
    (void)sig;
    running = 0;
}

/* Pre-OP -> Safe-OP hook: write PreOP-only SDOs (PP mode, SM sync) + arm DC SYNC0.
 * Runs inside ec_config_map_group at the PS transition, where the slave is
 * in Pre-OP and accepts these writes (writing before the mapping fails —
 * the slave isn't in Pre-OP yet right after ec_config_init).
 * C13.05 is NOT written here — it's a manufacturer param set later in Safe-OP. */
static int a6_setup(ecx_contextt *context, uint16_t slave)
{
    int8_t   mode      = 1;         /* 1 = PP (Profile Position) */
    uint16_t sync_mode = 2;         /* 2 = DC Sync0 */
    uint32_t cycle_ns  = CYCLE_NS;

    printf("[PO2SO] Pre-OP config for slave %d...\n", slave);

    if (ecx_SDOwrite(context, slave, 0x6060, 0, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE) > 0)
        printf("[PO2SO] PP mode (0x6060=1) OK\n");
    else
        printf("[PO2SO] FAILED PP mode (0x6060)\n");

    /* PP profile params (encoder counts/s and counts/s^2). 131072 cts/s = 60 rpm. */
    {
        uint32_t pvel = 131072, pacc = 1310720, pdec = 1310720;
        ecx_SDOwrite(context, slave, 0x6081, 0, FALSE, sizeof(pvel), &pvel, EC_TIMEOUTSAFE);
        ecx_SDOwrite(context, slave, 0x6083, 0, FALSE, sizeof(pacc), &pacc, EC_TIMEOUTSAFE);
        ecx_SDOwrite(context, slave, 0x6084, 0, FALSE, sizeof(pdec), &pdec, EC_TIMEOUTSAFE);
        printf("[PO2SO] PP profile velocity=131072 cts/s (60 rpm), acc/dec=1310720\n");
    }

    if (ecx_SDOwrite(context, slave, 0x1C32, 1, FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE) > 0)
        printf("[PO2SO] 0x1C32:01 = 2 (DC Sync0) OK\n");
    else
        printf("[PO2SO] FAILED 0x1C32:01\n");
    if (ecx_SDOwrite(context, slave, 0x1C32, 2, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTSAFE) > 0)
        printf("[PO2SO] 0x1C32:02 cycle=%u OK\n", cycle_ns);
    else
        printf("[PO2SO] FAILED 0x1C32:02 cycle\n");
    if (ecx_SDOwrite(context, slave, 0x1C32, 10, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTSAFE) > 0)
        printf("[PO2SO] 0x1C32:0A sync0-cycle=%u OK\n", cycle_ns);
    else
        printf("[PO2SO] FAILED 0x1C32:0A\n");

    if (ecx_SDOwrite(context, slave, 0x1C33, 1, FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE) > 0)
        printf("[PO2SO] 0x1C33:01 = 2 (DC Sync0) OK\n");
    else
        printf("[PO2SO] FAILED 0x1C33:01\n");
    if (ecx_SDOwrite(context, slave, 0x1C33, 2, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTSAFE) > 0)
        printf("[PO2SO] 0x1C33:02 cycle=%u OK\n", cycle_ns);
    else
        printf("[PO2SO] FAILED 0x1C33:02 cycle\n");
    if (ecx_SDOwrite(context, slave, 0x1C33, 10, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTSAFE) > 0)
        printf("[PO2SO] 0x1C33:0A sync0-cycle=%u OK\n", cycle_ns);
    else
        printf("[PO2SO] FAILED 0x1C33:0A\n");

    /* Arm DC SYNC0 HERE, during the Pre-OP -> Safe-OP transition. ETG flow
     * activates sync in P->S; the A6-EC rejects a DC config armed only after
     * Safe-OP with AL status 0x0030 "Invalid DC SYNCH Configuration". */
    ecx_dcsync0(context, slave, TRUE, CYCLE_NS, -97000000);
    printf("[PO2SO] DC SYNC0 armed (cycle=%lu ns) in P->S transition\n", (unsigned long)CYCLE_NS);

    return 1;
}

/* one process-data round trip; returns working counter */
static int exchange(void)
{
    ecx_send_processdata(&ctx);
    return ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
}

/* DC sync: PI controller aligning the master loop to the slave's SYNC0
 * (mirrors SOEM's samples/ec_sample/ec_sample.c ec_sync) */
static float pgain = 0.00002f;
static float igain = 0.00002f;
static int64 syncoffset = 500000; /* sync point 500 us after DC sync */

static void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta = (reftime - syncoffset) % cycletime;
    if (delta > (cycletime / 2))
        delta -= cycletime;
    integral += -delta;
    *offsettime = (int64)((-delta * pgain) + (integral * igain));
}

int main(int argc, char *argv[])
{
    ec_groupt *grp;
    int32_t start_pos = 0;
    uint8_t io_map[4096] = {0};
    int wkc = 0;

    if (argc != 2)
    {
        printf("Usage: sudo ./a6_pp_rt <ifname>\n");
        return 1;
    }
    signal(SIGINT, sighandler);

    /* ---- real-time setup (PREEMPT_RT): lock memory, pin to an isolated core,
     * promote this thread to SCHED_FIFO. Needs root (sudo) or CAP_SYS_NICE +
     * CAP_IPC_LOCK. Pair with `isolcpus=3` in /boot/firmware/cmdline.txt. ---- */
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

    /* ---- startup ---- */
    printf("Init on %s ... ", argv[1]);
    if (!ecx_init(&ctx, argv[1]))
    {
        printf("no socket\n");
        return 1;
    }

    if (ecx_config_init(&ctx) <= 0)
    {
        printf("no slaves found\n");
        ecx_close(&ctx);
        return 1;
    }
    printf("%d slave(s)\n", ctx.slavecount);

    /* ---- DC configuration FIRST: synchronize the clock BEFORE the P->S transition.
     * The PO2SO hook arms DC SYNC0 by computing START0 from DCSYSTIME; if the clock
     * isn't synced yet (a cold drive's local time is ~5 s after power-on), START0
     * lands in the distant past and SYNC0 never fires on the first run. ---- */
    printf("DC: group hasdc=%d slave hasdc=%d configadr=0x%04X\n",
           ctx.slavelist[0].hasdc, ctx.slavelist[1].hasdc, ctx.slavelist[1].configadr);
    ecx_configdc(&ctx);
    osal_usleep(500000);                          /* let DC clocks settle */

    /* ---- register Pre-OP config hook, then map IO (transitions Pre-OP -> Safe-OP).
     * The hook writes PP mode + SM-sync (0x1C32/0x1C33) + arms DC SYNC0, all in Pre-OP. ---- */
    ctx.slavelist[1].PO2SOconfig = a6_setup;
    ecx_config_map_group(&ctx, io_map, 0);
    grp = ctx.grouplist;
    rx = (A6_RxPDO *)grp->outputs;
    tx = (A6_TxPDO *)grp->inputs;
    printf("mapped O=%d I=%d bytes\n", grp->Obytes, grp->Ibytes);
    if (grp->Obytes != (int)sizeof(A6_RxPDO) || grp->Ibytes != (int)sizeof(A6_TxPDO))
        printf("WARNING: PDO sizes don't match structs (O %d vs %zu, I %d vs %zu)\n",
               grp->Obytes, sizeof(A6_RxPDO), grp->Ibytes, sizeof(A6_TxPDO));
    {
        uint8  ra  = 0;
        uint32 cyc = 0, off = 0, dly = 0;
        uint64 st = 0, start0 = 0;
        uint16 sh  = ctx.slavelist[1].configadr;
        ecx_FPRD(&ctx.port, sh, ECT_REG_DCSYNCACT,  sizeof(ra),  &ra,  EC_TIMEOUTRET);
        ecx_FPRD(&ctx.port, sh, ECT_REG_DCCYCLE0,   sizeof(cyc), &cyc, EC_TIMEOUTRET);
        ecx_FPRD(&ctx.port, sh, ECT_REG_DCSYSOFFSET, sizeof(off), &off, EC_TIMEOUTRET);
        ecx_FPRD(&ctx.port, sh, ECT_REG_DCSYSDELAY,  sizeof(dly), &dly, EC_TIMEOUTRET);
        ecx_FPRD(&ctx.port, sh, ECT_REG_DCSYSTIME,   sizeof(st),  &st,  EC_TIMEOUTRET);
        ecx_FPRD(&ctx.port, sh, ECT_REG_DCSTART0,    sizeof(start0), &start0, EC_TIMEOUTRET);
        cyc = etohl(cyc); off = etohl(off); dly = etohl(dly);
        st = etohll(st); start0 = etohll(start0);
        printf("DC: DCSYNCACT=0x%02X (expect 0x03)  DCCYCLE0=%u  OFFSET=%u  DELAY=%u\n",
               ra, cyc, off, dly);
        printf("DC: SYSTIME=%llu START0=%llu  (START0-SYSTIME=%lld ns, expect >0)\n",
               (unsigned long long)st, (unsigned long long)start0,
               (long long)(start0 - st));
    }

    /* ---- Safe-OP: C13.05 sync mode (manufacturer param).
     * 2 = "Sync 2" (host jitter > 1us) — the correct mode for a non-RT PC master
     * that jitters ~50-100us. "Sync 1" (default) assumes ~1us host jitter and the
     * drive latches ErC1.1 "Synchronization loss" under a jittery master. */
    {
        uint16_t sync_mode = 2;   /* C13.05 = 0x2013:06 -> Sync 2 */
        if (ecx_SDOwrite(&ctx, 1, 0x2013, 6, FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE) > 0)
            printf("C13.05 = 2 (Sync 2)\n");
        else
            printf("FAILED to set C13.05 (0x2013:06)\n");

        /* C13.11 (0x2013:12) = IRQ-loss threshold: raise 5 -> 10 so the non-RT
         * master's frame jitter doesn't trip ErC1.1 "Synchronization loss". */
        uint16_t irq_thr = 10;
        if (ecx_SDOwrite(&ctx, 1, 0x2013, 12, FALSE, sizeof(irq_thr), &irq_thr, EC_TIMEOUTSAFE) > 0)
            printf("C13.11 = 10 (IRQ-loss threshold)\n");
        else
            printf("FAILED to set C13.11 (0x2013:12)\n");

        uint16_t v = 0; int sz = sizeof(v);
        if (ecx_SDOread(&ctx, 1, 0x2013, 6, FALSE, &sz, &v, EC_TIMEOUTSAFE) > 0)
            printf("C13.05 readback = %u (expect 2)\n", v);
        sz = sizeof(v);
        if (ecx_SDOread(&ctx, 1, 0x2013, 5, FALSE, &sz, &v, EC_TIMEOUTSAFE) > 0)
            printf("C13.04 sync-loss count = %u\n", v);
    }

    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n");

    /* ---- pre-charge cyclic frames before OP ---- */
    ecx_send_processdata(&ctx);
    ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    start_pos = tx->position_actual;
    rx->target_position = start_pos;
    rx->control_word = 0x0000;
    for (int k = 0; k < 50; k++)
    {
        ecx_send_processdata(&ctx);
        ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
        osal_usleep(1000);
    }

    /* ---- OPERATIONAL ---- */
    ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    printf("OPERATIONAL reached\n");
    exchange();
    printf("post-OP status=0x%04X fault=%d error=0x%04X\n",
           tx->status_word, (tx->status_word >> 3) & 1, tx->error_code);

    /* ESC DC sync state (diagnostic) */
    {
        uint32_t tf = 0, sd = 0;
        uint16_t aa = 0;
        ecx_FPRD(&ctx.port, ctx.slavelist[1].configadr, 0x0934, sizeof(tf), &tf, EC_TIMEOUTRET);
        ecx_FPRD(&ctx.port, ctx.slavelist[1].configadr, 0x092C, sizeof(sd), &sd, EC_TIMEOUTRET);
        ecx_FPRD(&ctx.port, ctx.slavelist[1].configadr, 0x0980, sizeof(aa), &aa, EC_TIMEOUTRET);
        printf("DC timefilt(0x934)=%d ns  sysdiff(0x92C)=%d ns  AssignActivate(0x980)=0x%04X (SYNC0 ack=bit0)\n",
               (int32_t)etohl(tf), (int32_t)etohl(sd), etohs(aa));
    }

    /* diagnostic: EtherCAT AL-status code (0x0134) + drive IRQ-loss counter */
    {
        uint16_t alsc = 0;
        ecx_FPRD(&ctx.port, ctx.slavelist[1].configadr, 0x0134, sizeof(alsc), &alsc, EC_TIMEOUTRET);
        printf("AL status code (0x134) = 0x%04X\n", etohs(alsc));

        uint16_t irq = 0; int sz = sizeof(irq);
        if (ecx_SDOread(&ctx, 1, 0x2013, 13, FALSE, &sz, &irq, EC_TIMEOUTSAFE) > 0)
            printf("C13.12 IRQ-loss count = %u\n", irq);
    }

    /* read back SM-sync: what did the drive actually accept? */
    {
        uint16_t smtype = 0, smmodes = 0;
        uint32_t smcyc = 0, smmin = 0, sms0 = 0;
        int ssz;
        ssz = sizeof(smtype); ecx_SDOread(&ctx, 1, 0x1C32, 1,  FALSE, &ssz, &smtype, EC_TIMEOUTSAFE);
        ssz = sizeof(smcyc);  ecx_SDOread(&ctx, 1, 0x1C32, 2,  FALSE, &ssz, &smcyc,  EC_TIMEOUTSAFE);
        ssz = sizeof(smmodes);ecx_SDOread(&ctx, 1, 0x1C32, 4,  FALSE, &ssz, &smmodes,EC_TIMEOUTSAFE);
        ssz = sizeof(smmin);  ecx_SDOread(&ctx, 1, 0x1C32, 5,  FALSE, &ssz, &smmin,  EC_TIMEOUTSAFE);
        ssz = sizeof(sms0);   ecx_SDOread(&ctx, 1, 0x1C32, 10, FALSE, &ssz, &sms0,   EC_TIMEOUTSAFE);
        printf("SM-sync 0x1C32: type=%u (want 2) cycle=%u ns  supported-modes=0x%04X  min-cycle=%u ns  sync0-cycle=%u ns\n",
               smtype, smcyc, smmodes, smmin, sms0);
    }

    /* ---- DC-synced loop: deterministic CiA-402 enable, then jog ---- */
    {
        int64 toff = 0;
        struct timespec next;
        clock_gettime(CLOCK_MONOTONIC, &next);
        next.tv_nsec = ((next.tv_nsec / 1000000) + 1) * 1000000;

        int motion_step = 0;
        int state_watchdog = 0;
        int fault_reset = 0;
        int fault_retries = 0;

        printf("Beginning DC cyclic PP control loop...\n");
        for (int i = 0; i < 12000 && running; i++)
        {
            next.tv_nsec += CYCLE_NS + toff;
            if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

            wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
            if (wkc > 0)
                ec_sync(ctx.DCtime, CYCLE_NS, &toff);

            uint16_t sw = tx->status_word;
            uint16_t state = sw & 0x006F;

            /* fault? -> auto-reset (bit 7 rising edge), then the state machine re-arms */
            if (sw & 0x0008)
            {
                fault_reset = !fault_reset;
                rx->control_word = fault_reset ? 0x0080 : 0x0000;
                if (++fault_retries % 400 == 0)
                    printf("  [FAULT] status=0x%04X error=0x%04X, resetting...\n", sw, tx->error_code);
                ecx_send_processdata(&ctx);
                continue;
            }

            /* deterministic CiA-402 state machine */
            if ((sw & 0x004F) == 0x0040)            /* switch on disabled -> shutdown */
            {
                rx->control_word = 0x0006;
            }
            else if (state == 0x0021)               /* ready to switch on -> switch on */
            {
                rx->control_word = 0x0007;
            }
            else if (state == 0x0023)               /* switched on -> enable operation */
            {
                rx->control_word = 0x000F;
                start_pos = tx->position_actual;
                rx->target_position = start_pos;
            }
            else if (state == 0x0027)               /* operation enabled -> PP move */
            {
                motion_step++;
                /* toggle target every 3 s: +1 rev, then back to start (repeat) */
                int phase = (motion_step / 3000) % 2;
                if (motion_step == 1 || motion_step % 3000 == 0)
                {
                    rx->target_position = start_pos + (phase == 0 ? ENCODER_RES : 0);
                    rx->control_word = 0x003F;   /* enable + new setpoint + change immediately */
                    printf("  PP move: target=%d (%s)\n", rx->target_position,
                           phase == 0 ? "+1 rev" : "back to start");
                }
                else
                {
                    rx->control_word = 0x000F;   /* clear new setpoint */
                }
                if (motion_step % 200 == 0)
                    printf("  t=%4dms  actual=%8d  target=%8d  status=0x%04X\n",
                           motion_step, tx->position_actual, rx->target_position, sw);
                if (motion_step >= 6000)
                    break;
            }
            else
            {
                if (++state_watchdog > 3000)
                {
                    printf("Timeout waiting for CiA-402 state. status=0x%04X\n", sw);
                    break;
                }
            }

            ecx_send_processdata(&ctx);
        }
    }

    /* ---- shutdown: disable drive, then walk the state machine down OP -> Safe-OP
     * -> Pre-OP -> INIT so the DC sync is dropped at the Safe-OP -> Pre-OP
     * transition (a normal state change), not by jumping straight to INIT,
     * which the drive latches as ErC1.1 "Synchronization loss". ---- */
    printf("disabling ...\n");
    rx->control_word = 0x0007;
    exchange();
    osal_usleep(10000);
    rx->control_word = 0x0000;
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
