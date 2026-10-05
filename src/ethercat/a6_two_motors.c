/* a6_two_motors.c — control TWO daisy-chained A6-EC servo drives over EtherCAT
 * (CSP mode), real-time variant (mlockall + SCHED_FIFO + core pinning).
 *
 * Usage: sudo ./a6_two_motors <ifname>      (e.g. eth0)
 *
 * Topology (daisy chain, auto-increment addressing):
 *   master (Pi) --eth--> drive A  CN3 (IN)
 *   drive A      CN4 (OUT) --eth--> drive B  CN3 (IN)
 * Drive A is closest to the master -> slavelist[1].
 * Drive B is second in the chain      -> slavelist[2].
 *
 * Brings BOTH drives to OPERATIONAL with DC sync, enables both, then streams a
 * small sine per motor (different frequency per axis so you can see they are
 * commanded independently). Prints each motor's actual/target/following-error.
 *
 * Uses the A6-EC predefined PDO mapping (0x1701 / 0x1B01) from
 * STEPPERONLINE_A6_Servo_V0.02.xml.
 *
 * NO LOAD ASSUMED — run with BOTH shafts free. Amplitude is only +/- 0.25 rev.
 *
 * Key ordering (same as a6_csp_rt.c): the SM-sync objects 0x1C32/0x1C33 are
 * WriteRestrictions="PreOP", so they are written in the Pre-OP -> Safe-OP
 * transition hook (a6_setup) BEFORE ec_config_map_group, and DC SYNC0 is armed
 * there too. Writing them in Safe-OP is rejected -> Er74.1 "No sync signal".
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

#define CYCLE_NS      1000000L  /* 1 ms cycle */
#define ENCODER_RES   131072L   /* 17-bit encoder: 131072 counts/rev */
#define RT_CPU        3         /* core the loop is pinned to; pair with isolcpus=3 */
#define MAX_MOTORS    2
#define CSP_AMP       32768.0   /* +/- 0.25 rev */

/* one frequency per axis (Hz) — deliberately different so independent
 * control is obvious on the bench. Keep them low for a first run. */
static const double CSP_FREQ[MAX_MOTORS] = { 0.25, 0.40 };

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
static A6_RxPDO *rx[MAX_MOTORS];
static A6_TxPDO *tx[MAX_MOTORS];
static volatile int running = 1;

static void sighandler(int sig)
{
    (void)sig;
    running = 0;
}

/* Pre-OP -> Safe-OP hook: runs once per slave (registered on each slave's
 * PO2SOconfig). Writes CSP mode + SM-sync (0x1C32/0x1C33) + arms DC SYNC0,
 * all in Pre-OP, for THIS slave. */
static int a6_setup(ecx_contextt *context, uint16_t slave)
{
    int8_t   mode      = 8;         /* 8 = CSP */
    uint16_t sync_mode = 2;         /* 2 = DC Sync0 */
    uint32_t cycle_ns  = CYCLE_NS;

    printf("[PO2SO] Pre-OP config for slave %d ...\n", slave);

    ecx_SDOwrite(context, slave, 0x6060, 0, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE);

    ecx_SDOwrite(context, slave, 0x1C32, 1, FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C32, 2, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C32, 10, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTSAFE);

    ecx_SDOwrite(context, slave, 0x1C33, 1, FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C33, 2, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C33, 10, FALSE, sizeof(cycle_ns), &cycle_ns, EC_TIMEOUTSAFE);

    /* Arm DC SYNC0 for THIS slave during the P->S transition. */
    ecx_dcsync0(context, slave, TRUE, CYCLE_NS, -97000000);
    printf("[PO2SO] slave %d DC SYNC0 armed (cycle=%lu ns)\n", slave, (unsigned long)CYCLE_NS);

    return 1;
}

/* one process-data round trip; returns working counter */
static int exchange(void)
{
    ecx_send_processdata(&ctx);
    return ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
}

/* DC sync: PI controller aligning the master loop to the reference slave's
 * SYNC0 (mirrors SOEM's ec_sync). With multiple slaves the master syncs to
 * the first DC slave (slavelist[1]) via ctx.DCtime. */
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

int main(int argc, char *argv[])
{
    ec_groupt *grp;
    uint8_t io_map[4096] = {0};
    int wkc = 0;
    int n;                      /* number of drives actually found */

    if (argc != 2)
    {
        printf("Usage: sudo ./a6_two_motors <ifname>\n");
        return 1;
    }
    signal(SIGINT, sighandler);

    /* ---- real-time setup (PREEMPT_RT) ---- */
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

    n = ctx.slavecount;
    if (n > MAX_MOTORS)
    {
        printf("found %d slaves, capping to %d\n", n, MAX_MOTORS);
        n = MAX_MOTORS;
    }
    if (n < 2)
        printf("WARNING: only %d slave(s) found — is the second drive powered and daisy-chained?\n", n);

    for (int s = 1; s <= n; s++)
        printf("  slave %d: name='%s' configadr=0x%04X\n", s, ctx.slavelist[s].name, ctx.slavelist[s].configadr);

    /* ---- DC configuration FIRST (before P->S transition) ---- */
    ecx_configdc(&ctx);
    osal_usleep(500000);        /* let DC clocks settle */

    /* register the Pre-OP hook on EVERY slave, then map IO (P->S). */
    for (int s = 1; s <= n; s++)
        ctx.slavelist[s].PO2SOconfig = a6_setup;
    ecx_config_map_group(&ctx, io_map, 0);

    grp = ctx.grouplist;
    /* process data is concatenated in chain order: rx[0]=drive A, rx[1]=drive B */
    for (int m = 0; m < n; m++)
    {
        rx[m] = (A6_RxPDO *)(grp->outputs + m * sizeof(A6_RxPDO));
        tx[m] = (A6_TxPDO *)(grp->inputs  + m * sizeof(A6_TxPDO));
    }
    printf("mapped O=%d I=%d bytes (expect %d/%d)\n",
           grp->Obytes, grp->Ibytes,
           (int)(n * sizeof(A6_RxPDO)), (int)(n * sizeof(A6_TxPDO)));

    /* ---- Safe-OP: per-slave manufacturer params ---- */
    for (int s = 1; s <= n; s++)
    {
        uint16_t sync_mode = 2;         /* 0x2013:06 -> "Sync 2" (host jitter > 1us) */
        uint16_t irq_thr   = 10;        /* 0x2013:18 "Irq lost window" 5 -> 10      */
        uint16_t sync_lost = 20;        /* 0x2013:03 "Sync lost window" 8 -> 20     */
        ecx_SDOwrite(&ctx, s, 0x2013, 6,  FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE);
        ecx_SDOwrite(&ctx, s, 0x2013, 18, FALSE, sizeof(irq_thr),   &irq_thr,   EC_TIMEOUTSAFE);
        ecx_SDOwrite(&ctx, s, 0x2013, 3,  FALSE, sizeof(sync_lost), &sync_lost, EC_TIMEOUTSAFE);
        printf("slave %d: C13.05=2, Irq-lost=10, Sync-lost=20 written\n", s);
    }

    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n");

    /* ---- pre-charge cyclic frames before OP; zero each target at current pos ---- */
    for (int m = 0; m < n; m++)
    {
        rx[m]->control_word    = 0x0000;
        rx[m]->target_position = tx[m]->position_actual;
    }
    for (int k = 0; k < 50; k++)
    {
        exchange();
        osal_usleep(1000);
    }

    /* ---- OPERATIONAL ---- */
    ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    printf("OPERATIONAL reached\n");
    exchange();
    for (int m = 0; m < n; m++)
        printf("  M%d post-OP: status=0x%04X fault=%d error=0x%04X\n",
               m, tx[m]->status_word, (tx[m]->status_word >> 3) & 1, tx[m]->error_code);

    /* ---- DC SYNC0 ack per slave (bit 0 of 0x0980 AssignActivate) ---- */
    for (int s = 1; s <= n; s++)
    {
        uint16_t aa = 0;
        ecx_FPRD(&ctx.port, ctx.slavelist[s].configadr, 0x0980, sizeof(aa), &aa, EC_TIMEOUTRET);
        printf("  slave %d AssignActivate(0x980)=0x%04X (SYNC0 ack = bit0)\n", s, etohs(aa));
    }

    /* ---- DC-synced loop: per-motor CiA-402 enable, then sine ---- */
    {
        int64   toff = 0;
        int32_t start_pos[MAX_MOTORS] = {0};
        int     motion_step[MAX_MOTORS]  = {0};
        int     state_watchdog[MAX_MOTORS] = {0};
        int     fault_reset[MAX_MOTORS]    = {0};
        int     fault_retries[MAX_MOTORS]  = {0};
        struct timespec next;
        clock_gettime(CLOCK_MONOTONIC, &next);
        next.tv_nsec = ((next.tv_nsec / 1000000) + 1) * 1000000;

        printf("Beginning DC cyclic CSP loop (%d motors, small sine, free shafts)...\n", n);
        for (int i = 0; i < 20000 && running; i++)
        {
            next.tv_nsec += CYCLE_NS + toff;
            if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

            wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
            if (wkc > 0)
                ec_sync(ctx.DCtime, CYCLE_NS, &toff);

            for (int m = 0; m < n; m++)
            {
                uint16_t sw    = tx[m]->status_word;
                uint16_t state = sw & 0x006F;

                /* fault -> auto-reset (bit 7 rising edge), skip state machine this cycle */
                if (sw & 0x0008)
                {
                    fault_reset[m] = !fault_reset[m];
                    rx[m]->control_word = fault_reset[m] ? 0x0080 : 0x0000;
                    if (++fault_retries[m] % 400 == 0)
                        printf("  [M%d FAULT] status=0x%04X error=0x%04X, resetting...\n",
                               m, sw, tx[m]->error_code);
                    continue;
                }

                if ((sw & 0x004F) == 0x0040)          /* switch on disabled -> shutdown */
                {
                    rx[m]->control_word = 0x0006;
                }
                else if (state == 0x0021)             /* ready -> switch on */
                {
                    rx[m]->control_word = 0x0007;
                }
                else if (state == 0x0023)             /* switched on -> enable */
                {
                    rx[m]->control_word = 0x000F;
                    start_pos[m] = tx[m]->position_actual;
                    rx[m]->target_position = start_pos[m];
                }
                else if (state == 0x0027)             /* operation enabled -> stream target */
                {
                    motion_step[m]++;
                    double t    = motion_step[m] * 1e-3;
                    double ramp = (motion_step[m] < 1000) ? motion_step[m] / 1000.0 : 1.0;
                    rx[m]->target_position = start_pos[m]
                        + (int32_t)(CSP_AMP * ramp * sin(2.0 * M_PI * CSP_FREQ[m] * t));
                    rx[m]->control_word = 0x000F;
                }
                else
                {
                    if (++state_watchdog[m] > 3000)
                        printf("  [M%d] timeout waiting for CiA-402 state. status=0x%04X\n", m, sw);
                }
            }

            ecx_send_processdata(&ctx);

            if (i % 200 == 0)
                for (int m = 0; m < n; m++)
                    printf("  M%d: actual=%8d target=%8d ferr=%8d status=0x%04X\n",
                           m, tx[m]->position_actual, rx[m]->target_position,
                           tx[m]->following_error, tx[m]->status_word);
        }
    }

    /* ---- shutdown: disable both drives, walk state machine down so DC sync
     * drops at the Safe-OP -> Pre-OP transition (not an abrupt INIT jump,
     * which the drive latches as ErC1.1). ---- */
    printf("disabling ...\n");
    for (int m = 0; m < n; m++)
        rx[m]->control_word = 0x0007;
    exchange();
    osal_usleep(10000);
    for (int m = 0; m < n; m++)
        rx[m]->control_word = 0x0000;
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
