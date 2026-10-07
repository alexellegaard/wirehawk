/* a6_reset.c — reset a fault on ALL daisy-chained A6-EC drives.
 *
 * Usage: sudo ./a6_reset <ifname>     (e.g. enx00e06c399ebd)
 *
 * Brings every drive to OPERATIONAL with DC sync active, then sends the CiA 402
 * fault-reset (control word 0x80) to each slave and prints its status word
 * before/after. Leaves the drives disabled (walked back down to INIT). Resets
 * ALL slaves found on the bus (up to MAX_MOTORS).
 *
 * Uses the A6-EC predefined PDO mapping (0x1701 / 0x1B01) from
 * STEPPERONLINE_A6_Servo_V0.02.xml. Same DC ordering as a6_step_test.c: the
 * SM-sync objects (0x1C32/0x1C33) are PreOP-only, so they are written in the
 * Pre-OP -> Safe-OP transition hook and DC SYNC0 is armed there too.
 */
#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define CYCLE_NS   1000000L
#define MAX_MOTORS 4

/* ---- A6-EC predefined PDO layout (0x1701 / 0x1B01) ---- */
typedef struct __attribute__((packed))
{
    uint16_t control_word;     /* 0x6040 */
    int32_t  target_position;  /* 0x607A */
    uint16_t touch_probe;      /* 0x60B8 */
    uint32_t physical_outputs; /* 0x60FE */
} A6_RxPDO;                    /* 12 bytes */

typedef struct __attribute__((packed))
{
    uint16_t error_code;       /* 0x603F */
    uint16_t status_word;      /* 0x6041 */
    int32_t  position_actual;  /* 0x6064 */
    int16_t  torque_actual;    /* 0x6077 */
    int32_t  following_error;  /* 0x60F4 */
    uint16_t touch_status;     /* 0x60B9 */
    int32_t  touch_pos1;       /* 0x60BA */
    int32_t  touch_pos2;       /* 0x60BC */
    uint32_t digital_inputs;   /* 0x60FD */
} A6_TxPDO;                    /* 28 bytes */

static ecx_contextt ctx;
static A6_RxPDO *rx[MAX_MOTORS];
static A6_TxPDO *tx[MAX_MOTORS];

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

int main(int argc, char *argv[])
{
    ec_groupt *grp;
    uint8_t io_map[4096] = {0};
    int n;

    if (argc != 2)
    {
        printf("Usage: sudo ./a6_reset <ifname>\n");
        return 1;
    }

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
    n = (ctx.slavecount > MAX_MOTORS) ? MAX_MOTORS : ctx.slavecount;

    ecx_configdc(&ctx);
    osal_usleep(500000);                          /* let DC clocks settle */
    for (int s = 1; s <= n; s++)
        ctx.slavelist[s].PO2SOconfig = a6_setup;
    ecx_config_map_group(&ctx, io_map, 0);

    grp = ctx.grouplist;
    for (int m = 0; m < n; m++)
    {
        rx[m] = (A6_RxPDO *)(grp->outputs + m * sizeof(A6_RxPDO));
        tx[m] = (A6_TxPDO *)(grp->inputs  + m * sizeof(A6_TxPDO));
    }
    printf("mapped O=%d I=%d bytes\n", grp->Obytes, grp->Ibytes);

    /* Safe-OP manufacturer sync params (same as a6_step_test) */
    for (int s = 1; s <= n; s++)
    {
        uint16_t v = 2;  ecx_SDOwrite(&ctx, s, 0x2013, 6,  FALSE, sizeof(v), &v, EC_TIMEOUTSAFE);
        v = 10;          ecx_SDOwrite(&ctx, s, 0x2013, 18, FALSE, sizeof(v), &v, EC_TIMEOUTSAFE);
        v = 20;          ecx_SDOwrite(&ctx, s, 0x2013, 3,  FALSE, sizeof(v), &v, EC_TIMEOUTSAFE);
    }
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n");

    /* pre-charge cyclic frames, pin target to actual */
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

    for (int m = 0; m < n; m++)
        printf("M%d before reset: status=0x%04X fault=%d error=0x%04X\n",
               m, tx[m]->status_word, (tx[m]->status_word >> 3) & 1, tx[m]->error_code);

    /* fault reset (control word bit 7, edge-triggered), then clear the bit */
    for (int m = 0; m < n; m++) rx[m]->control_word = 0x0080;
    exchange();
    osal_usleep(50000);
    for (int m = 0; m < n; m++) rx[m]->control_word = 0x0000;
    exchange();
    osal_usleep(50000);
    exchange();

    for (int m = 0; m < n; m++)
        printf("M%d after reset:  status=0x%04X fault=%d error=0x%04X\n",
               m, tx[m]->status_word, (tx[m]->status_word >> 3) & 1, tx[m]->error_code);

    for (int m = 0; m < n; m++)
        if ((tx[m]->status_word >> 3) & 1)
            printf("M%d FAULT STILL PRESENT after reset (status=0x%04X)\n", m, tx[m]->status_word);

    /* shutdown: walk the state machine down so DC sync drops at S->P, not INIT */
    for (int m = 0; m < n; m++) rx[m]->control_word = 0x0000;
    exchange();
    osal_usleep(10000);
    ctx.slavelist[0].state = EC_STATE_SAFE_OP; ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    ctx.slavelist[0].state = EC_STATE_PRE_OP; ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    ctx.slavelist[0].state = EC_STATE_INIT; ecx_writestate(&ctx, 0);
    ecx_close(&ctx);
    printf("done.\n");
    return 0;
}
