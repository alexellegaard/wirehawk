/* a6_reset.c — minimal SOEM program to reset a fault on the A6-EC drive.
 *
 * Usage: sudo ./a6_reset <ifname>     (e.g. enx00e06c399ebd)
 *
 * Brings the drive to OPERATIONAL with DC sync active, then sends the CiA 402
 * fault-reset (control word 0x80) to clear a resettable fault, and prints the
 * status word before/after. Leaves the drive disabled (back in INIT).
 *
 * Uses the A6-EC predefined PDO mapping (0x1701 / 0x1B01) from
 * STEPPERONLINE_A6_Servo_V0.02.xml.
 */
#define _POSIX_C_SOURCE 200809L

#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

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
static A6_RxPDO *rx;
static A6_TxPDO *tx;

static int exchange(void)
{
    ecx_send_processdata(&ctx);
    return ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
}

int main(int argc, char *argv[])
{
    ec_groupt *grp;
    uint8_t io_map[4096] = {0};

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

    ecx_config_map_group(&ctx, io_map, 0);
    grp = ctx.grouplist;
    rx = (A6_RxPDO *)grp->outputs;
    tx = (A6_TxPDO *)grp->inputs;
    if (grp->Obytes != (int)sizeof(A6_RxPDO) || grp->Ibytes != (int)sizeof(A6_TxPDO))
    {
        printf("WARNING: PDO sizes don't match structs (O %d vs %zu, I %d vs %zu)\n",
               grp->Obytes, sizeof(A6_RxPDO), grp->Ibytes, sizeof(A6_TxPDO));
    }

    ecx_configdc(&ctx);
    osal_usleep(500000);                          /* let DC clocks settle */
    ecx_dcsync0(&ctx, 1, TRUE, 1000000, -97000000);  /* 1 ms SYNC0; shift so first pulse ~3-4ms out, not ~100ms */

    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n");

    ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    printf("OPERATIONAL reached\n");

    exchange();
    printf("before reset: status=0x%04X fault=%d error=0x%04X\n",
           tx->status_word, (tx->status_word >> 3) & 1, tx->error_code);

    rx->control_word = 0x80;   /* fault reset (control word bit 7) */
    exchange();
    osal_usleep(50000);

    rx->control_word = 0x00;   /* clear the fault reset bit */
    exchange();
    osal_usleep(50000);

    exchange();
    printf("after reset:  status=0x%04X fault=%d error=0x%04X\n",
           tx->status_word, (tx->status_word >> 3) & 1, tx->error_code);

    if ((tx->status_word >> 3) & 1)
        printf("FAULT STILL PRESENT after reset (status=0x%04X)\n", tx->status_word);
    else
        printf("fault cleared. drive is now disabled — ready to run a6_csp.\n");

    ctx.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&ctx, 0);
    ecx_close(&ctx);
    printf("done.\n");
    return 0;
}
