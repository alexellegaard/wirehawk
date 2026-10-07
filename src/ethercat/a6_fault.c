/* a6_fault.c — read the A6 fault code (0x203F) + CiA error (0x603F) + status (0x6041)
 * for both slaves. Read-only, Safe-OP (servo stays off). Use after a trip to find
 * the EXACT fault instead of the 3-char 7-segment code.
 *
 * Usage: sudo ./a6_fault <ifname>
 *
 *   0x203F  drive/vendor fault code  (the authoritative one, e.g. 0x030 = Er03.0)
 *   0x603F  CiA 402 error code       (0x6320 = parameter error, 0x7500 = loop timeout, ...)
 *   0x6041  status word              (bit 3 = fault)
 */
#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

static ecx_contextt ctx;

static int rd16(uint16_t slave, uint16_t idx, uint8_t sub, uint16_t *v)
{
    int sz = sizeof(*v); *v = 0;
    return ecx_SDOread(&ctx, slave, idx, sub, FALSE, &sz, v, EC_TIMEOUTSAFE) > 0;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Usage: sudo ./a6_fault <ifname>\n");
        return 1;
    }

    printf("Init on %s ...\n", argv[1]);
    if (!ecx_init(&ctx, argv[1])) { printf("no socket\n"); return 1; }
    if (ecx_config_init(&ctx) <= 0) { printf("no slaves\n"); ecx_close(&ctx); return 1; }
    printf("%d slave(s)\n", ctx.slavecount);

    uint8_t io_map[4096] = {0};
    ecx_config_map_group(&ctx, io_map, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n\n");

    printf("== fault / status ==\n");
    printf("  slave | 0x203F (drive fault) | 0x603F (CiA error) | 0x6041 (status)\n");
    for (int s = 1; s <= ctx.slavecount; s++)
    {
        uint16_t f = 0, e = 0, st = 0;
        rd16(s, 0x203F, 0, &f);
        rd16(s, 0x603F, 0, &e);
        rd16(s, 0x6041, 0, &st);
        printf("  %5d |            0x%04X    |          0x%04X   | 0x%04X  (fault bit3=%d)\n",
               s, f, e, st, (st >> 3) & 1);
    }

    ecx_close(&ctx);
    return 0;
}
