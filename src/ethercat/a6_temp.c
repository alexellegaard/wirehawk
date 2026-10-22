/* a6_temp.c — read the A6-EC drive heatsink temperature (Safe-OP, no motion).
 *
 * Reads U40.30 "Heatsink temperature" (SDO 0x2040, sub-index 0x31) — an I16
 * in 0.1 °C units (raw 244 = 24.4 °C) — and prints °C per slave.
 *
 * Usage: sudo ./a6_temp <ifname>
 */
#include "soem/soem.h"
#include <stdio.h>
#include <stdint.h>

static ecx_contextt ctx;

static int sdo_read(uint16_t slave, uint16_t idx, uint8_t sub, int bytes, uint32_t *v)
{
    int sz = bytes;
    *v = 0;
    return ecx_SDOread(&ctx, slave, idx, sub, FALSE, &sz, v, EC_TIMEOUTSAFE) > 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) { printf("Usage: sudo %s <ifname>\n", argv[0]); return 1; }

    printf("Init on %s ...\n", argv[1]);
    if (!ecx_init(&ctx, argv[1])) { printf("no socket\n"); return 1; }
    if (ecx_config_init(&ctx) <= 0) { printf("no slaves\n"); ecx_close(&ctx); return 1; }
    printf("%d slave(s)\n", ctx.slavecount);
    {
        uint8_t io_map[4096] = {0};
        ecx_config_map_group(&ctx, io_map, 0);
    }
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n");

    for (int s = 1; s <= ctx.slavecount; s++) {
        uint32_t raw32 = 0;
        if (sdo_read(s, 0x2040, 0x31, 2, &raw32)) {
            int16_t raw = (int16_t)(raw32 & 0xFFFF);
            printf("slave %d (motor %d): heatsink temp = %6.1f C (raw %d)\n",
                   s, s - 1, raw / 10.0, raw);
        } else {
            printf("slave %d (motor %d): read FAIL\n", s, s - 1);
        }
    }

    ecx_close(&ctx);
    return 0;
}
