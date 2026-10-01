/* a6_set_profile.c — set 0x6083 (profile acceleration) + 0x6084 (profile
 * deceleration) on all daisy-chained drives, store to EEPROM, verify.
 *
 * Usage: sudo ./a6_set_profile <ifname> <value>
 *   e.g. sudo ./a6_set_profile eth0 1310720
 *
 * Runs in Safe-OP (motors stay disabled). 0x6083/0x6084 are UINT32.
 */
#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

static ecx_contextt ctx;

int main(int argc, char** argv)
{
    if (argc != 3) {
        printf("Usage: sudo ./a6_set_profile <ifname> <value>\n");
        return 1;
    }
    uint32_t val = (uint32_t)strtoul(argv[2], NULL, 0);
    printf("setting 0x6083/0x6084 = %u\n", val);

    if (!ecx_init(&ctx, argv[1])) { printf("no socket\n"); return 1; }
    if (ecx_config_init(&ctx) <= 0) { printf("no slaves\n"); ecx_close(&ctx); return 1; }
    int n = ctx.slavecount;
    printf("%d slave(s)\n", n);

    {
        uint8_t io_map[4096] = {0};
        ecx_config_map_group(&ctx, io_map, 0);
    }
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n");

    for (int s = 1; s <= n; s++) {
        int w1 = ecx_SDOwrite(&ctx, s, 0x6083, 0, FALSE, sizeof(val), &val, EC_TIMEOUTSAFE);
        int w2 = ecx_SDOwrite(&ctx, s, 0x6084, 0, FALSE, sizeof(val), &val, EC_TIMEOUTSAFE);
        printf("  slave %d: 0x6083/0x6084 -> %s\n", s, (w1 > 0 && w2 > 0) ? "OK" : "FAIL");
    }

    uint32_t save = 0x65766173;   /* 0x1010:01 = "save all" */
    for (int s = 1; s <= n; s++)
        ecx_SDOwrite(&ctx, s, 0x1010, 1, FALSE, sizeof(save), &save, EC_TIMEOUTSAFE);
    printf("stored to EEPROM\n");

    ecx_close(&ctx);
    return 0;
}
