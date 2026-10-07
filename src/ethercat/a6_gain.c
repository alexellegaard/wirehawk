/* a6_gain.c — read / set the A6 position & speed loop gains (1st gain set).
 *
 * Usage:
 *   sudo ./a6_gain <ifname>                 # read current gains
 *   sudo ./a6_gain <ifname> <kp> <kv>       # set Kp and Kv on BOTH slaves
 *
 * Objects (0x2001, decimal sub-indices, confirmed by a6_gains dump):
 *   0x2001:01  1st position loop gain  Kp   (factory 400)
 *   0x2001:02  1st speed loop gain     Kv   (factory 250)
 *
 * Runs in Safe-OP (motors disabled). Read-only unless you pass <kp> <kv>.
 * Lowering Kp on an unloaded shaft damps the position-loop ring that shows
 * up as the bench "wobble"; the trade-off is a slightly softer response.
 */
#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

static ecx_contextt ctx;

static int rd16(uint16_t slave, uint16_t idx, uint8_t sub, uint16_t* v)
{
    int sz = sizeof(*v);
    *v = 0;
    return ecx_SDOread(&ctx, slave, idx, sub, FALSE, &sz, v, EC_TIMEOUTSAFE) > 0;
}

static int wr16(uint16_t slave, uint16_t idx, uint8_t sub, uint16_t v)
{
    int sz = sizeof(v);
    return ecx_SDOwrite(&ctx, slave, idx, sub, FALSE, sz, &v, EC_TIMEOUTSAFE) > 0;
}

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 4)
    {
        printf("Usage: sudo ./a6_gain <ifname> [kp] [kv]\n");
        return 1;
    }

    printf("Init on %s ...\n", argv[1]);
    if (!ecx_init(&ctx, argv[1])) { printf("no socket\n"); return 1; }
    if (ecx_config_init(&ctx) <= 0) { printf("no slaves\n"); ecx_close(&ctx); return 1; }
    printf("%d slave(s)\n", ctx.slavecount);
    if (ctx.slavecount < 1) { printf("no slaves\n"); ecx_close(&ctx); return 1; }

    {
        uint8_t io_map[4096] = {0};
        ecx_config_map_group(&ctx, io_map, 0);
    }
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n");

    printf("\n== 1st gain set (0x2001:01 Kp / :02 Kv) ==\n");
    for (int s = 1; s <= ctx.slavecount; s++)
    {
        uint16_t kp = 0, kv = 0;
        rd16(s, 0x2001, 1, &kp);
        rd16(s, 0x2001, 2, &kv);
        printf("  slave %d: Kp=%u Kv=%u\n", s, kp, kv);
    }

    if (argc == 4)
    {
        uint16_t kp = (uint16_t)strtoul(argv[2], NULL, 0);
        uint16_t kv = (uint16_t)strtoul(argv[3], NULL, 0);
        printf("\nWriting Kp=%u Kv=%u to ALL slaves ...\n", kp, kv);
        for (int s = 1; s <= ctx.slavecount; s++)
        {
            int oka = wr16(s, 0x2001, 1, kp);
            int okb = wr16(s, 0x2001, 2, kv);
            uint16_t rk = 0, rv = 0;
            rd16(s, 0x2001, 1, &rk);
            rd16(s, 0x2001, 2, &rv);
            printf("  slave %d: wrote Kp=%s Kv=%s -> readback Kp=%u Kv=%u\n",
                   s, oka ? "OK" : "FAIL", okb ? "OK" : "FAIL", rk, rv);
        }
    }

    ecx_close(&ctx);
    return 0;
}
