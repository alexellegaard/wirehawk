/* a6_filter.c — read / set the A6 speed-feedback filter (smooths the velocity
 * estimate, which is the thing the slave-2 clock jitter is corrupting).
 *
 * Usage:
 *   sudo ./a6_filter <ifname>                 # read current filter params only
 *   sudo ./a6_filter <ifname> <sel> <avg>     # set filter selection + avg times
 *
 * Objects (0x2001, sub-indices decimal 17/18/19 — confirmed by matching the
 * defaults 0 / 8000 / 0 in the a6_gains dump; the manual labels them
 * C01.10/11/12):
 *   0x2001:17  Speed feedback filter selection:
 *               0 = internal setting, 1 = low-pass, 2 = overlapping average,
 *               3 = speed observer, 4 = no filter
 *   0x2001:18  Low-pass cutoff frequency (Hz, 10..16000; 8000 = no filtering)
 *   0x2001:19  Overlapping-average times (0=none, 1=2x, 2=4x, 3=8x, ... 6=64x)
 *
 * Runs in Safe-OP (motors disabled). Read-only unless you pass <sel> <avg>.
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
        printf("Usage: sudo ./a6_filter <ifname> [sel] [avg]\n");
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

    printf("\n== speed feedback filter (0x2001:17 sel / :18 cutoff / :19 avg) ==\n");
    for (int s = 1; s <= ctx.slavecount; s++)
    {
        uint16_t sel = 0, cutoff = 0, avg = 0;
        rd16(s, 0x2001, 17, &sel);
        rd16(s, 0x2001, 18, &cutoff);
        rd16(s, 0x2001, 19, &avg);
        printf("  slave %d: sel=%u cutoff=%u Hz avg=%u\n", s, sel, cutoff, avg);
    }

    if (argc == 4)
    {
        uint16_t sel = (uint16_t)strtoul(argv[2], NULL, 0);
        uint16_t avg = (uint16_t)strtoul(argv[3], NULL, 0);
        printf("\nWriting sel=%u avg=%u to ALL slaves ...\n", sel, avg);
        for (int s = 1; s <= ctx.slavecount; s++)
        {
            int oka = wr16(s, 0x2001, 17, sel);
            int okb = wr16(s, 0x2001, 19, avg);
            uint16_t rs = 0, ra = 0;
            rd16(s, 0x2001, 17, &rs);
            rd16(s, 0x2001, 19, &ra);
            printf("  slave %d: wrote sel=%s avg=%s -> readback sel=%u avg=%u\n",
                   s, oka ? "OK" : "FAIL", okb ? "OK" : "FAIL", rs, ra);
        }
    }

    ecx_close(&ctx);
    return 0;
}
