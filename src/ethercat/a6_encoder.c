/* a6_encoder.c — read the A6-EC absolute-encoder state (Safe-OP, no motion).
 *
 * For the battery-backed multi-turn encoder test: reads the encoder mode
 * selector C00.07 (0x2000:08), the raw single-turn+multi-turn position
 * (0x6063), the user-unit position (0x6064), and the fault/error registers.
 * Optionally writes the mode on ONE slave so only the battery-fitted drive is
 * switched to absolute, leaving the other (no-battery) drives incremental.
 *
 * Usage:
 *   sudo ./a6_encoder <ifname>                 read all slaves
 *   sudo ./a6_encoder <ifname> set <slave> <mode>
 *        write C00.07 = <mode> to <slave> (1-based slave index), print readback
 *
 * Mode values (C00.07 = 0x2000:08, U16, effective on re-power-on):
 *   0 incremental (factory default)
 *   1 absolute linear (multi-turn, needs battery)
 *   2 absolute linear infinite (multi-turn, no overflow alarm, needs battery)
 *   3 absolute single-turn (angle only, NO battery)
 *   4 absolute rotation / 5 absolute mechanical single-turn (need battery)
 *
 * Runs in Safe-OP; never enables the drives.
 */
#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

static ecx_contextt ctx;

static int sdo_read(uint16_t slave, uint16_t idx, uint8_t sub, int bytes, uint32_t *v)
{
    int sz = bytes;
    *v = 0;
    return ecx_SDOread(&ctx, slave, idx, sub, FALSE, &sz, v, EC_TIMEOUTSAFE) > 0;
}

static int sdo_write(uint16_t slave, uint16_t idx, uint8_t sub, int bytes, uint32_t v)
{
    int sz = bytes;
    return ecx_SDOwrite(&ctx, slave, idx, sub, FALSE, sz, &v, EC_TIMEOUTSAFE) > 0;
}

static const char *mode_name(uint32_t m)
{
    switch (m) {
        case 0: return "incremental (factory default)";
        case 1: return "absolute linear (multi-turn, needs battery)";
        case 2: return "absolute linear infinite (multi-turn, no overflow, battery)";
        case 3: return "absolute single-turn (angle only, NO battery)";
        case 4: return "absolute rotation (needs battery)";
        case 5: return "absolute mechanical single-turn (needs battery)";
        default: return "UNKNOWN";
    }
}

/* 17-bit single-turn = 131072 counts/rev; multi-turn is the integer rev count. */
#define COUNTS_PER_REV 131072.0

static void print_slave(int s)
{
    uint32_t mode = 0, raw = 0, pos = 0, fault = 0, err = 0;
    int okm = sdo_read(s, 0x2000, 8, 2, &mode);
    int okr = sdo_read(s, 0x6063, 0, 4, &raw);
    int okp = sdo_read(s, 0x6064, 0, 4, &pos);
    int okf = sdo_read(s, 0x203F, 0, 2, &fault);
    int oke = sdo_read(s, 0x603F, 0, 2, &err);

    int32_t raw_i = (int32_t)raw;
    int32_t pos_i = (int32_t)pos;

    printf("slave %d (motor %d)%s:\n", s, s - 1, (s == ctx.slavecount) ? "  <-- battery-fitted drive" : "");
    printf("  mode  C00.07 (0x2000:08) = %u  %s%s\n",
           mode, mode_name(mode), okm ? "" : "  [read FAIL]");
    printf("  raw   0x6063 = %d counts = %+.3f rev%s\n",
           raw_i, raw_i / COUNTS_PER_REV, okr ? "" : "  [read FAIL]");
    printf("  pos   0x6064 = %d counts = %+.3f rev%s\n",
           pos_i, pos_i / COUNTS_PER_REV, okp ? "" : "  [read FAIL]");
    printf("  fault 0x203F = 0x%04X    error 0x603F = 0x%04X%s\n",
           fault, err, (okf && oke) ? "" : "  [read FAIL]");
}

static void usage(void)
{
    printf("Usage:\n");
    printf("  sudo ./a6_encoder <ifname>                  read all slaves\n");
    printf("  sudo ./a6_encoder <ifname> set <slave> <mode>   write C00.07 on one slave\n");
}

int main(int argc, char **argv)
{
    if (argc < 2) { usage(); return 1; }

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

    /* "set <slave> <mode>": write C00.07 (0x2000:08) on ONE slave. */
    if (argc >= 3 && strcmp(argv[2], "set") == 0)
    {
        if (argc < 5) { usage(); ecx_close(&ctx); return 1; }
        int slave = atoi(argv[3]);
        uint32_t mode = (uint32_t)strtoul(argv[4], NULL, 0);
        if (slave < 1 || slave > ctx.slavecount)
        {
            printf("slave %d out of range (1..%d)\n", slave, ctx.slavecount);
            ecx_close(&ctx);
            return 1;
        }
        int ok = sdo_write(slave, 0x2000, 8, 2, mode);
        uint32_t rb = 0;
        int okr = sdo_read(slave, 0x2000, 8, 2, &rb);
        printf("slave %d (motor %d): C00.07 write=%u (%s) [%s]  readback=%u (%s) [%s]\n",
               slave, slave - 1, mode, mode_name(mode), ok ? "OK" : "FAIL",
               rb, mode_name(rb), okr ? "OK" : "FAIL");
        printf("NOTE: C00.07 takes effect only after re-power-on.\n");
        ecx_close(&ctx);
        return 0;
    }

    for (int s = 1; s <= ctx.slavecount; s++)
    {
        print_slave(s);
        printf("\n");
    }

    ecx_close(&ctx);
    return 0;
}
