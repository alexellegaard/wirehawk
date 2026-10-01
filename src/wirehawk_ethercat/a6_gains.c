/* a6_gains.c — read (and optionally copy) the A6-EC tuning parameters on both
 * daisy-chained drives.
 *
 * Usage:
 *   sudo ./a6_gains <ifname>           print both drives' gains + inertia ratio
 *   sudo ./a6_gains <ifname> copy      copy drive A's params -> drive B, store, verify
 *
 * Reads/writes over CoE SDO only, in Pre-OP (no cyclic PDO, no DC, motors stay
 * disabled), so it is safe to run with the motors idle.
 *
 * Parameters (A6-EC):
 *   0x2001:01..04  1st gain set  (position Kp, speed Kv, speed Ti, torque filter)
 *   0x2001:05..08  2nd gain set  (used when gain switchover is active)
 *   0x2000:06      load inertia ratio (C00.06) — load-specific; copy only if the
 *                  two motors have the SAME mechanical load, otherwise auto-tune
 *                  drive B (panel F30.10) instead.
 */
#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

static ecx_contextt ctx;

static const char* GNAMES[8] = {
    "1st pos-loop Kp",   "1st spd-loop Kv",   "1st spd integral Ti", "1st torque filter",
    "2nd pos-loop Kp",   "2nd spd-loop Kv",   "2nd spd integral Ti", "2nd torque filter",
};

static int rd_u16(uint16_t slave, uint16_t idx, uint8_t sub, uint16_t* v)
{
    int sz = sizeof(*v);
    return ecx_SDOread(&ctx, slave, idx, sub, FALSE, &sz, v, EC_TIMEOUTSAFE) > 0;
}

static int wr_u16(uint16_t slave, uint16_t idx, uint8_t sub, uint16_t v)
{
    return ecx_SDOwrite(&ctx, slave, idx, sub, FALSE, sizeof(v), &v, EC_TIMEOUTSAFE) > 0;
}

static void print_both(void)
{
    uint16_t ia = 0, ib = 0;
    rd_u16(1, 0x2000, 6, &ia);
    rd_u16(2, 0x2000, 6, &ib);
    printf("  %-22s  A=%-6u  B=%-6u\n", "inertia ratio C00.06", ia, ib);
    for (int i = 0; i < 8; i++) {
        uint16_t a = 0, b = 0;
        rd_u16(1, 0x2001, (uint8_t)(i + 1), &a);
        rd_u16(2, 0x2001, (uint8_t)(i + 1), &b);
        printf("  %-22s  A=%-6u  B=%-6u\n", GNAMES[i], a, b);
    }
}

int main(int argc, char** argv)
{
    int docopy = (argc == 3 && strcmp(argv[2], "copy") == 0);
    if (argc < 2 || argc > 3) {
        printf("Usage: sudo ./a6_gains <ifname> [copy]\n");
        return 1;
    }

    printf("Init on %s ...\n", argv[1]);
    if (!ecx_init(&ctx, argv[1])) { printf("no socket\n"); return 1; }
    if (ecx_config_init(&ctx) <= 0) { printf("no slaves\n"); ecx_close(&ctx); return 1; }
    printf("%d slave(s)\n", ctx.slavecount);
    if (ctx.slavecount < 2) {
        printf("need 2 slaves (found %d)\n", ctx.slavecount);
        ecx_close(&ctx);
        return 1;
    }

    printf("== params BEFORE ==\n");
    print_both();

    if (docopy) {
        printf("== copying A (slave 1) -> B (slave 2) ==\n");

        uint16_t iner = 0;
        rd_u16(1, 0x2000, 6, &iner);
        printf("  inertia ratio C00.06 -> %u  [load-specific: only correct if both motors have the SAME load]\n", iner);
        wr_u16(2, 0x2000, 6, iner);

        for (int i = 0; i < 8; i++) {
            uint16_t v = 0;
            rd_u16(1, 0x2001, (uint8_t)(i + 1), &v);
            if (wr_u16(2, 0x2001, (uint8_t)(i + 1), v))
                printf("  %-22s -> %u\n", GNAMES[i], v);
            else
                printf("  %-22s WRITE FAILED\n", GNAMES[i]);
        }

        /* persist to EEPROM: 0x1010:01 = "save all" (0x65766173) */
        uint32_t save = 0x65766173;
        if (ecx_SDOwrite(&ctx, 2, 0x1010, 1, FALSE, sizeof(save), &save, EC_TIMEOUTSAFE) > 0)
            printf("  stored to EEPROM (drive B)\n");
        else
            printf("  EEPROM store FAILED (params are RAM-only until power cycle)\n");

        printf("== params AFTER ==\n");
        print_both();
    }

    ecx_close(&ctx);
    return 0;
}
