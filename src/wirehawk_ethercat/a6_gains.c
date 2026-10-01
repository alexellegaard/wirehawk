/* a6_gains.c — dump + diff the A6-EC tuning objects on BOTH daisy-chained drives.
 *
 * Usage: sudo ./a6_gains <ifname>
 *
 * Reads 0x2000 (C00 group: inertia ratio, stiffness, ...) and 0x2001 (C01
 * group: gain parameters) over CoE SDO, in Safe-OP (motors stay disabled).
 * Prints every sub-index side-by-side for drive A (slave 1) and B (slave 2),
 * flagging any value that differs between the two drives with " <<<< DIFF".
 *
 * Note: 0x2001 sub-indices follow the ESI DT2001 type (not 1..N in a row):
 *   1st gain set = sub 1..4, 2nd gain set = sub 9..12.
 */
#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

static ecx_contextt ctx;

static int rd32(uint16_t slave, uint16_t idx, uint8_t sub, uint32_t* v)
{
    int sz = sizeof(*v);
    *v = 0;
    int wkc = ecx_SDOread(&ctx, slave, idx, sub, FALSE, &sz, v, EC_TIMEOUTSAFE);
    return wkc > 0;
}

static const char* name2001(uint8_t sub)
{
    switch (sub) {
        case 1:  return "1st pos-loop Kp";
        case 2:  return "1st spd-loop Kv";
        case 3:  return "1st spd integral Ti";
        case 4:  return "1st torque filter";
        case 9:  return "2nd pos-loop Kp";
        case 10: return "2nd spd-loop Kv";
        case 11: return "2nd spd integral Ti";
        case 12: return "2nd torque filter";
        case 17: return "spd fbk filter sel";
        case 18: return "spd fbk LP cutoff";
        case 19: return "spd fbk moving-avg";
        default: return "";
    }
}

static void dump(uint16_t idx, uint8_t max_sub)
{
    for (uint8_t s = 1; s <= max_sub; s++) {
        uint32_t a = 0, b = 0;
        int oka = rd32(1, idx, s, &a);
        int okb = rd32(2, idx, s, &b);
        char sa[16], sb[16];
        snprintf(sa, sizeof(sa), oka ? "%u" : "FAIL", a);
        snprintf(sb, sizeof(sb), okb ? "%u" : "FAIL", b);
        const char* mark = (oka != okb) || (oka && okb && a != b) ? "  <<<< DIFF" : "";
        const char* nm = (idx == 0x2001) ? name2001(s) : "";
        printf("  0x%04X:%02d  %-20s  A=%-10s B=%-10s%s\n", idx, s, nm, sa, sb, mark);
    }
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        printf("Usage: sudo ./a6_gains <ifname>\n");
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

    {
        uint8_t io_map[4096] = {0};
        ecx_config_map_group(&ctx, io_map, 0);
    }
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE_OP reached\n");

    printf("== 0x2000 (C00: basic / inertia / stiffness) ==\n");
    dump(0x2000, 20);
    printf("== 0x2001 (C01: gain parameters) ==\n");
    dump(0x2001, 20);

    ecx_close(&ctx);
    return 0;
}
