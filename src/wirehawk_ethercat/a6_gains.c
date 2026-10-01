/* a6_gains.c — dump + diff the A6-EC tuning & identity objects on BOTH drives.
 *
 * Usage: sudo ./a6_gains <ifname>
 *
 * Sweeps, side-by-side for drive A (slave 1) and B (slave 2):
 *   identity objects (0x1008 name, 0x1009 hw rev, 0x100A firmware),
 *   0x2000 (C00 basic/inertia/stiffness), 0x2001 (C01 gains),
 *   0x2002 + 0x2003 (advanced: MFC / observers / vibration suppression),
 *   and a set of CiA-402 behaviour objects (mode, positioning option,
 *   interpolation time, polarity, profile accel/decel, motor type),
 *   plus position-scaling / homing objects (0x6063 internal position,
 *   0x607C home offset, 0x6091 gear ratio, 0x6065..0x6068 windows) to hunt
 *   a per-drive zero-offset / following-window difference.
 * Any value that differs between the two drives is flagged " <<<< DIFF".
 *
 * Runs in Safe-OP (motors stay disabled). Safe with motors idle.
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

static int rd_str(uint16_t slave, uint16_t idx, uint8_t sub, char* buf, int buflen)
{
    memset(buf, 0, (size_t)buflen);
    int sz = buflen - 1;
    int wkc = ecx_SDOread(&ctx, slave, idx, sub, FALSE, &sz, buf, EC_TIMEOUTSAFE);
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

static void ident(uint16_t idx, const char* label)
{
    char a[64], b[64];
    int oka = rd_str(1, idx, 0, a, sizeof(a));
    int okb = rd_str(2, idx, 0, b, sizeof(b));
    const char* mark = (oka != okb) || (oka && okb && strcmp(a, b) != 0) ? "  <<<< DIFF" : "";
    printf("  0x%04X  %-22s  A=%-24s B=%-24s%s\n", idx, label,
           oka ? a : "FAIL", okb ? b : "FAIL", mark);
}

static void cia(uint16_t idx, uint8_t sub, const char* label)
{
    uint32_t a = 0, b = 0;
    int oka = rd32(1, idx, sub, &a);
    int okb = rd32(2, idx, sub, &b);
    char sa[16], sb[16];
    snprintf(sa, sizeof(sa), oka ? "%u" : "FAIL", a);
    snprintf(sb, sizeof(sb), okb ? "%u" : "FAIL", b);
    const char* mark = (oka != okb) || (oka && okb && a != b) ? "  <<<< DIFF" : "";
    printf("  0x%04X:%02d  %-22s  A=%-10s B=%-10s%s\n", idx, sub, label, sa, sb, mark);
}

/* Signed 32-bit variant for DINT objects (home offset, position internal) */
static void cia_s(uint16_t idx, uint8_t sub, const char* label)
{
    int32_t a = 0, b = 0;
    int oka = rd32(1, idx, sub, (uint32_t*)&a);
    int okb = rd32(2, idx, sub, (uint32_t*)&b);
    char sa[16], sb[16];
    snprintf(sa, sizeof(sa), oka ? "%d" : "FAIL", a);
    snprintf(sb, sizeof(sb), okb ? "%d" : "FAIL", b);
    const char* mark = (oka != okb) || (oka && okb && a != b) ? "  <<<< DIFF" : "";
    printf("  0x%04X:%02d  %-22s  A=%-10s B=%-10s%s\n", idx, sub, label, sa, sb, mark);
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

    printf("== identity ==\n");
    ident(0x1008, "device name");
    ident(0x1009, "hardware version");
    ident(0x100A, "software/firmware");

    printf("== 0x2000 (C00: basic / inertia / stiffness) ==\n");
    dump(0x2000, 20);
    printf("== 0x2001 (C01: gain parameters) ==\n");
    dump(0x2001, 20);
    printf("== 0x2002 (C02: advanced tuning) ==\n");
    dump(0x2002, 16);
    printf("== 0x2003 (C03: advanced tuning) ==\n");
    dump(0x2003, 16);

    printf("== CiA-402 behaviour ==\n");
    cia(0x6060, 0, "mode of operation");
    cia(0x60F2, 0, "positioning option");
    cia(0x60C2, 1, "interp time period");
    cia(0x60C2, 2, "interp time index");
    cia(0x60E0, 0, "polarity");
    cia(0x6083, 0, "profile accel");
    cia(0x6084, 0, "profile decel");
    cia(0x6402, 0, "motor type");

    printf("== position scaling / homing (offset candidates) ==\n");
    cia_s(0x6063, 0, "pos actual internal*");   /* raw encoder, before gear ratio */
    cia  (0x6064, 0, "pos actual (user)");      /* what the TxPDO reports */
    cia_s(0x607C, 0, "home offset");            /* shifts zero after homing */
    cia  (0x6091, 1, "gear ratio: motor rev");
    cia  (0x6091, 2, "gear ratio: shaft rev");
    cia  (0x6065, 0, "following err window");
    cia  (0x6066, 0, "following err timeout");
    cia  (0x6067, 0, "position window");
    cia  (0x6068, 0, "position window time");
    cia  (0x6085, 0, "quick stop decel");

    ecx_close(&ctx);
    return 0;
}
