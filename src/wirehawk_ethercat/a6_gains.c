/* a6_gains.c — full object-dictionary dump + diff of TWO A6 drives.
 *
 * Usage: sudo ./a6_gains <ifname>
 *
 * Reads every vendor object (0x2000..0x2042) and the CiA-402 / identity
 * objects on both slaves (drive A = slave 1, drive B = slave 2) and flags
 * any value that differs " <<<< DIFF".
 *
 * Sub-index NAMES come from the ESI XML (STEPPERONLINE_A6_Servo_V0.02.xml).
 * NOTE: the firmware is V512 and the ESI is V0.02, so for 0x2001/0x2002 the
 * name of a sub-index may be shifted vs the real firmware. The <<<< DIFF flag
 * is authoritative — trust the flag, not the label.
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

typedef struct {
    uint16_t     idx;
    uint8_t      max_sub;
    const char*  name;
    const char*  sub_name[64];
} ObjDef;

static const ObjDef OBJS[] = {
    { 0x2000, 23, "Configuration Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2001, 59, "Gain Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2002, 47, "Gain auto-tuning mode Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2003, 25, "Command Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2004, 29, "InOutput Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2005, 17, "Shutdown Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2006, 23, "Protection Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2010, 23, "Homing Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2013, 27, "Ethercat Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2020, 14, "Motor Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2021, 16, "Drive Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2031, 14, "Operation reset Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2040, 47, "Operation monitoring Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2041, 17, "Condition monitoring Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
    { 0x2042, 19, "Version Parameter",
      { "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "Inverter type identification 1", "", "", "", "", "", "", "", "" } },
};

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

static void dumpobj(const ObjDef* d)
{
    printf("== 0x%04X (%s) ==\n", d->idx, d->name);
    for (uint8_t s = 1; s <= d->max_sub; s++) {
        uint32_t a = 0, b = 0;
        int oka = rd32(1, d->idx, s, &a);
        int okb = rd32(2, d->idx, s, &b);
        if (!oka && !okb) continue;              /* not readable on either -> skip noise */
        char sa[16], sb[16];
        snprintf(sa, sizeof(sa), oka ? "%u" : "FAIL", a);
        snprintf(sb, sizeof(sb), okb ? "%u" : "FAIL", b);
        const char* mark = (oka != okb) || (oka && okb && a != b) ? "  <<<< DIFF" : "";
        const char* nm = (s-1 < 64) ? d->sub_name[s-1] : "";
        printf("  0x%04X:%02d  %-36s  A=%-10s B=%-10s%s\n", d->idx, s, nm, sa, sb, mark);
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

    printf("== identity ==\n");
    ident(0x1008, "device name");
    ident(0x1009, "hardware version");
    ident(0x100A, "software/firmware");

    for (size_t i = 0; i < sizeof(OBJS)/sizeof(OBJS[0]); i++)
        dumpobj(&OBJS[i]);

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
    cia_s(0x6063, 0, "pos actual internal*");
    cia  (0x6064, 0, "pos actual (user)");
    cia_s(0x607C, 0, "home offset");
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
