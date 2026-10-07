/* a6_gains.c — full object-dictionary dump + diff of TWO A6 drives.
 *
 * Usage: sudo ./a6_gains <ifname>
 *
 * Reads every vendor object (0x2000..0x2042) and the CiA-402 / identity
 * objects on both slaves (drive A = slave 1, drive B = slave 2) and flags
 * any value that differs " <<<< DIFF".
 *
 * Sub-index NAMES come from the ESI XML (STEPPERONLINE_A6_Servo_V0.02.xml).
 * NOTE: firmware is V512, ESI is V0.02, so 0x2001/0x2002 names may be shifted
 * vs the real firmware. The <<<< DIFF flag is authoritative — trust the flag.
 *
 * Runs in Safe-OP (motors stay disabled). Safe with motors idle.
 */
#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

static ecx_contextt ctx;
#define MAX_SLAVES 8
static int nslaves;

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
    char buf[MAX_SLAVES][64];
    int ok[MAX_SLAVES];
    int all_ok = 1;
    for (int s = 0; s < nslaves; s++) {
        ok[s] = rd_str(s + 1, idx, 0, buf[s], sizeof(buf[s]));
        if (!ok[s]) all_ok = 0;
    }
    int differs = !all_ok;
    if (all_ok)
        for (int s = 1; s < nslaves; s++)
            if (strcmp(buf[0], buf[s]) != 0) { differs = 1; break; }
    printf("  0x%04X  %-22s", idx, label);
    for (int s = 0; s < nslaves; s++)
        printf("  S%d=%-20s", s + 1, ok[s] ? buf[s] : "FAIL");
    printf("%s\n", differs ? "  <<<< DIFF" : "");
}

static void cia(uint16_t idx, uint8_t sub, const char* label)
{
    uint32_t v[MAX_SLAVES];
    int ok[MAX_SLAVES];
    int all_ok = 1;
    for (int s = 0; s < nslaves; s++) { ok[s] = rd32(s + 1, idx, sub, &v[s]); if (!ok[s]) all_ok = 0; }
    int differs = !all_ok;
    if (all_ok) for (int s = 1; s < nslaves; s++) if (v[s] != v[0]) { differs = 1; break; }
    printf("  0x%04X:%02d  %-22s", idx, sub, label);
    for (int s = 0; s < nslaves; s++) {
        char b[16];
        snprintf(b, sizeof(b), ok[s] ? "%u" : "FAIL", v[s]);
        printf("  S%d=%-10s", s + 1, b);
    }
    printf("%s\n", differs ? "  <<<< DIFF" : "");
}

static void cia_s(uint16_t idx, uint8_t sub, const char* label)
{
    int32_t v[MAX_SLAVES];
    int ok[MAX_SLAVES];
    int all_ok = 1;
    for (int s = 0; s < nslaves; s++) { ok[s] = rd32(s + 1, idx, sub, (uint32_t*)&v[s]); if (!ok[s]) all_ok = 0; }
    int differs = !all_ok;
    if (all_ok) for (int s = 1; s < nslaves; s++) if (v[s] != v[0]) { differs = 1; break; }
    printf("  0x%04X:%02d  %-22s", idx, sub, label);
    for (int s = 0; s < nslaves; s++) {
        char b[16];
        snprintf(b, sizeof(b), ok[s] ? "%d" : "FAIL", v[s]);
        printf("  S%d=%-10s", s + 1, b);
    }
    printf("%s\n", differs ? "  <<<< DIFF" : "");
}

static void dumpobj(const ObjDef* d)
{
    printf("== 0x%04X (%s) ==\n", d->idx, d->name);
    for (uint8_t s = 1; s <= d->max_sub; s++) {
        uint32_t v[MAX_SLAVES];
        int ok[MAX_SLAVES];
        int any_ok = 0;
        for (int m = 0; m < nslaves; m++) {
            ok[m] = rd32(m + 1, d->idx, s, &v[m]);
            if (ok[m]) any_ok = 1;
        }
        if (!any_ok) continue;
        int differs = 0;
        for (int m = 0; m < nslaves; m++) {
            if (!ok[m] || v[m] != v[0]) { differs = 1; break; }
        }
        const char* nm = (s-1 < 64) ? d->sub_name[s-1] : "";
        printf("  0x%04X:%02d  %-36s", d->idx, s, nm);
        for (int m = 0; m < nslaves; m++) {
            char b[16];
            snprintf(b, sizeof(b), ok[m] ? "%u" : "FAIL", v[m]);
            printf("  S%d=%-10s", m + 1, b);
        }
        printf("%s\n", differs ? "  <<<< DIFF" : "");
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
    nslaves = (ctx.slavecount > MAX_SLAVES) ? MAX_SLAVES : ctx.slavecount;
    if (nslaves < 1) {
        printf("no slaves\n");
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
