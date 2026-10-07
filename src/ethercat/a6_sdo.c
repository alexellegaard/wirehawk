/* a6_sdo.c — generic SDO read/write on ALL A6-EC slaves (Safe-OP, no motion).
 *
 * Replaces the old one-object tools (a6_gain, a6_filter, a6_set_profile).
 * Reads or writes any object on every slave and prints the per-slave value
 * (and, on write, the readback). Values are unsigned.
 *
 * Usage:
 *   sudo ./a6_sdo <ifname> <idx> <sub> [value] [bytes]
 *     idx/sub : hex (0x…) or decimal
 *     value   : write if given, else read
 *     bytes   : 1, 2 or 4 (default 2 = U16)
 *   sudo ./a6_sdo <ifname> save        # store all params to EEPROM (0x1010:01)
 *
 * Examples (A6 vendor objects, decimal sub-indices):
 *   sudo ./a6_sdo eth0 0x2001 1            # read  Kp (0x2001:01) all slaves
 *   sudo ./a6_sdo eth0 0x2001 1 200        # write Kp = 200
 *   sudo ./a6_sdo eth0 0x2001 2 250        # write Kv = 250
 *   sudo ./a6_sdo eth0 0x2001 17 0         # speed-feedback filter sel = 0
 *   sudo ./a6_sdo eth0 0x2001 19 0         # filter average times = 0
 *   sudo ./a6_sdo eth0 0x2000 6 12         # stiffness level C00.05 = 12
 *   sudo ./a6_sdo eth0 0x2000 7 0          # load-inertia ratio C00.06 = 0
 *   sudo ./a6_sdo eth0 0x6083 0 1310720 4  # profile accel (U32) = 10 rev/s²
 *   sudo ./a6_sdo eth0 save                # store to EEPROM
 *
 * Runs in Safe-OP (motors disabled); never enables the drives.
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

static void usage(void)
{
    printf("Usage:\n");
    printf("  sudo ./a6_sdo <ifname> <idx> <sub> [value] [bytes]   read/write an object\n");
    printf("    idx/sub : hex (0x...) or decimal\n");
    printf("    value   : write if given, else read (unsigned)\n");
    printf("    bytes   : 1, 2 or 4 (default 2 = U16)\n");
    printf("  sudo ./a6_sdo <ifname> save   store all params to EEPROM\n");
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

    /* "save" subcommand: store all params to EEPROM (0x1010:01 = "save"). */
    if (argc == 3 && strcmp(argv[2], "save") == 0)
    {
        uint32_t save = 0x65766173;
        printf("Storing all params to EEPROM on all slaves ...\n");
        for (int s = 1; s <= ctx.slavecount; s++)
        {
            int ok = sdo_write(s, 0x1010, 1, 4, save);
            printf("  slave %d: save %s\n", s, ok ? "OK" : "FAIL");
        }
        ecx_close(&ctx);
        return 0;
    }

    if (argc < 4) { usage(); ecx_close(&ctx); return 1; }

    uint16_t idx = (uint16_t)strtoul(argv[2], NULL, 0);
    uint8_t  sub = (uint8_t)strtoul(argv[3], NULL, 0);
    int write  = (argc >= 5);
    uint32_t val = write ? (uint32_t)strtoul(argv[4], NULL, 0) : 0;
    int bytes = (argc >= 6) ? atoi(argv[5]) : 2;
    if (bytes != 1 && bytes != 2 && bytes != 4)
    {
        printf("bytes must be 1, 2 or 4\n");
        ecx_close(&ctx);
        return 1;
    }

    for (int s = 1; s <= ctx.slavecount; s++)
    {
        if (write)
        {
            int ok  = sdo_write(s, idx, sub, bytes, val);
            uint32_t rb = 0;
            int okr = sdo_read(s, idx, sub, bytes, &rb);
            printf("  slave %d: 0x%04X:%02d write=%u (0x%X) [%s]  readback=%u (0x%X) [%s]\n",
                   s, idx, sub, val, val, ok ? "OK" : "FAIL", rb, rb, okr ? "OK" : "FAIL");
        }
        else
        {
            uint32_t v = 0;
            int ok = sdo_read(s, idx, sub, bytes, &v);
            printf("  slave %d: 0x%04X:%02d = %u (0x%X) [%s]\n",
                   s, idx, sub, v, v, ok ? "OK" : "FAIL");
        }
    }

    ecx_close(&ctx);
    return 0;
}
