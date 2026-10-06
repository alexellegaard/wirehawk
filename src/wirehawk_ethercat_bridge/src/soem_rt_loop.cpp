// soem_rt_loop.cpp — the realtime SOEM CSP loop, run in its own thread.
// Lifted from the standalone a6_csp_rt.c, generalized to N motors, and driven
// by the BridgeData struct instead of printf + a hardcoded sine sweep.
//
// This thread must NOT call any ROS2 API. It reads/writes BridgeData under a
// brief mutex (few-byte copies) and does nothing else but the EtherCAT cycle.

#include "wirehawk_ethercat_bridge/bridge.hpp"

extern "C" {
#include "soem/soem.h"
}

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <mutex>
#include <signal.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>

namespace wirehawk_bridge {

// ---- A6-EC predefined PDO layout (0x1701 / 0x1B01) ----
struct A6_RxPDO {
    uint16_t control_word;     // 0x6040
    int32_t  target_position;  // 0x607A
    uint16_t touch_probe;      // 0x60B8
    uint32_t physical_outputs; // 0x60FE
} __attribute__((packed));     // 12 bytes

struct A6_TxPDO {
    uint16_t error_code;       // 0x603F
    uint16_t status_word;      // 0x6041
    int32_t  position_actual;  // 0x6064
    int16_t  torque_actual;    // 0x6077
    int32_t  following_error;  // 0x60F4
    uint16_t touch_status;     // 0x60B9
    int32_t  touch_pos1;       // 0x60BA
    int32_t  touch_pos2;       // 0x60BC
    uint32_t digital_inputs;   // 0x60FD
} __attribute__((packed));     // 28 bytes

static_assert(sizeof(A6_RxPDO) == 12, "RxPDO must be 12 bytes");
static_assert(sizeof(A6_TxPDO) == 28, "TxPDO must be 28 bytes");

// File-scope config/context, shared by the PO2SO hook and the loop.
static Config        g_cfg;
static ecx_contextt  g_ctx;

// ---- Pre-OP -> Safe-OP hook (per slave): CSP mode + SM-sync + arm DC SYNC0 ----
static int a6_setup(ecx_contextt* context, uint16_t slave)
{
    int8_t   mode      = 8;   // 8 = CSP
    uint16_t sync_mode = 2;   // 2 = DC Sync0
    uint32_t cycle_ns  = (uint32_t)g_cfg.cycle_ns;

    if (ecx_SDOwrite(context, slave, 0x6060, 0, FALSE, sizeof(mode), &mode, EC_TIMEOUTSAFE) > 0)
        printf("RT: slave %d CSP mode (0x6060=8) OK\n", slave);
    else
        printf("RT: slave %d FAILED CSP mode\n", slave);

    ecx_SDOwrite(context, slave, 0x1C32, 1,  FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C32, 2,  FALSE, sizeof(cycle_ns),  &cycle_ns,  EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C32, 10, FALSE, sizeof(cycle_ns),  &cycle_ns,  EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C33, 1,  FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C33, 2,  FALSE, sizeof(cycle_ns),  &cycle_ns,  EC_TIMEOUTSAFE);
    ecx_SDOwrite(context, slave, 0x1C33, 10, FALSE, sizeof(cycle_ns),  &cycle_ns,  EC_TIMEOUTSAFE);

    // Arm DC SYNC0 during the Pre-OP -> Safe-OP transition (the A6-EC validates
    // the DC config here and rejects one armed only after Safe-OP).
    ecx_dcsync0(context, slave, TRUE, g_cfg.cycle_ns, -97000000);
    return 1;
}

// ---- DC sync: PI controller aligning the loop to the slave's SYNC0 ----
static float  pgain = 0.00002f;
static float  igain = 0.00002f;
static int64_t syncoffset = 500000;

static void ec_sync(int64_t reftime, int64_t cycletime, int64_t* offsettime)
{
    static int64_t integral = 0;
    int64_t delta = (reftime - syncoffset) % cycletime;
    if (delta > (cycletime / 2))
        delta -= cycletime;
    integral += -delta;
    *offsettime = (int64_t)((-delta * pgain) + (integral * igain));
}

// ---- Trapezoidal interpolator: one motor's motion state ----
struct MotionState {
    int32_t cmd = 0;    // interpolated command (what we write to the drive)
    double  vel = 0.0;  // current velocity (counts/s)
};

// Advance the interpolated command one cycle toward `target` at the command's
// own velocity (feedforward), bounded by max_speed and max_accel. A small
// position term (kp*err) corrects drift. The previous form derived velocity from
// the position gap (v = err/dt), which decays to zero as the gap closes — on a
// staircase (100 Hz) target that produced a sawtooth "speed up / slow down".
static void step_trapezoid(MotionState& m, int32_t target, double target_vel,
                           double max_speed, double max_accel, double kp, double dt)
{
    double err = (double)target - (double)m.cmd;

    // feedforward velocity + drift correction, clamped to the speed limit
    double v = target_vel + kp * err;
    if (v >  max_speed) v =  max_speed;
    if (v < -max_speed) v = -max_speed;

    // clamp the velocity change to the acceleration limit
    double dv = v - m.vel;
    double amax = max_accel * dt;
    if (dv >  amax) dv =  amax;
    if (dv < -amax) dv = -amax;
    m.vel += dv;

    m.cmd += (int32_t)llround(m.vel * dt);
}

int soem_rt_loop(const Config& cfg, BridgeData* data)
{
    g_cfg = cfg;
    int wkc = 0;
    ec_groupt* grp;

    // ---- realtime setup (this thread) ----
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        perror("mlockall (continuing)");
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(g_cfg.rt_cpu, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == -1)
        perror("sched_setaffinity (continuing)");
    struct sched_param sp{};
    sp.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1)
        perror("sched_setscheduler (continuing)");

    // ---- SOEM init ----
    printf("RT: init on %s ...\n", g_cfg.interface.c_str());
    if (!ecx_init(&g_ctx, g_cfg.interface.c_str())) {
        printf("RT: no socket on %s\n", g_cfg.interface.c_str());
        data->running = false;
        return 1;
    }
    if (ecx_config_init(&g_ctx) <= 0) {
        printf("RT: no slaves found\n");
        ecx_close(&g_ctx);
        data->running = false;
        return 1;
    }
    int N = g_ctx.slavecount;
    printf("RT: %d slave(s)\n", N);
    if (N > MAX_MOTORS) {
        printf("RT: %d slaves exceeds MAX_MOTORS=%d\n", N, MAX_MOTORS);
        ecx_close(&g_ctx);
        data->running = false;
        return 1;
    }
    if (N != g_cfg.num_motors)
        printf("RT: WARNING config num_motors=%d but %d slave(s) found\n", g_cfg.num_motors, N);

    // ---- DC first (clock sync before the P->S transition), then map ----
    ecx_configdc(&g_ctx);
    osal_usleep(500000);

    for (int i = 1; i <= N; i++)
        g_ctx.slavelist[i].PO2SOconfig = a6_setup;
    {
        uint8_t io_map[4096] = {0};
        ecx_config_map_group(&g_ctx, io_map, 0);
    }
    grp = g_ctx.grouplist;
    printf("RT: mapped O=%u I=%u bytes\n", (unsigned)grp->Obytes, (unsigned)grp->Ibytes);
    if (grp->Obytes != (uint32_t)(N * (int)sizeof(A6_RxPDO)) || grp->Ibytes != (uint32_t)(N * (int)sizeof(A6_TxPDO)))
        printf("RT: WARNING PDO sizes don't match structs (O %u vs %zu, I %u vs %zu)\n",
               (unsigned)grp->Obytes, N * sizeof(A6_RxPDO), (unsigned)grp->Ibytes, N * sizeof(A6_TxPDO));

    // Per-slave PDO pointers. Assumes identical drives (uniform 12/28-byte layout),
    // which holds for a chain of A6-EC drives of the same model.
    A6_RxPDO* rx[MAX_MOTORS];
    A6_TxPDO* tx[MAX_MOTORS];
    for (int i = 0; i < N; i++) {
        rx[i] = (A6_RxPDO*)(grp->outputs + i * sizeof(A6_RxPDO));
        tx[i] = (A6_TxPDO*)(grp->inputs  + i * sizeof(A6_TxPDO));
    }

    // ---- Safe-OP: vendor sync-mode + IRQ-loss threshold (per slave) ----
    for (int i = 1; i <= N; i++) {
        uint16_t sync_mode = 2;   // "Sync 2" (host jitter > 1us)
        ecx_SDOwrite(&g_ctx, i, 0x2013, 6,  FALSE, sizeof(sync_mode), &sync_mode, EC_TIMEOUTSAFE);
        uint16_t irq_thr = 10;    // relax the IRQ-loss threshold 5 -> 10
        ecx_SDOwrite(&g_ctx, i, 0x2013, 18, FALSE, sizeof(irq_thr), &irq_thr, EC_TIMEOUTSAFE);
        uint16_t sync_lost = 20;  // relax "Sync lost window" 8 -> 20 (max), the untried sibling of IRQ-loss
        ecx_SDOwrite(&g_ctx, i, 0x2013, 3,  FALSE, sizeof(sync_lost), &sync_lost, EC_TIMEOUTSAFE);
    }
    ecx_statecheck(&g_ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("RT: SAFE_OP reached\n");

    // ---- pre-charge cyclic frames ----
    ecx_send_processdata(&g_ctx);
    ecx_receive_processdata(&g_ctx, EC_TIMEOUTRET);
    for (int i = 0; i < N; i++) {
        rx[i]->target_position = tx[i]->position_actual;
        rx[i]->control_word = 0x0000;
    }
    for (int k = 0; k < 50; k++) {
        ecx_send_processdata(&g_ctx);
        ecx_receive_processdata(&g_ctx, EC_TIMEOUTRET);
        osal_usleep(1000);
    }

    // ---- OPERATIONAL ----
    g_ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&g_ctx, 0);
    ecx_statecheck(&g_ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    printf("RT: OPERATIONAL reached\n");

    // ---- motion state init (hold current position) ----
    MotionState motion[MAX_MOTORS];
    bool fault_reset[MAX_MOTORS] = {false};
    // Auto-zero: on enable, latch raw−logical so the first streamed target maps
    // onto the current position (no startup slew). Makes counts_offset a fixed
    // bias rather than a fragile absolute zero; re-latched every enable.
    int64_t zero_offset[MAX_MOTORS] = {0};
    bool    zeroed[MAX_MOTORS] = {false};
    for (int i = 0; i < N; i++) {
        motion[i].cmd = tx[i]->position_actual;
        motion[i].vel = 0.0;
    }

    // ---- the RT loop (1 kHz, absolute deadline) ----
    int64_t toff = 0;
    int64_t total_cycles = 0, deadline_misses = 0, max_late_ns = 0;  // jitter monitor
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    next.tv_nsec = ((next.tv_nsec / 1000000) + 1) * 1000000;
    double dt = g_cfg.cycle_ns * 1e-9;

    printf("RT: entering %ld Hz CSP loop\n", (long)(1000000000L / g_cfg.cycle_ns));
    while (data->running.load()) {
        next.tv_nsec += g_cfg.cycle_ns + toff;
        if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        // jitter monitor: how late was this wakeup vs the intended deadline?
        {
            struct timespec woken;
            clock_gettime(CLOCK_MONOTONIC, &woken);
            int64_t woken_ns = (int64_t)woken.tv_sec * 1000000000LL + woken.tv_nsec;
            int64_t next_ns  = (int64_t)next.tv_sec  * 1000000000LL + next.tv_nsec;
            int64_t late_ns  = woken_ns - next_ns;
            if (late_ns > 0) {
                deadline_misses++;
                if (late_ns > max_late_ns) max_late_ns = late_ns;
            }
        }
        total_cycles++;
        if (total_cycles % 10000 == 0)   // every ~10 s
            printf("RT jitter: %lld late / %lld cycles (%.4f%%), max late %lld ns\n",
                   (long long)deadline_misses, (long long)total_cycles,
                   100.0 * (double)deadline_misses / (double)total_cycles,
                   (long long)max_late_ns);

        wkc = ecx_receive_processdata(&g_ctx, EC_TIMEOUTRET);
        if (wkc > 0)
            ec_sync(g_ctx.DCtime, g_cfg.cycle_ns, &toff);

        // read the commanded target + feedforward velocity (latest value, brief lock)
        int32_t target[MAX_MOTORS];
        double  target_vel[MAX_MOTORS];
        {
            std::lock_guard<std::mutex> lk(data->mtx);
            for (int i = 0; i < N; i++) {
                target[i]     = data->target_position[i];
                target_vel[i] = data->target_velocity[i];
            }
        }

        bool all_enabled = true;
        for (int i = 0; i < N; i++) {
            uint16_t sw = tx[i]->status_word;
            uint16_t state = sw & 0x006F;

            if (sw & 0x0008) {                       // fault -> toggle reset bit
                fault_reset[i] = !fault_reset[i];
                rx[i]->control_word = fault_reset[i] ? 0x0080 : 0x0000;
                all_enabled = false;
                continue;
            }
            if ((sw & 0x004F) == 0x0040) {           // switch on disabled -> shutdown
                rx[i]->control_word = 0x0006;
                all_enabled = false;
            } else if (state == 0x0021) {            // ready -> switch on
                rx[i]->control_word = 0x0007;
                all_enabled = false;
            } else if (state == 0x0023) {            // switched on -> enable
                rx[i]->control_word = 0x000F;
                motion[i].cmd = tx[i]->position_actual;
                motion[i].vel = 0.0;
                rx[i]->target_position = motion[i].cmd;
                zeroed[i] = false;                   // re-latch offset on this enable
                all_enabled = false;
            } else if (state == 0x0027) {            // operation enabled -> stream
                int32_t tgt = motion[i].cmd;
                if (data->target_valid.load()) {
                    if (!zeroed[i]) {                // auto-zero once per enable
                        zero_offset[i] = (int64_t)tx[i]->position_actual - (int64_t)target[i];
                        zeroed[i] = true;
                        printf("RT: motor %d auto-zero offset = %lld counts\n",
                               i, (long long)zero_offset[i]);
                    }
                    tgt = (int32_t)((int64_t)target[i] + zero_offset[i]);
                }
                step_trapezoid(motion[i], tgt, target_vel[i],
                               g_cfg.max_speed, g_cfg.max_accel,
                               g_cfg.pos_feedback_gain, dt);
                rx[i]->target_position = motion[i].cmd;
                rx[i]->control_word = 0x000F;
            } else {
                all_enabled = false;
            }
        }
        data->enabled = all_enabled;

        // write feedback to the struct (brief lock)
        {
            std::lock_guard<std::mutex> lk(data->mtx);
            for (int i = 0; i < N; i++) {
                // Publish the LOGICAL position (command frame): undo the auto-zero
                // latch and the fixed counts_offset so state/motors is directly
                // comparable to cmd/motors (what the controller's FK expects).
                data->actual_position[i] = (int32_t)((int64_t)tx[i]->position_actual
                                                     - zero_offset[i]
                                                     - g_cfg.counts_offset[i]);
                data->actual_torque[i]   = tx[i]->torque_actual;
                data->status_word[i]     = tx[i]->status_word;
                data->error_code[i]      = tx[i]->error_code;
            }
        }

        ecx_send_processdata(&g_ctx);
    }

    // ---- shutdown: disable, walk state machine down OP -> Safe-OP -> Pre-OP -> INIT ----
    printf("RT: disabling ...\n");
    for (int i = 0; i < N; i++) rx[i]->control_word = 0x0007;
    ecx_send_processdata(&g_ctx);
    ecx_receive_processdata(&g_ctx, EC_TIMEOUTRET);
    osal_usleep(10000);
    for (int i = 0; i < N; i++) rx[i]->control_word = 0x0000;
    ecx_send_processdata(&g_ctx);
    ecx_receive_processdata(&g_ctx, EC_TIMEOUTRET);
    osal_usleep(10000);

    g_ctx.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&g_ctx, 0);
    ecx_statecheck(&g_ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    g_ctx.slavelist[0].state = EC_STATE_PRE_OP;
    ecx_writestate(&g_ctx, 0);
    ecx_statecheck(&g_ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    g_ctx.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&g_ctx, 0);
    ecx_close(&g_ctx);
    printf("RT: done\n");
    return 0;
}

} // namespace wirehawk_bridge
