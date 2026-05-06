/* =========================================================================
 *  DRV8701 + RP2040 Brushed DC Screwdriver Firmware
 *  -------------------------------------------------
 *  Toolchain : Pico SDK (C/C++), no Arduino libraries.
 *  MCU       : Raspberry Pi Pico (RP2040)
 *  Driver    : DRV8701 in PH/EN mode with external N-MOSFET H-bridge
 *
 *  Features
 *    - 20 kHz PWM on EN (inaudible)
 *    - Soft-start ramp
 *    - Pot-based current/torque threshold (2 A .. 10 A)
 *    - Stall detection: absolute threshold + dI/dt slope detection
 *    - Soft "dithering" current limiter near threshold for clean seating
 *    - Brake-then-sleep stop sequence (fast, clean stop at screw seating)
 *    - nFAULT monitoring with retry hold-off
 *    - Direction lock-out while motor is energised (no shoot-through)
 *    - Hardware watchdog
 *    - Non-blocking 1 kHz control loop, UART debug log at 10 Hz
 *
 *  Safety / hardware assumptions (verify on your board!)
 *    - DRV8701 VREF tied to AVDD => disables internal chopper, firmware
 *      handles torque control. SP/SN wired to Rsense as in TI's typical
 *      application (page 28 of SLVSCX5B).
 *    - nFAULT has an external pull-up >= 10 kohm to a logic rail (3.3 V or
 *      AVDD). Internal pull-up is also enabled as a backup.
 *    - All buttons active-LOW with pull-up (internal pull-ups enabled).
 *    - SO output may be scaled with a resistor divider. Configure
 *      SO_DIVIDER_RATIO accordingly. Wrong ratio => wrong torque trip.
 *    - Rsense = 5 mohm, DRV8701 fixed amp gain = 20 V/V.
 *
 *  Build (CMakeLists.txt fragment)
 *    add_executable(screwdriver screwdriver_firmware.cpp)
 *    target_link_libraries(screwdriver PRIVATE
 *        pico_stdlib hardware_pwm hardware_adc hardware_gpio
 *        hardware_uart hardware_watchdog hardware_clocks)
 *    pico_enable_stdio_uart(screwdriver 1)
 *    pico_enable_stdio_usb(screwdriver 0)
 *    pico_add_extra_outputs(screwdriver)
 * ========================================================================= */

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"

/* =========================================================================
 *  CONFIG -- adjust to match your hardware
 * ========================================================================= */

/* ---- Pin map ---- */
static constexpr uint PIN_UART_TX   = 0;
static constexpr uint PIN_UART_RX   = 1;
static constexpr uint PIN_MAIN_SW   = 9;
static constexpr uint PIN_BTN_CCW   = 12;
static constexpr uint PIN_BTN_CW    = 13;
static constexpr uint PIN_EN_PWM    = 16;
static constexpr uint PIN_PH_DIR    = 17;
static constexpr uint PIN_NSLEEP    = 18;
static constexpr uint PIN_NFAULT    = 19;
static constexpr uint PIN_ADC_SO    = 26;
static constexpr uint PIN_ADC_POT   = 27;
static constexpr uint ADC_CH_SO     = 0;
static constexpr uint ADC_CH_POT    = 1;

/* ---- Button polarity ---- */
static constexpr int  BTN_PRESSED   = 0;   // active-LOW with pullup

/* ---- Analog ---- */
static constexpr float ADC_VREF_V       = 3.3f;
static constexpr float ADC_COUNTS       = 4096.0f;
static constexpr float SO_DIVIDER_RATIO = 0.666f;   // V_adc / V_SO. Set to 0.5 for 1:1 divider, etc.
static constexpr float DRV_AMP_GAIN     = 20.0f;  // V/V (DRV8701 fixed)
static constexpr float R_SENSE_OHM      = 0.005f; // 5 mohm
static constexpr float SO_OFFSET_V      = 0.050f; // typical V_OFF (DS table 6.5)

/* ---- PWM ---- */
/* 125 MHz / (clkdiv * (wrap+1)) = 20 kHz  =>  clkdiv=1, wrap=6249 */
static constexpr uint16_t PWM_WRAP        = 6249;
static constexpr float    PWM_CLKDIV      = 1.0f;
static constexpr uint16_t PWM_DUTY_MAX    = PWM_WRAP;       // 100 %

/* ---- Torque mapping ---- */
static constexpr float I_THRESHOLD_MIN_A  = 2.0f;
static constexpr float I_THRESHOLD_MAX_A  = 10.0f;
static constexpr float I_HARD_LIMIT_A     = 14.0f;          // unconditional stop at 14A (if it ever reaches)
static constexpr float DUTY_FLOOR_FOR_I_EST = 0.10f;        // below this, freeze current estimate

/* ---- Timing ---- */
static constexpr uint32_t LOOP_PERIOD_US     = 1000;        // 1 ms control loop
static constexpr uint32_t SOFT_START_MS      = 200;
static constexpr uint32_t COOLDOWN_MS        = 300;
static constexpr uint32_t DEBOUNCE_MS        = 25;
static constexpr uint32_t FAULT_HOLDOFF_MS   = 150;
static constexpr uint32_t WAKE_DELAY_MS      = 2;           // > t_WAKE (1 ms)
static constexpr uint32_t LOG_PERIOD_MS      = 100;
static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 250;

/* ---- Filtering & detection ---- */
static constexpr float    IIR_ALPHA           = 0.25f;
static constexpr int      DI_DT_HISTORY       = 8;          // samples (= ms at 1 kHz)
static constexpr float    DI_DT_TRIP_A_PER_WIN= 4.0f;       // A rise across the window => stall
static constexpr float    DITHER_START_FRAC   = 0.80f;
static constexpr float    DITHER_MIN_DUTY_FRAC= 0.30f;
static constexpr int      OVERCURRENT_CONFIRM_MS = 5;       // dwell at I>Ith before trip

/* ---- UART ---- */
static constexpr uint     UART_ID_NUM         = 0;          // uart0 on GPIO0/1
static constexpr uint32_t UART_BAUD           = 115200;
#define UART_ID  uart0

/* =========================================================================
 *  TYPES
 * ========================================================================= */

enum class State : uint8_t {
    BOOT,
    IDLE,           // driver in sleep, waiting for trigger
    STARTING,       // soft-start ramp
    RUNNING,        // full-power running, monitoring current
    DITHERING,      // soft current limit, near threshold
    STOPPING,       // brake applied, draining inertia
    STOPPED,        // torque event reached; wait for trigger release
    FAULT           // nFAULT asserted or hard limit hit; recoverable on release
};

enum class Direction : uint8_t {
    CW = 1,         // PH = HIGH
    CCW = 0         // PH = LOW
};

struct Button {
    uint     pin;
    bool     stable;            // debounced level (true = pressed)
    bool     last_raw;
    absolute_time_t last_change;
};

struct Context {
    State       state;
    State       prev_logged_state;
    Direction   dir;
    Direction   pending_dir;    // updated only when motor is off
    uint        pwm_slice;
    uint        pwm_chan;

    Button      sw_main;
    Button      sw_cw;
    Button      sw_ccw;

    /* Analog */
    float       i_filt_a;       // filtered motor-side current estimate
    float       i_raw_a;        // last instantaneous estimate (no IIR)
    float       i_threshold_a;  // pot-derived
    float       pot_v;
    float       so_v;

    /* dI/dt history */
    float       i_hist[DI_DT_HISTORY];
    int         i_hist_idx;

    /* Trip confirmations */
    int         overcurrent_ms;
    int         fault_holdoff_ms;

    /* PWM control */
    uint16_t    duty_target;    // requested by state machine
    uint16_t    duty_current;   // actual applied
    absolute_time_t soft_start_began;

    /* Cooldown */
    absolute_time_t cooldown_began;

    /* Logging */
    absolute_time_t last_log;

    /* Latest fault info */
    bool        nfault_asserted;
    const char* fault_reason;
};

static Context g{};

/* =========================================================================
 *  LOW-LEVEL HELPERS
 * ========================================================================= */

static inline uint16_t clamp_u16(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return (uint16_t)lo;
    if (v > hi) return (uint16_t)hi;
    return (uint16_t)v;
}

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* =========================================================================
 *  PERIPHERAL INIT
 * ========================================================================= */

static void uart_init_debug(void)
{
    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
    /* Route stdio (printf) to UART. Requires pico_enable_stdio_uart(1) in CMake. */
    stdio_init_all();
}

static void gpio_init_all(void)
{
    /* All control GPIOs: configure to safe state BEFORE enabling outputs.
       During brown-out / re-boot, the DRV8701 sees pulldowns + nSLEEP=0,
       which puts the bridge into Hi-Z. */

    /* nSLEEP -- output, force low (sleep) until we are ready */
    gpio_init(PIN_NSLEEP);
    gpio_put(PIN_NSLEEP, 0);
    gpio_set_dir(PIN_NSLEEP, GPIO_OUT);

    /* PH -- output, default CW (HIGH) */
    gpio_init(PIN_PH_DIR);
    gpio_put(PIN_PH_DIR, 1);
    gpio_set_dir(PIN_PH_DIR, GPIO_OUT);

    /* nFAULT -- input with pull-up (external 10k recommended) */
    gpio_init(PIN_NFAULT);
    gpio_set_dir(PIN_NFAULT, GPIO_IN);
    gpio_pull_up(PIN_NFAULT);

    /* Buttons -- input, pull-up, active LOW */
    const uint btns[] = {PIN_MAIN_SW, PIN_BTN_CW, PIN_BTN_CCW};
    for (uint p : btns) {
        gpio_init(p);
        gpio_set_dir(p, GPIO_IN);
        gpio_pull_up(p);
    }
}

static void pwm_init_motor(void)
{
    gpio_set_function(PIN_EN_PWM, GPIO_FUNC_PWM);
    g.pwm_slice = pwm_gpio_to_slice_num(PIN_EN_PWM);
    g.pwm_chan  = pwm_gpio_to_channel(PIN_EN_PWM);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_config_set_clkdiv(&cfg, PWM_CLKDIV);
    pwm_init(g.pwm_slice, &cfg, false);  // start disabled

    pwm_set_chan_level(g.pwm_slice, g.pwm_chan, 0);
    pwm_set_enabled(g.pwm_slice, true);
}

static void adc_init_all(void)
{
    adc_init();
    adc_gpio_init(PIN_ADC_SO);
    adc_gpio_init(PIN_ADC_POT);
}

/* =========================================================================
 *  ADC + CONVERSION
 * ========================================================================= */

static uint16_t adc_read_channel(uint ch)
{
    adc_select_input(ch);
    /* RP2040 ADC: ~2 us per conversion. Take 4 samples and average to
       mitigate noise from the 20 kHz PWM front-end. */
    uint32_t sum = 0;
    for (int i = 0; i < 4; ++i) sum += adc_read();
    return (uint16_t)(sum / 4);
}

static float adc_to_volts(uint16_t counts)
{
    return ((float)counts / ADC_COUNTS) * ADC_VREF_V;
}

/* Convert a sense-amplifier voltage into average motor current
 *  V_so = (I * R_sense * G + V_off)        during PWM ON only
 *  Filtered V_so ~= duty * (I*Rs*G + V_off)+ (1-duty)*0
 *  =>  I  ~=  (V_so_filt/duty - V_off) / (Rs * G)
 * Below DUTY_FLOOR_FOR_I_EST, duty correction becomes unstable -> we freeze. */
static float volts_to_current_a(float v_so_avg, float duty_frac)
{
    if (duty_frac < DUTY_FLOOR_FOR_I_EST) return g.i_filt_a;  // freeze
    float v_so_real = v_so_avg / SO_DIVIDER_RATIO;
    float v_so_during_on = v_so_real / duty_frac;
    float v_across_amp   = v_so_during_on - SO_OFFSET_V;
    if (v_across_amp < 0) v_across_amp = 0;
    return v_across_amp / (DRV_AMP_GAIN * R_SENSE_OHM);
}

static float pot_to_threshold_a(float v_pot)
{
    float frac = clampf(v_pot / ADC_VREF_V, 0.0f, 1.0f);
    return I_THRESHOLD_MIN_A + frac * (I_THRESHOLD_MAX_A - I_THRESHOLD_MIN_A);
}

/* =========================================================================
 *  BUTTONS (debounced)
 * ========================================================================= */

static void button_init(Button& b, uint pin)
{
    b.pin = pin;
    b.stable = false;
    b.last_raw = (gpio_get(pin) == BTN_PRESSED);
    b.last_change = get_absolute_time();
}

static void button_update(Button& b)
{
    bool raw = (gpio_get(b.pin) == BTN_PRESSED);
    absolute_time_t now = get_absolute_time();
    if (raw != b.last_raw) {
        b.last_raw = raw;
        b.last_change = now;
    }
    if (absolute_time_diff_us(b.last_change, now) >= (int64_t)DEBOUNCE_MS * 1000) {
        b.stable = raw;
    }
}

/* =========================================================================
 *  MOTOR CONTROL PRIMITIVES
 * ========================================================================= */

static void motor_set_duty(uint16_t duty)
{
    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
    g.duty_current = duty;
    pwm_set_chan_level(g.pwm_slice, g.pwm_chan, duty);
}

static void motor_set_direction(Direction d)
{
    g.dir = d;
    gpio_put(PIN_PH_DIR, d == Direction::CW ? 1 : 0);
}

static void motor_brake(void)
{
    /* nSLEEP=1, EN=0 -> low-side slow decay = brake (Table 1, datasheet). */
    motor_set_duty(0);
    gpio_put(PIN_NSLEEP, 1);
}

static void motor_sleep(void)
{
    motor_set_duty(0);
    gpio_put(PIN_NSLEEP, 0);
}

static void motor_wake(void)
{
    motor_set_duty(0);
    gpio_put(PIN_NSLEEP, 1);
    sleep_ms(WAKE_DELAY_MS);  // only at state transition; not in hot loop
}

/* =========================================================================
 *  STATE LOGGING
 * ========================================================================= */

static const char* state_name(State s)
{
    switch (s) {
        case State::BOOT:      return "BOOT";
        case State::IDLE:      return "IDLE";
        case State::STARTING:  return "STARTING";
        case State::RUNNING:   return "RUNNING";
        case State::DITHERING: return "DITHERING";
        case State::STOPPING:  return "STOPPING";
        case State::STOPPED:   return "STOPPED";
        case State::FAULT:     return "FAULT";
    }
    return "?";
}

static void log_state_transition(State prev, State next, const char* reason)
{
    printf("[STATE] %s -> %s  (%s)\n", state_name(prev), state_name(next),
           reason ? reason : "");
}

static void enter_state(State s, const char* reason)
{
    if (s == g.state) return;
    log_state_transition(g.state, s, reason);
    g.state = s;
}

/* =========================================================================
 *  CONTROL LOOP CORE
 * ========================================================================= */

static void update_inputs(void)
{
    button_update(g.sw_main);
    button_update(g.sw_cw);
    button_update(g.sw_ccw);

    /* Direction selection only takes effect when motor is OFF. */
    bool motor_off =
        (g.state == State::IDLE) ||
        (g.state == State::STOPPED) ||
        (g.state == State::FAULT);

    if (motor_off) {
        bool cw  = g.sw_cw.stable;
        bool ccw = g.sw_ccw.stable;
        if (cw && !ccw)        g.pending_dir = Direction::CW;
        else if (ccw && !cw)   g.pending_dir = Direction::CCW;
        /* if both or neither, keep last value */
    }

    /* nFAULT is open-drain, active LOW. */
    g.nfault_asserted = (gpio_get(PIN_NFAULT) == 0);

    /* Analog */
    g.so_v  = adc_to_volts(adc_read_channel(ADC_CH_SO));
    g.pot_v = adc_to_volts(adc_read_channel(ADC_CH_POT));

    /* Threshold from pot */
    g.i_threshold_a = pot_to_threshold_a(g.pot_v);

    /* Current estimate */
    float duty_frac = (float)g.duty_current / (float)PWM_WRAP;
    g.i_raw_a = volts_to_current_a(g.so_v, duty_frac);

    /* IIR low-pass */
    g.i_filt_a = (1.0f - IIR_ALPHA) * g.i_filt_a + IIR_ALPHA * g.i_raw_a;

    /* Slope window history */
    g.i_hist[g.i_hist_idx] = g.i_filt_a;
    g.i_hist_idx = (g.i_hist_idx + 1) % DI_DT_HISTORY;
}

static float current_slope_window(void)
{
    /* Difference between newest and oldest sample in the ring. */
    int newest = (g.i_hist_idx + DI_DT_HISTORY - 1) % DI_DT_HISTORY;
    int oldest = g.i_hist_idx;  // about to be overwritten next, so this is oldest now
    return g.i_hist[newest] - g.i_hist[oldest];
}

static void apply_soft_current_limit(void)
{
    /* Linear duty scaling between DITHER_START_FRAC*Ith and Ith. */
    float i = g.i_filt_a;
    float ith = g.i_threshold_a;
    float scale = 1.0f;
    if (i > DITHER_START_FRAC * ith) {
        float band = (1.0f - DITHER_START_FRAC) * ith;
        scale = (ith - i) / band;            // 1 -> 0 across the band
        scale = clampf(scale, DITHER_MIN_DUTY_FRAC, 1.0f);
    }
    uint16_t scaled = (uint16_t)((float)g.duty_target * scale);
    motor_set_duty(scaled);
}

/* Returns true if a torque-trip condition is met. */
static bool torque_trip_check(const char** reason_out)
{
    if (g.i_filt_a >= I_HARD_LIMIT_A) {
        *reason_out = "hard limit";
        return true;
    }
    if (g.i_filt_a >= g.i_threshold_a) {
        g.overcurrent_ms++;
        if (g.overcurrent_ms >= OVERCURRENT_CONFIRM_MS) {
            *reason_out = "I > Ith (sustained)";
            return true;
        }
    } else {
        g.overcurrent_ms = 0;
    }
    if (current_slope_window() >= DI_DT_TRIP_A_PER_WIN) {
        *reason_out = "rapid dI/dt (stall)";
        return true;
    }
    return false;
}

/* =========================================================================
 *  STATE MACHINE
 * ========================================================================= */

static void state_machine_step(void)
{
    /* Global fault check first -- nFAULT can fire at any time. */
    if (g.nfault_asserted && g.state != State::FAULT && g.state != State::BOOT) {
        motor_brake();
        g.fault_reason = "nFAULT asserted by DRV8701";
        g.fault_holdoff_ms = 0;
        enter_state(State::FAULT, g.fault_reason);
        return;
    }

    switch (g.state) {

    case State::BOOT: {
        /* Boot completes inside main() before entering the loop. */
        enter_state(State::IDLE, "boot complete");
        break;
    }

    case State::IDLE: {
        /* Driver is asleep, motor disconnected. */
        if (g.sw_main.stable) {
            /* Lock in pending direction now -- will not change until we re-enter IDLE */
            motor_set_direction(g.pending_dir);
            motor_wake();
            g.duty_target  = PWM_DUTY_MAX;
            g.duty_current = 0;
            g.i_filt_a     = 0.0f;
            for (int i = 0; i < DI_DT_HISTORY; ++i) g.i_hist[i] = 0.0f;
            g.overcurrent_ms = 0;
            g.soft_start_began = get_absolute_time();
            enter_state(State::STARTING, "trigger pressed");
        }
        break;
    }

    case State::STARTING: {
        if (!g.sw_main.stable) {
            motor_brake();
            g.cooldown_began = get_absolute_time();
            enter_state(State::STOPPING, "trigger released during start");
            break;
        }

        /* Linear ramp from 0 to PWM_DUTY_MAX over SOFT_START_MS */
        int64_t elapsed_us = absolute_time_diff_us(g.soft_start_began, get_absolute_time());
        float t = (float)elapsed_us / (float)(SOFT_START_MS * 1000);
        if (t >= 1.0f) {
            motor_set_duty(PWM_DUTY_MAX);
            enter_state(State::RUNNING, "soft-start done");
        } else {
            uint16_t d = (uint16_t)(t * (float)PWM_DUTY_MAX);
            motor_set_duty(d);
        }
        /* During soft-start we still honor hard-limit (FET protection) */
        if (g.i_filt_a >= I_HARD_LIMIT_A) {
            motor_brake();
            g.cooldown_began = get_absolute_time();
            enter_state(State::STOPPING, "hard limit during start");
        }
        break;
    }

    case State::RUNNING: {
        if (!g.sw_main.stable) {
            motor_brake();
            g.cooldown_began = get_absolute_time();
            enter_state(State::STOPPING, "trigger released");
            break;
        }
        const char* reason = nullptr;
        if (torque_trip_check(&reason)) {
            motor_brake();
            g.cooldown_began = get_absolute_time();
            enter_state(State::STOPPING, reason);
            break;
        }
        if (g.i_filt_a > DITHER_START_FRAC * g.i_threshold_a) {
            enter_state(State::DITHERING, "approaching threshold");
            apply_soft_current_limit();
        } else {
            motor_set_duty(g.duty_target);
        }
        break;
    }

    case State::DITHERING: {
        if (!g.sw_main.stable) {
            motor_brake();
            g.cooldown_began = get_absolute_time();
            enter_state(State::STOPPING, "trigger released");
            break;
        }
        const char* reason = nullptr;
        if (torque_trip_check(&reason)) {
            motor_brake();
            g.cooldown_began = get_absolute_time();
            enter_state(State::STOPPING, reason);
            break;
        }
        if (g.i_filt_a < 0.6f * g.i_threshold_a) {
            /* Hysteresis: well below the dither band, go full power again. */
            enter_state(State::RUNNING, "current dropped, resume full");
            motor_set_duty(g.duty_target);
        } else {
            apply_soft_current_limit();
        }
        break;
    }

    case State::STOPPING: {
        /* Brake is already applied. Hold brake for COOLDOWN_MS, then sleep. */
        int64_t elapsed_us = absolute_time_diff_us(g.cooldown_began, get_absolute_time());
        if (elapsed_us >= (int64_t)COOLDOWN_MS * 1000) {
            motor_sleep();
            enter_state(State::STOPPED, "cooldown complete");
        }
        break;
    }

    case State::STOPPED: {
        /* Wait for trigger release before allowing a new run. */
        if (!g.sw_main.stable) {
            enter_state(State::IDLE, "trigger released after stop");
        }
        break;
    }

    case State::FAULT: {
        motor_sleep();   // ensure safe state
        if (!g.nfault_asserted) {
            g.fault_holdoff_ms++;
            if (g.fault_holdoff_ms >= (int)FAULT_HOLDOFF_MS && !g.sw_main.stable) {
                g.fault_holdoff_ms = 0;
                enter_state(State::IDLE, "fault cleared, trigger released");
            }
        } else {
            g.fault_holdoff_ms = 0;
        }
        break;
    }
    }
}

/* =========================================================================
 *  DEBUG LOG
 * ========================================================================= */

static void debug_log_periodic(void)
{
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(g.last_log, now) < (int64_t)LOG_PERIOD_MS * 1000) return;
    g.last_log = now;

    float duty_pct = 100.0f * (float)g.duty_current / (float)PWM_WRAP;
    printf("[%-9s] dir=%-3s I=%5.2fA  Ith=%5.2fA  duty=%5.1f%%  pot=%4.2fV  so=%4.2fV  nFAULT=%d\n",
           state_name(g.state),
           g.dir == Direction::CW ? "CW" : "CCW",
           (double)g.i_filt_a,
           (double)g.i_threshold_a,
           (double)duty_pct,
           (double)g.pot_v,
           (double)g.so_v,
           g.nfault_asserted ? 1 : 0);
}

/* =========================================================================
 *  MAIN
 * ========================================================================= */

int main(void)
{
    /* --- 1. UART up first so we can log everything --- */
    uart_init_debug();
    sleep_ms(50);
    printf("\n\n=== DRV8701 Screwdriver firmware boot ===\n");
    printf("sys_clk = %u Hz\n", clock_get_hz(clk_sys));
    printf("PWM = 20 kHz, wrap=%u, clkdiv=%.2f\n", PWM_WRAP, (double)PWM_CLKDIV);

    /* --- 2. GPIOs into safe state (nSLEEP=0) --- */
    gpio_init_all();

    /* --- 3. Peripherals --- */
    pwm_init_motor();
    adc_init_all();

    /* --- 4. Software state --- */
    memset(&g, 0, sizeof(g));
    g.state         = State::BOOT;
    g.dir           = Direction::CW;
    g.pending_dir   = Direction::CW;
    g.pwm_slice     = pwm_gpio_to_slice_num(PIN_EN_PWM);
    g.pwm_chan      = pwm_gpio_to_channel(PIN_EN_PWM);
    button_init(g.sw_main, PIN_MAIN_SW);
    button_init(g.sw_cw,   PIN_BTN_CW);
    button_init(g.sw_ccw,  PIN_BTN_CCW);
    g.last_log      = get_absolute_time();

    /* --- 5. Watchdog --- */
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    /* --- 6. Sanity: confirm nFAULT not stuck low (would be a wiring issue) --- */
    sleep_ms(10);
    if (gpio_get(PIN_NFAULT) == 0) {
        printf("[WARN] nFAULT is asserted at boot. Check pull-up / VM / wiring.\n");
    }

    /* --- 7. Enter IDLE: driver stays in sleep until trigger pulled --- */
    motor_sleep();
    enter_state(State::IDLE, "init done");

    absolute_time_t next_tick = get_absolute_time();

    /* === Main control loop ================================================ */
    while (true) {
        watchdog_update();

        update_inputs();
        state_machine_step();
        debug_log_periodic();

        /* Sleep until the next 1 ms tick. Non-blocking style: we compute the
           next scheduled time and sleep_until it. If we ever overrun, we
           re-base immediately so we don't accumulate lag. */
        next_tick = delayed_by_us(next_tick, LOOP_PERIOD_US);
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(now, next_tick) > 0) {
            sleep_until(next_tick);
        } else {
            next_tick = now;  // overrun; reset
        }
    }
    /* unreachable */
    return 0;
}

// need to check