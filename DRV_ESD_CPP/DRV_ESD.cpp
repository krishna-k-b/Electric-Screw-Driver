/* =========================================================================
 *  DRV8701 + RP2040 Brushed DC Screwdriver Firmware
 *  -------------------------------------------------
 *  Toolchain : Pico SDK (C/C++), no Arduino libraries.
 *  MCU       : Raspberry Pi Pico (RP2040)
 *  Driver    : DRV8701 in PH/EN mode with external N-MOSFET H-bridge
 *
 *  CMSIS-DAP / Picoprobe serial debug
 *  ------------------------------------
 *  Connect a second Pico running Picoprobe firmware:
 *    Target GP0 (TX) -> Picoprobe GP5
 *    Target GP1 (RX) -> Picoprobe GP4
 *  Open the CDC serial port at 115200 baud in your IDE / terminal.
 *  All events are timestamped [T+xxxxx ms].
 *
 *  SO divider: 10 k (top) + 20 k (bottom)
 *    V_adc = V_SO * 20/(10+20) = 0.6667  =>  SO_DIVIDER_RATIO = 0.6667
 *
 *  TRIGGER LOGIC (no separate trigger button):
 *    - CW  button (GP13, active-LOW, internal pull-up) = run CW  while held
 *    - CCW button (GP12, active-LOW, internal pull-up) = run CCW while held
 *    - Both pressed simultaneously => no motion
 *    - Motor also controllable via serial commands (see <help>)
 *
 *  POT: not connected. Threshold defaults to I_THRESHOLD_DEFAULT_A.
 *       Use <ith X.X> serial command to change at runtime.
 *
 *  LED (GP25 onboard):
 *    OFF   = idle / stopped / boot
 *    BLINK = motor energised (STARTING / RUNNING / DITHERING) - 0.5 s period
 *    ON    = FAULT state
 *
 *  Serial commands  (send inside angle brackets, e.g.  <help> ):
 *    <help>      full status dump + command list
 *    <status>    same as <help>
 *    <cw>        start motor CW  (latched until <stop> or trip)
 *    <ccw>       start motor CCW (latched until <stop> or trip)
 *    <stop>      release serial motor request
 *    <ith X.X>   set current threshold in amps  (e.g. <ith 4.5>)
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

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdio.h>
#include <string.h>

/* =========================================================================
 *  CONFIG -- adjust to match your hardware
 * ========================================================================= */

/* ---- Pin map ---- */
static constexpr uint PIN_UART_TX = 0;
static constexpr uint PIN_UART_RX = 1;
/* GP9 (main trigger) intentionally unused -- CW/CCW are the triggers */
static constexpr uint PIN_BTN_CCW = 12;
static constexpr uint PIN_BTN_CW  = 13;
static constexpr uint PIN_EN_PWM  = 16;
static constexpr uint PIN_PH_DIR  = 17;
static constexpr uint PIN_NSLEEP  = 18;
static constexpr uint PIN_NFAULT  = 19;
static constexpr uint PIN_ADC_SO  = 26;
static constexpr uint PIN_ADC_POT = 27; /* wired but reading ignored (POT_CONNECTED=false) */
static constexpr uint ADC_CH_SO   = 0;
static constexpr uint ADC_CH_POT  = 1;
static constexpr uint PIN_LED     = 25; /* onboard LED */

/* ---- Button polarity ---- */
static constexpr int BTN_PRESSED = 0; /* active-LOW with pull-up */

/* ---- Pot / threshold ---- */
static constexpr bool  POT_CONNECTED          = false;
static constexpr float I_THRESHOLD_DEFAULT_A  = 5.0f; /* used when POT_CONNECTED=false */

/* ---- Analog ---- */
static constexpr float ADC_VREF_V       = 3.3f;
static constexpr float ADC_COUNTS       = 4096.0f;
static constexpr float SO_DIVIDER_RATIO = 0.6667f; /* 10k top + 20k bottom */
static constexpr float DRV_AMP_GAIN     = 20.0f;  /* V/V */
static constexpr float R_SENSE_OHM      = 0.005f; /* 5 mohm */
static constexpr float SO_OFFSET_V      = 0.050f; /* V_OFF typical (DS tbl 6.5) */

/* ---- PWM ---- */
/* 125 MHz / (1 * 6250) = 20 kHz */
static constexpr uint16_t PWM_WRAP    = 6249;
static constexpr float    PWM_CLKDIV  = 1.0f;
static constexpr uint16_t PWM_DUTY_MAX = PWM_WRAP;

/* ---- Torque limits ---- */
static constexpr float I_THRESHOLD_MIN_A  = 2.0f;
static constexpr float I_THRESHOLD_MAX_A  = 10.0f;
static constexpr float I_HARD_LIMIT_A     = 14.0f;
static constexpr float DUTY_FLOOR_FOR_I_EST = 0.10f;

/* ---- Timing ---- */
static constexpr uint32_t LOOP_PERIOD_US      = 5000;
static constexpr uint32_t SOFT_START_MS       = 200;
static constexpr uint32_t COOLDOWN_MS         = 300;
static constexpr uint32_t DEBOUNCE_MS         = 25;
static constexpr uint32_t FAULT_HOLDOFF_MS    = 150;
static constexpr uint32_t WAKE_DELAY_MS       = 2;
static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 250;

/* ---- Logging ---- */
static constexpr uint32_t LOG_PERIOD_ACTIVE_MS = 50;  /* 20 Hz while running */
static constexpr uint32_t LOG_PERIOD_IDLE_MS   = 500; /* 2 Hz at idle/stopped */

/* ---- LED ---- */
static constexpr uint32_t LED_BLINK_HALF_MS = 250; /* on 250ms, off 250ms => 0.5s period */

/* ---- Filtering & detection ---- */
static constexpr float IIR_ALPHA            = 0.25f;
static constexpr int   DI_DT_HISTORY        = 8;
static constexpr float DI_DT_TRIP_A_PER_WIN = 4.0f;
static constexpr float DITHER_START_FRAC    = 0.80f;
static constexpr float DITHER_MIN_DUTY_FRAC = 0.30f;
static constexpr int   OVERCURRENT_CONFIRM_MS = 5;

/* ---- UART ---- */
static constexpr uint32_t UART_BAUD = 921600
;
#define UART_ID uart0

/* =========================================================================
 *  TYPES
 * ========================================================================= */

enum class State : uint8_t {
  BOOT,
  IDLE,
  STARTING,
  RUNNING,
  DITHERING,
  STOPPING,
  STOPPED,
  FAULT
};

enum class Direction : uint8_t { CW = 1, CCW = 0 };

struct Button {
  uint  pin;
  bool  stable;
  bool  last_raw;
  absolute_time_t last_change;
  bool  edge_pressed;
  bool  edge_released;
};

struct Context {
  State     state;
  Direction dir;
  uint      pwm_slice;
  uint      pwm_chan;

  Button sw_cw;
  Button sw_ccw;

  /* Serial virtual buttons (latched by <cw>/<ccw>, cleared by <stop>/trip) */
  bool serial_cw_req;
  bool serial_ccw_req;

  /* Conflict tracking (both buttons simultaneously) */
  bool conflict_logged;

  /* Analog */
  float i_filt_a;
  float i_raw_a;
  float i_threshold_a; /* set from pot (if connected) or serial <ith> command */
  float pot_v;
  float so_v;
  float duty_frac;

  /* dI/dt history */
  float i_hist[DI_DT_HISTORY];
  int   i_hist_idx;
  float di_dt_last;

  /* Trip confirmations */
  int overcurrent_ms;
  int fault_holdoff_ms;

  /* PWM control */
  uint16_t duty_target;
  uint16_t duty_current;
  absolute_time_t soft_start_began;

  /* Cooldown */
  absolute_time_t cooldown_began;

  /* Logging */
  absolute_time_t last_log;
  absolute_time_t boot_time;
  uint16_t        last_logged_duty;

  /* Hardware state tracking (avoid repeated log spam) */
  bool nsleep_out;  /* current logical state of nSLEEP pin */

  /* Fault */
  bool        nfault_asserted;
  const char *fault_reason;

  /* Serial command buffer */
  char serial_buf[80];
  int  serial_buf_len;
  bool serial_in_cmd;
};

static Context g{};

/* =========================================================================
 *  TIMESTAMP + LOG MACRO
 * ========================================================================= */

static uint32_t ts_ms(void) {
  return (uint32_t)(absolute_time_diff_us(g.boot_time, get_absolute_time()) /
                    1000);
}

#define LOG(fmt, ...) printf("[T+%7u ms] " fmt "\n", ts_ms(), ##__VA_ARGS__)

/* =========================================================================
 *  LOW-LEVEL HELPERS
 * ========================================================================= */

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/* =========================================================================
 *  TRIGGER LOGIC
 *  Returns true if the currently active direction's trigger is still held.
 *  Physical button OR latched serial request counts.
 * ========================================================================= */

static bool want_cw(void)  { return g.sw_cw.stable  || g.serial_cw_req;  }
static bool want_ccw(void) { return g.sw_ccw.stable  || g.serial_ccw_req; }

static bool trigger_held(void) {
  if (g.dir == Direction::CW)  return want_cw();
  if (g.dir == Direction::CCW) return want_ccw();
  return false;
}

/* =========================================================================
 *  PERIPHERAL INIT
 * ========================================================================= */

static void uart_init_debug(void) {
  uart_init(UART_ID, UART_BAUD);
  gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
  gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);
  uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(UART_ID, true);
  stdio_init_all();
}

static void gpio_init_all(void) {
  /* nSLEEP -- output, force LOW (sleep / Hi-Z) before we are ready */
  gpio_init(PIN_NSLEEP);
  gpio_put(PIN_NSLEEP, 0);
  gpio_set_dir(PIN_NSLEEP, GPIO_OUT);
  g.nsleep_out = false;

  /* PH direction -- output, default CW (HIGH) */
  gpio_init(PIN_PH_DIR);
  gpio_put(PIN_PH_DIR, 1);
  gpio_set_dir(PIN_PH_DIR, GPIO_OUT);

  /* nFAULT -- input with internal + external pull-up */
  gpio_init(PIN_NFAULT);
  gpio_set_dir(PIN_NFAULT, GPIO_IN);
  gpio_pull_up(PIN_NFAULT);

  /* CW / CCW buttons -- input, internal pull-up, active-LOW */
  const uint btns[] = {PIN_BTN_CW, PIN_BTN_CCW};
  for (uint p : btns) {
    gpio_init(p);
    gpio_set_dir(p, GPIO_IN);
    gpio_pull_up(p);
  }

  /* Onboard LED */
  gpio_init(PIN_LED);
  gpio_set_dir(PIN_LED, GPIO_OUT);
  gpio_put(PIN_LED, 0);
}

static void pwm_init_motor(void) {
  gpio_set_function(PIN_EN_PWM, GPIO_FUNC_PWM);
  g.pwm_slice = pwm_gpio_to_slice_num(PIN_EN_PWM);
  g.pwm_chan  = pwm_gpio_to_channel(PIN_EN_PWM);

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_wrap(&cfg, PWM_WRAP);
  pwm_config_set_clkdiv(&cfg, PWM_CLKDIV);
  pwm_init(g.pwm_slice, &cfg, false);

  pwm_set_chan_level(g.pwm_slice, g.pwm_chan, 0);
  pwm_set_enabled(g.pwm_slice, true);
}

static void adc_init_all(void) {
  adc_init();
  adc_gpio_init(PIN_ADC_SO);
  adc_gpio_init(PIN_ADC_POT);
}

/* =========================================================================
 *  ADC + CONVERSION
 * ========================================================================= */

static uint16_t adc_read_channel(uint ch) {
  adc_select_input(ch);
  uint32_t sum = 0;
  for (int i = 0; i < 4; ++i)
    sum += adc_read();
  return (uint16_t)(sum / 4);
}

static float adc_to_volts(uint16_t counts) {
  return ((float)counts / ADC_COUNTS) * ADC_VREF_V;
}

static float volts_to_current_a(float v_so_avg, float duty_frac) {
  if (duty_frac < DUTY_FLOOR_FOR_I_EST)
    return g.i_filt_a; /* freeze below floor duty */
  float v_so_real      = v_so_avg / SO_DIVIDER_RATIO;
  float v_so_during_on = v_so_real / duty_frac;
  float v_across_amp   = v_so_during_on - SO_OFFSET_V;
  if (v_across_amp < 0)
    v_across_amp = 0;
  return v_across_amp / (DRV_AMP_GAIN * R_SENSE_OHM);
}

static float pot_to_threshold_a(float v_pot) {
  float frac = clampf(v_pot / ADC_VREF_V, 0.0f, 1.0f);
  return I_THRESHOLD_MIN_A + frac * (I_THRESHOLD_MAX_A - I_THRESHOLD_MIN_A);
}

/* =========================================================================
 *  BUTTONS (debounced, edge detection)
 * ========================================================================= */

static void button_init(Button &b, uint pin) {
  b.pin           = pin;
  b.stable        = false;
  b.last_raw      = (gpio_get(pin) == BTN_PRESSED);
  b.last_change   = get_absolute_time();
  b.edge_pressed  = false;
  b.edge_released = false;
}

static void button_update(Button &b) {
  bool raw = (gpio_get(b.pin) == BTN_PRESSED);
  absolute_time_t now = get_absolute_time();

  if (raw != b.last_raw) {
    b.last_raw    = raw;
    b.last_change = now;
  }

  bool old_stable = b.stable;
  if (absolute_time_diff_us(b.last_change, now) >= (int64_t)DEBOUNCE_MS * 1000)
    b.stable = raw;

  b.edge_pressed  = (!old_stable && b.stable);
  b.edge_released = (old_stable && !b.stable);
}

/* =========================================================================
 *  MOTOR CONTROL PRIMITIVES
 *  -- LOG only when hardware state actually changes to avoid UART spam
 * ========================================================================= */

static void motor_set_duty(uint16_t duty) {
  if (duty > PWM_DUTY_MAX)
    duty = PWM_DUTY_MAX;
  g.duty_current = duty;
  pwm_set_chan_level(g.pwm_slice, g.pwm_chan, duty);
}

static void motor_set_direction(Direction d) {
  g.dir = d;
  gpio_put(PIN_PH_DIR, d == Direction::CW ? 1 : 0);
  LOG("DIR set -> %s (GP%u = %d)", d == Direction::CW ? "CW" : "CCW",
      PIN_PH_DIR, d == Direction::CW ? 1 : 0);
}

static void motor_brake(void) {
  /* EN=0, nSLEEP=1 → low-side slow decay = brake (DRV8701 Table 1) */
  motor_set_duty(0);
  if (!g.nsleep_out) {
    gpio_put(PIN_NSLEEP, 1);
    g.nsleep_out = true;
  }
  LOG("MOTOR brake  (EN=0, nSLEEP=1, low-side slow decay)");
}

static void motor_sleep(void) {
  motor_set_duty(0);
  if (g.nsleep_out) { /* only act + log if not already sleeping */
    gpio_put(PIN_NSLEEP, 0);
    g.nsleep_out = false;
    LOG("MOTOR sleep  (nSLEEP=0, bridge Hi-Z)");
  }
}

static void motor_wake(void) {
  motor_set_duty(0);
  gpio_put(PIN_NSLEEP, 1);
  g.nsleep_out = true;
  sleep_ms(WAKE_DELAY_MS); /* only at state transition, not in hot loop */
  LOG("MOTOR wake   (nSLEEP=1, t_WAKE %u ms done)", WAKE_DELAY_MS);
}

/* =========================================================================
 *  LED
 *  OFF   = idle/stopped/boot
 *  BLINK = motor active (STARTING/RUNNING/DITHERING)  0.5 s period
 *  ON    = FAULT
 * ========================================================================= */

static void led_update(void) {
  bool is_fault  = (g.state == State::FAULT);
  bool is_active = (g.state == State::STARTING ||
                    g.state == State::RUNNING   ||
                    g.state == State::DITHERING);

  if (is_fault) {
    gpio_put(PIN_LED, 1);
  } else if (is_active) {
    /* toggle every LED_BLINK_HALF_MS */
    bool on = ((ts_ms() / LED_BLINK_HALF_MS) & 1u) == 0;
    gpio_put(PIN_LED, on ? 1 : 0);
  } else {
    gpio_put(PIN_LED, 0);
  }
}

/* =========================================================================
 *  STATE NAMES
 * ========================================================================= */

static const char *state_name(State s) {
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

static void enter_state(State s, const char *reason) {
  if (s == g.state)
    return;
  LOG("STATE  %s -> %s  [%s]", state_name(g.state), state_name(s),
      reason ? reason : "");
  g.state            = s;
  g.last_logged_duty = 0xFFFFu; /* force first telem to log duty */
}

/* =========================================================================
 *  INPUT UPDATE
 * ========================================================================= */

static void update_inputs(void) {
  button_update(g.sw_cw);
  button_update(g.sw_ccw);

  /* Button edge events */
  if (g.sw_cw.edge_pressed)
    LOG("BTN CW  pressed  (GP%u pulled LOW)", PIN_BTN_CW);
  if (g.sw_cw.edge_released)
    LOG("BTN CW  released (GP%u HIGH)", PIN_BTN_CW);
  if (g.sw_ccw.edge_pressed)
    LOG("BTN CCW pressed  (GP%u pulled LOW)", PIN_BTN_CCW);
  if (g.sw_ccw.edge_released)
    LOG("BTN CCW released (GP%u HIGH)", PIN_BTN_CCW);

  /* nFAULT -- open-drain active-LOW */
  bool nf_now = (gpio_get(PIN_NFAULT) == 0);
  if (nf_now != g.nfault_asserted) {
    LOG("nFAULT %s (GP%u = %d)",
        nf_now ? "ASSERTED -- LOW  (DRV8701 fault!)" : "de-asserted -- HIGH (cleared)",
        PIN_NFAULT, nf_now ? 0 : 1);
  }
  g.nfault_asserted = nf_now;

  /* ADC */
  g.so_v = adc_to_volts(adc_read_channel(ADC_CH_SO));
  if (POT_CONNECTED) {
    g.pot_v         = adc_to_volts(adc_read_channel(ADC_CH_POT));
    g.i_threshold_a = pot_to_threshold_a(g.pot_v);
  } else {
    g.pot_v = 0.0f; /* floating, ignored */
    /* g.i_threshold_a is set by <ith> serial command or boot default */
  }

  g.duty_frac = (float)g.duty_current / (float)PWM_WRAP;
  g.i_raw_a   = volts_to_current_a(g.so_v, g.duty_frac);
  g.i_filt_a  = (1.0f - IIR_ALPHA) * g.i_filt_a + IIR_ALPHA * g.i_raw_a;

  /* dI/dt ring buffer */
  g.i_hist[g.i_hist_idx] = g.i_filt_a;
  g.i_hist_idx            = (g.i_hist_idx + 1) % DI_DT_HISTORY;
}

/* =========================================================================
 *  CURRENT SLOPE
 * ========================================================================= */

static float current_slope_window(void) {
  int newest    = (g.i_hist_idx + DI_DT_HISTORY - 1) % DI_DT_HISTORY;
  int oldest    =  g.i_hist_idx;
  float slope   = g.i_hist[newest] - g.i_hist[oldest];
  g.di_dt_last  = slope;
  return slope;
}

/* =========================================================================
 *  SOFT CURRENT LIMIT (DITHERING)
 * ========================================================================= */

static void apply_soft_current_limit(void) {
  float i     = g.i_filt_a;
  float ith   = g.i_threshold_a;
  float scale = 1.0f;
  if (i > DITHER_START_FRAC * ith) {
    float band = (1.0f - DITHER_START_FRAC) * ith;
    scale      = (ith - i) / band;
    scale      = clampf(scale, DITHER_MIN_DUTY_FRAC, 1.0f);
  }
  motor_set_duty((uint16_t)((float)g.duty_target * scale));
}

/* =========================================================================
 *  TORQUE TRIP CHECK
 * ========================================================================= */

static bool torque_trip_check(const char **reason_out) {
  /* Hard absolute limit */
  if (g.i_filt_a >= I_HARD_LIMIT_A) {
    LOG("TRIP hard-limit  I=%.2fA >= %.1fA (FET protection)",
        (double)g.i_filt_a, (double)I_HARD_LIMIT_A);
    *reason_out = "hard limit";
    return true;
  }

  /* Sustained overcurrent -- log first tick and confirmation only */
  if (g.i_filt_a >= g.i_threshold_a) {
    g.overcurrent_ms++;
    if (g.overcurrent_ms == 1)
      LOG("OVERCURRENT start  I=%.2fA >= Ith=%.2fA  (confirm in %d ms)",
          (double)g.i_filt_a, (double)g.i_threshold_a, OVERCURRENT_CONFIRM_MS);
    if (g.overcurrent_ms >= OVERCURRENT_CONFIRM_MS) {
      LOG("TRIP sustained overcurrent  I=%.2fA  held %d ms",
          (double)g.i_filt_a, OVERCURRENT_CONFIRM_MS);
      *reason_out = "I > Ith (sustained)";
      return true;
    }
  } else {
    if (g.overcurrent_ms > 0)
      LOG("OVERCURRENT cleared  (was %d ms, now I=%.2fA < Ith=%.2fA)",
          g.overcurrent_ms, (double)g.i_filt_a, (double)g.i_threshold_a);
    g.overcurrent_ms = 0;
  }

  /* Rapid dI/dt (stall) */
  float slope = current_slope_window();
  if (slope >= DI_DT_TRIP_A_PER_WIN) {
    LOG("TRIP rapid dI/dt  slope=%.2fA / %d ms window  (threshold %.1fA)",
        (double)slope, DI_DT_HISTORY, (double)DI_DT_TRIP_A_PER_WIN);
    *reason_out = "rapid dI/dt (stall)";
    return true;
  }

  return false;
}

/* =========================================================================
 *  SERIAL -- HELP / STATUS DUMP
 * ========================================================================= */

static void serial_print_help(void) {
  uint32_t t = ts_ms();
  printf("\n");
  printf("==========================================================\n");
  printf("  DRV8701 Screwdriver  --  status @ T+%u ms\n", t);
  printf("==========================================================\n");

  /* State */
  printf("  State          : %s\n", state_name(g.state));
  printf("  Direction      : %s\n", g.dir == Direction::CW ? "CW" : "CCW");
  printf("  Fault reason   : %s\n",
         g.fault_reason ? g.fault_reason : "none");

  /* Trigger sources */
  printf("  BTN CW  (GP%u) : %s%s\n", PIN_BTN_CW,
         g.sw_cw.stable    ? "PRESSED" : "released",
         g.serial_cw_req   ? " [serial latched]" : "");
  printf("  BTN CCW (GP%u) : %s%s\n", PIN_BTN_CCW,
         g.sw_ccw.stable   ? "PRESSED" : "released",
         g.serial_ccw_req  ? " [serial latched]" : "");

  /* DRV8701 pin states */
  printf("----------------------------------------------------------\n");
  printf("  DRV8701 pins\n");
  printf("    nSLEEP (GP%u)  : %s  (%s)\n", PIN_NSLEEP,
         g.nsleep_out ? "HIGH (active)" : "LOW  (sleep/Hi-Z)",
         g.nsleep_out ? "bridge enabled" : "bridge Hi-Z");
  printf("    PH/DIR (GP%u)  : %s  (motor %s)\n", PIN_PH_DIR,
         g.dir == Direction::CW ? "HIGH" : "LOW",
         g.dir == Direction::CW ? "CW" : "CCW");
  printf("    EN/PWM (GP%u)  : duty %.1f%%  (%u / %u counts)\n", PIN_EN_PWM,
         100.0 * (double)g.duty_current / (double)PWM_WRAP,
         g.duty_current, PWM_WRAP);
  printf("    nFAULT (GP%u)  : %s\n", PIN_NFAULT,
         g.nfault_asserted ? "LOW  *** FAULT ACTIVE ***" : "HIGH (OK)");

  /* Current sensing */
  printf("----------------------------------------------------------\n");
  printf("  Current sensing\n");
  printf("    SO voltage     : %.3f V  (ADC CH%u GP%u)\n",
         (double)g.so_v, ADC_CH_SO, PIN_ADC_SO);
  printf("    SO divider     : %.4f  (10k top + 20k bottom)\n",
         (double)SO_DIVIDER_RATIO);
  printf("    Amp gain       : %.0f V/V\n", (double)DRV_AMP_GAIN);
  printf("    R_sense        : %.0f mohm\n", (double)(R_SENSE_OHM * 1000.0f));
  printf("    SO offset      : %.0f mV\n",   (double)(SO_OFFSET_V * 1000.0f));
  printf("    I raw          : %.3f A\n", (double)g.i_raw_a);
  printf("    I filtered     : %.3f A  (IIR alpha=%.2f)\n",
         (double)g.i_filt_a, (double)IIR_ALPHA);
  printf("    dI/dt slope    : %+.3f A / %d ms\n",
         (double)g.di_dt_last, DI_DT_HISTORY);
  printf("    dI/dt trip     : %.1f A / %d ms window\n",
         (double)DI_DT_TRIP_A_PER_WIN, DI_DT_HISTORY);

  /* Threshold */
  printf("----------------------------------------------------------\n");
  printf("  Torque threshold\n");
  printf("    Pot connected  : %s\n", POT_CONNECTED ? "YES" : "NO (serial/default)");
  if (POT_CONNECTED)
    printf("    Pot voltage    : %.3f V  (ADC CH%u GP%u)\n",
           (double)g.pot_v, ADC_CH_POT, PIN_ADC_POT);
  printf("    I threshold    : %.2f A  (range %.1f .. %.1f A)\n",
         (double)g.i_threshold_a,
         (double)I_THRESHOLD_MIN_A, (double)I_THRESHOLD_MAX_A);
  printf("    I hard limit   : %.1f A  (unconditional)\n",
         (double)I_HARD_LIMIT_A);
  printf("    OC confirm     : %d ms\n", OVERCURRENT_CONFIRM_MS);
  printf("    OC ms counter  : %d\n", g.overcurrent_ms);
  printf("    Dither start   : %.0f%% of Ith = %.2f A\n",
         (double)(DITHER_START_FRAC * 100.0f),
         (double)(DITHER_START_FRAC * g.i_threshold_a));
  printf("    Dither min duty: %.0f%%\n", (double)(DITHER_MIN_DUTY_FRAC * 100.0f));

  /* PWM / soft-start */
  printf("----------------------------------------------------------\n");
  printf("  PWM / soft-start\n");
  printf("    PWM freq       : 20 kHz  (wrap=%u clkdiv=%.1f)\n",
         PWM_WRAP, (double)PWM_CLKDIV);
  printf("    Soft-start     : %u ms ramp\n", SOFT_START_MS);
  printf("    Brake cooldown : %u ms\n", COOLDOWN_MS);
  printf("    Fault holdoff  : %u ms\n", FAULT_HOLDOFF_MS);

  /* LED */
  printf("----------------------------------------------------------\n");
  printf("  LED (GP%u onboard)\n", PIN_LED);
  printf("    Mode           : %s\n",
         g.state == State::FAULT    ? "ON solid (FAULT)" :
         (g.state == State::STARTING ||
          g.state == State::RUNNING  ||
          g.state == State::DITHERING) ? "BLINKING 0.5s (motor active)" :
                                          "OFF (idle)");

  /* System */
  printf("----------------------------------------------------------\n");
  printf("  System\n");
  printf("    Uptime         : %u ms\n", t);
  printf("    sys_clk        : %u Hz\n", clock_get_hz(clk_sys));
  printf("    Watchdog       : %u ms timeout\n", WATCHDOG_TIMEOUT_MS);
  printf("    Loop period    : %u us\n", LOOP_PERIOD_US);
  printf("    Telem (active) : every %u ms\n", LOG_PERIOD_ACTIVE_MS);
  printf("    Telem (idle)   : every %u ms\n", LOG_PERIOD_IDLE_MS);

  /* Commands */
  printf("----------------------------------------------------------\n");
  printf("  Serial commands (send inside < >)\n");
  printf("    <help>         this screen\n");
  printf("    <status>       same as help\n");
  printf("    <cw>           run motor CW  (latched until <stop>)\n");
  printf("    <ccw>          run motor CCW (latched until <stop>)\n");
  printf("    <stop>         clear serial motor request\n");
  printf("    <ith X.X>      set current threshold  e.g. <ith 4.5>\n");
  printf("==========================================================\n\n");
}

/* =========================================================================
 *  SERIAL -- COMMAND HANDLER
 * ========================================================================= */

static void serial_handle_command(const char *cmd) {
  /* cmd includes the surrounding < > */
  LOG("SERIAL cmd received: %s", cmd);

  if (strcmp(cmd, "<help>") == 0 || strcmp(cmd, "<status>") == 0) {
    serial_print_help();

  } else if (strcmp(cmd, "<cw>") == 0) {
    if (g.state == State::IDLE || g.state == State::STOPPED) {
      g.serial_cw_req  = true;
      g.serial_ccw_req = false;
      LOG("SERIAL CW  request latched -- motor will start next loop tick");
    } else {
      LOG("SERIAL CW  ignored -- state=%s (must be IDLE or STOPPED)",
          state_name(g.state));
    }

  } else if (strcmp(cmd, "<ccw>") == 0) {
    if (g.state == State::IDLE || g.state == State::STOPPED) {
      g.serial_ccw_req = true;
      g.serial_cw_req  = false;
      LOG("SERIAL CCW request latched -- motor will start next loop tick");
    } else {
      LOG("SERIAL CCW ignored -- state=%s (must be IDLE or STOPPED)",
          state_name(g.state));
    }

  } else if (strcmp(cmd, "<stop>") == 0) {
    bool was_active = g.serial_cw_req || g.serial_ccw_req;
    g.serial_cw_req  = false;
    g.serial_ccw_req = false;
    LOG("SERIAL stop -- serial requests cleared%s",
        was_active ? " (will stop motor if serial was the only trigger)" : "");

  } else if (strncmp(cmd, "<ith ", 5) == 0) {
    float val = 0.0f;
    if (sscanf(cmd + 5, "%f", &val) == 1) {
      float clamped = clampf(val, I_THRESHOLD_MIN_A, I_THRESHOLD_MAX_A);
      g.i_threshold_a = clamped;
      LOG("SERIAL threshold set -> %.2f A%s", (double)clamped,
          (val != clamped) ? " (clamped to range)" : "");
    } else {
      LOG("SERIAL <ith> parse error -- usage: <ith 4.5>");
    }

  } else {
    LOG("SERIAL unknown command: %s  (send <help> for list)", cmd);
  }
}

/* =========================================================================
 *  SERIAL -- NON-BLOCKING INPUT POLL
 * ========================================================================= */

static void serial_process(void) {
  int c;
  while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
    if (c == '<') {
      g.serial_buf_len        = 0;
      g.serial_in_cmd         = true;
      g.serial_buf[g.serial_buf_len++] = '<';
    } else if (g.serial_in_cmd) {
      if (c == '>') {
        g.serial_buf[g.serial_buf_len++] = '>';
        g.serial_buf[g.serial_buf_len]   = '\0';
        serial_handle_command(g.serial_buf);
        g.serial_buf_len = 0;
        g.serial_in_cmd  = false;
      } else if (g.serial_buf_len < (int)sizeof(g.serial_buf) - 2) {
        g.serial_buf[g.serial_buf_len++] = (char)c;
      } else {
        /* overflow -- discard */
        g.serial_buf_len = 0;
        g.serial_in_cmd  = false;
        LOG("SERIAL cmd buffer overflow -- discarded");
      }
    }
    /* characters outside < > are ignored */
  }
}

/* =========================================================================
 *  PERIODIC TELEMETRY
 * ========================================================================= */

static void debug_log_periodic(void) {
  bool active = (g.state == State::STARTING || g.state == State::RUNNING ||
                 g.state == State::DITHERING || g.state == State::STOPPING);

  uint32_t period_ms = active ? LOG_PERIOD_ACTIVE_MS : LOG_PERIOD_IDLE_MS;
  absolute_time_t now = get_absolute_time();
  if (absolute_time_diff_us(g.last_log, now) < (int64_t)period_ms * 1000)
    return;
  g.last_log = now;

  float duty_pct    = 100.0f * g.duty_frac;
  float ith_pct     = 100.0f * g.i_filt_a /
                      (g.i_threshold_a > 0 ? g.i_threshold_a : 1.0f);
  float ramp_pct    = 0.0f;
  float cooldown_pct = 0.0f;

  if (g.state == State::STARTING) {
    int64_t el = absolute_time_diff_us(g.soft_start_began, now);
    ramp_pct   = clampf(100.0f * (float)el / (float)(SOFT_START_MS * 1000),
                        0.0f, 100.0f);
  }
  if (g.state == State::STOPPING) {
    int64_t el    = absolute_time_diff_us(g.cooldown_began, now);
    cooldown_pct  = clampf(100.0f * (float)el / (float)(COOLDOWN_MS * 1000),
                           0.0f, 100.0f);
  }

  printf("[T+%7u ms] TELEM %-9s dir=%-3s  "
         "I=%5.2fA(raw=%5.2fA) Ith=%5.2fA(%3.0f%%)  "
         "duty=%5.1f%%  dIdt=%+5.2fA  SO=%5.3fV  nFLT=%d  "
         "nSLP=%d  CW=%d CCW=%d  sCW=%d sCCW=%d",
         ts_ms(),
         state_name(g.state),
         g.dir == Direction::CW ? "CW" : "CCW",
         (double)g.i_filt_a,
         (double)g.i_raw_a,
         (double)g.i_threshold_a,
         (double)ith_pct,
         (double)duty_pct,
         (double)g.di_dt_last,
         (double)g.so_v,
         g.nfault_asserted ? 1 : 0,
         g.nsleep_out ? 1 : 0,
         g.sw_cw.stable  ? 1 : 0,
         g.sw_ccw.stable ? 1 : 0,
         g.serial_cw_req  ? 1 : 0,
         g.serial_ccw_req ? 1 : 0);

  if (g.state == State::STARTING)  printf("  ramp=%3.0f%%",     (double)ramp_pct);
  if (g.state == State::STOPPING)  printf("  cool=%3.0f%%",     (double)cooldown_pct);
  if (g.overcurrent_ms > 0)        printf("  OC_ms=%d",         g.overcurrent_ms);
  if (g.nfault_asserted)           printf("  *** FAULT ***");
  printf("\n");

  /* Warn on meaningful duty change between telem ticks */
  int16_t dd = (int16_t)g.duty_current - (int16_t)g.last_logged_duty;
  if (dd < -50 || dd > 50) {
    LOG("DUTY  %u -> %u  (%.1f%% -> %.1f%%)",
        g.last_logged_duty, g.duty_current,
        100.0f * (float)g.last_logged_duty / (float)PWM_WRAP,
        100.0f * (float)g.duty_current     / (float)PWM_WRAP);
  }
  g.last_logged_duty = g.duty_current;
}

/* =========================================================================
 *  STATE MACHINE
 * ========================================================================= */

static void state_machine_step(void) {
  /* Global nFAULT check -- fires at any time */
  if (g.nfault_asserted && g.state != State::FAULT && g.state != State::BOOT) {
    LOG("FAULT nFAULT asserted by DRV8701  (check VM power, OCP, TSD)");
    g.serial_cw_req  = false; /* clear serial requests on fault */
    g.serial_ccw_req = false;
    motor_brake();
    g.fault_reason     = "nFAULT asserted by DRV8701";
    g.fault_holdoff_ms = 0;
    enter_state(State::FAULT, g.fault_reason);
    return;
  }

  switch (g.state) {

  /* ------------------------------------------------------------------ */
  case State::BOOT:
    enter_state(State::IDLE, "boot complete");
    break;

  /* ------------------------------------------------------------------ */
  case State::IDLE: {
    bool cw_want  = want_cw();
    bool ccw_want = want_ccw();

    if (cw_want && ccw_want) {
      /* Both pressed / requested -- do nothing, log once per change */
      if (!g.conflict_logged) {
        LOG("IDLE CW+CCW conflict -- no motion until one is released");
        g.conflict_logged = true;
      }
      break;
    }
    g.conflict_logged = false; /* reset when conflict resolves */

    if (cw_want || ccw_want) {
      Direction dir = cw_want ? Direction::CW : Direction::CCW;
      const char *src = cw_want
          ? (g.sw_cw.stable ? "BTN CW" : "serial <cw>")
          : (g.sw_ccw.stable ? "BTN CCW" : "serial <ccw>");
      LOG("IDLE trigger -> %s  source=%s  Ith=%.2fA",
          dir == Direction::CW ? "CW" : "CCW", src, (double)g.i_threshold_a);
      motor_set_direction(dir);
      motor_wake();
      g.duty_target  = PWM_DUTY_MAX;
      g.duty_current = 0;
      g.i_filt_a     = 0.0f;
      g.di_dt_last   = 0.0f;
      for (int i = 0; i < DI_DT_HISTORY; ++i)
        g.i_hist[i] = 0.0f;
      g.overcurrent_ms   = 0;
      g.soft_start_began = get_absolute_time();
      enter_state(State::STARTING, "trigger engaged");
    }
    break;
  }

  /* ------------------------------------------------------------------ */
  case State::STARTING: {
    if (!trigger_held()) {
      LOG("STARTING trigger released early -- aborting ramp");
      motor_brake();
      g.cooldown_began = get_absolute_time();
      enter_state(State::STOPPING, "trigger released during start");
      break;
    }
    int64_t elapsed_us =
        absolute_time_diff_us(g.soft_start_began, get_absolute_time());
    float t = (float)elapsed_us / (float)(SOFT_START_MS * 1000);
    if (t >= 1.0f) {
      motor_set_duty(PWM_DUTY_MAX);
      LOG("STARTING ramp complete -> full duty (%u counts = 100%%)", PWM_DUTY_MAX);
      enter_state(State::RUNNING, "soft-start done");
    } else {
      motor_set_duty((uint16_t)(t * (float)PWM_DUTY_MAX));
    }
    if (g.i_filt_a >= I_HARD_LIMIT_A) {
      LOG("STARTING hard limit hit during ramp!  I=%.2fA", (double)g.i_filt_a);
      motor_brake();
      g.cooldown_began = get_absolute_time();
      enter_state(State::STOPPING, "hard limit during start");
    }
    break;
  }

  /* ------------------------------------------------------------------ */
  case State::RUNNING: {
    if (!trigger_held()) {
      LOG("RUNNING trigger released");
      motor_brake();
      g.cooldown_began = get_absolute_time();
      enter_state(State::STOPPING, "trigger released");
      break;
    }
    const char *reason = nullptr;
    if (torque_trip_check(&reason)) {
      g.serial_cw_req  = false;
      g.serial_ccw_req = false;
      motor_brake();
      g.cooldown_began = get_absolute_time();
      enter_state(State::STOPPING, reason);
      break;
    }
    if (g.i_filt_a > DITHER_START_FRAC * g.i_threshold_a) {
      LOG("RUNNING I=%.2fA > %.0f%% of Ith=%.2fA -> entering dither zone",
          (double)g.i_filt_a,
          (double)(DITHER_START_FRAC * 100.0f),
          (double)g.i_threshold_a);
      enter_state(State::DITHERING, "approaching threshold");
      apply_soft_current_limit();
    } else {
      motor_set_duty(g.duty_target);
    }
    break;
  }

  /* ------------------------------------------------------------------ */
  case State::DITHERING: {
    if (!trigger_held()) {
      LOG("DITHERING trigger released");
      motor_brake();
      g.cooldown_began = get_absolute_time();
      enter_state(State::STOPPING, "trigger released");
      break;
    }
    const char *reason = nullptr;
    if (torque_trip_check(&reason)) {
      g.serial_cw_req  = false;
      g.serial_ccw_req = false;
      motor_brake();
      g.cooldown_began = get_absolute_time();
      enter_state(State::STOPPING, reason);
      break;
    }
    if (g.i_filt_a < 0.6f * g.i_threshold_a) {
      LOG("DITHERING I=%.2fA < hysteresis (60%% of Ith=%.2fA) -> full power",
          (double)g.i_filt_a, (double)g.i_threshold_a);
      enter_state(State::RUNNING, "current dropped, resume full");
      motor_set_duty(g.duty_target);
    } else {
      apply_soft_current_limit();
    }
    break;
  }

  /* ------------------------------------------------------------------ */
  case State::STOPPING: {
    int64_t elapsed_us =
        absolute_time_diff_us(g.cooldown_began, get_absolute_time());
    if (elapsed_us >= (int64_t)COOLDOWN_MS * 1000) {
      LOG("STOPPING cooldown complete (%u ms)", COOLDOWN_MS);
      motor_sleep();
      enter_state(State::STOPPED, "cooldown complete");
    }
    break;
  }

  /* ------------------------------------------------------------------ */
  case State::STOPPED:
    /* Wait for trigger-direction button to be released before next run */
    if (!trigger_held()) {
      LOG("STOPPED trigger released -- returning to IDLE");
      enter_state(State::IDLE, "trigger released after stop");
    }
    break;

  /* ------------------------------------------------------------------ */
  case State::FAULT:
    motor_sleep(); /* motor_sleep guards against repeated GPIO writes/logs */
    if (!g.nfault_asserted) {
      g.fault_holdoff_ms++;
      if (g.fault_holdoff_ms == 1)
        LOG("FAULT nFAULT de-asserted -- holding off %u ms before recovery",
            FAULT_HOLDOFF_MS);
      if (g.fault_holdoff_ms >= (int)FAULT_HOLDOFF_MS && !trigger_held()) {
        LOG("FAULT recovered -- holdoff expired + trigger released");
        g.fault_holdoff_ms = 0;
        g.fault_reason     = nullptr;
        enter_state(State::IDLE, "fault cleared");
      }
    } else {
      if (g.fault_holdoff_ms > 0) {
        LOG("FAULT nFAULT re-asserted -- holdoff reset");
      }
      g.fault_holdoff_ms = 0;
    }
    break;
  }
}

/* =========================================================================
 *  MAIN
 * ========================================================================= */

int main(void) {
  /* 1. UART first so everything after is logged */
  uart_init_debug();
  sleep_ms(50);

  g.boot_time = get_absolute_time();

  printf("\n\n");
  printf("=========================================================\n");
  printf("  DRV8701 Screwdriver  -- verbose debug build\n");
  printf("  CMSIS-DAP / Picoprobe  115200 8N1  GP0(TX) GP1(RX)\n");
  printf("=========================================================\n");
  printf("  sys_clk        = %u Hz\n",   clock_get_hz(clk_sys));
  printf("  PWM            = 20 kHz  wrap=%u clkdiv=%.1f\n",
         PWM_WRAP, (double)PWM_CLKDIV);
  printf("  SO divider     = %.4f  (10k top + 20k btm)\n",
         (double)SO_DIVIDER_RATIO);
  printf("  R_sense        = %.0f mohm\n",  (double)(R_SENSE_OHM * 1000.0f));
  printf("  Amp gain       = %.0f V/V\n",   (double)DRV_AMP_GAIN);
  printf("  SO offset      = %.0f mV\n",    (double)(SO_OFFSET_V * 1000.0f));
  printf("  Pot connected  = %s\n",         POT_CONNECTED ? "YES" : "NO");
  printf("  I_thr default  = %.1f A\n",     (double)I_THRESHOLD_DEFAULT_A);
  printf("  I_thr range    = %.1f .. %.1f A\n",
         (double)I_THRESHOLD_MIN_A, (double)I_THRESHOLD_MAX_A);
  printf("  I_hard_limit   = %.1f A\n",     (double)I_HARD_LIMIT_A);
  printf("  Triggers       : CW=GP%u  CCW=GP%u  (active-LOW, internal pull-up)\n",
         PIN_BTN_CW, PIN_BTN_CCW);
  printf("  LED            : GP%u  (blink=running, solid=fault, off=idle)\n",
         PIN_LED);
  printf("  Send <help> for full status + command list\n");
  printf("=========================================================\n\n");

  /* 2. GPIOs */
  gpio_init_all();
  LOG("GPIO init done  (nSLEEP=0, bridge Hi-Z, LED off)");

  /* 3. Peripherals */
  pwm_init_motor();
  LOG("PWM init done  (slice=%u chan=%u @ 20 kHz)",
      pwm_gpio_to_slice_num(PIN_EN_PWM), pwm_gpio_to_channel(PIN_EN_PWM));
  adc_init_all();
  LOG("ADC init done  (SO=CH%u GP%u  POT=CH%u GP%u -- pot %s)",
      ADC_CH_SO, PIN_ADC_SO, ADC_CH_POT, PIN_ADC_POT,
      POT_CONNECTED ? "active" : "ignored");

  /* 4. Software state */
  memset(&g, 0, sizeof(g));
  g.state         = State::BOOT;
  g.dir           = Direction::CW;
  g.pwm_slice     = pwm_gpio_to_slice_num(PIN_EN_PWM);
  g.pwm_chan      = pwm_gpio_to_channel(PIN_EN_PWM);
  g.i_threshold_a = I_THRESHOLD_DEFAULT_A;
  g.boot_time     = get_absolute_time(); /* re-capture after memset */
  button_init(g.sw_cw,  PIN_BTN_CW);
  button_init(g.sw_ccw, PIN_BTN_CCW);
  g.last_log         = get_absolute_time();
  g.last_logged_duty = 0xFFFFu;

  LOG("Default Ith = %.2f A  (use <ith X.X> to change)", (double)g.i_threshold_a);

  /* 5. Watchdog */
  watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
  LOG("Watchdog armed  (%u ms timeout)", WATCHDOG_TIMEOUT_MS);

  /* 6. nFAULT sanity check at boot */
  sleep_ms(10);
  if (gpio_get(PIN_NFAULT) == 0) {
    LOG("*** WARN *** nFAULT is LOW at boot! "
        "Check VM / pull-up / DRV8701 wiring (GP%u)", PIN_NFAULT);
  } else {
    LOG("nFAULT OK (HIGH) at boot  (GP%u = 1)", PIN_NFAULT);
  }

  /* 7. Check if any button is already pressed at boot (log the raw state) */
  LOG("Boot button state:  CW(GP%u)=%d  CCW(GP%u)=%d",
      PIN_BTN_CW,  gpio_get(PIN_BTN_CW),
      PIN_BTN_CCW, gpio_get(PIN_BTN_CCW));

  /* 8. Initial ADC snapshot */
  {
    float so_v  = adc_to_volts(adc_read_channel(ADC_CH_SO));
    float pot_v = adc_to_volts(adc_read_channel(ADC_CH_POT));
    LOG("Initial ADC:  SO=%.3fV (expect ~0V at rest)  "
        "POT=%.3fV %s",
        (double)so_v, (double)pot_v,
        POT_CONNECTED ? "" : "(pot disconnected -- reading ignored)");
  }

  /* 9. Enter IDLE */
  motor_sleep();
  enter_state(State::IDLE, "init done");
  LOG("Ready.  Press CW(GP%u) or CCW(GP%u) to run, or send serial commands.",
      PIN_BTN_CW, PIN_BTN_CCW);

  absolute_time_t next_tick = get_absolute_time();

  /* === Main control loop ============================================== */
  while (true) {
    watchdog_update();

    serial_process();
    update_inputs();
    state_machine_step();
    led_update();
    debug_log_periodic();

    next_tick = delayed_by_us(next_tick, LOOP_PERIOD_US);
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(now, next_tick) > 0) {
      sleep_until(next_tick);
    } else {
      LOG("OVERRUN: loop body exceeded %u us", LOOP_PERIOD_US);
      next_tick = now;
    }
  }

  return 0;
}