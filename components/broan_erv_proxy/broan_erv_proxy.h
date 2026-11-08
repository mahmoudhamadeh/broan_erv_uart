#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace esphome {
namespace broan_erv_proxy {

static const char *const TAG = "broan_proxy";

struct RegisterRecord {
  uint16_t key{0};
  std::vector<uint8_t> value;
  bool fetched_from_erv{false};
  bool seeded{false};   // value came from seed_defaults_(), not from a cache-miss placeholder
  uint32_t read_count_wall{0};
  uint32_t write_count_wall{0};
};

struct PendingWrite {
  uint16_t key{0};
  std::vector<uint8_t> value;
};

struct SentFrameRecord {
  std::vector<uint8_t> bytes;
  uint32_t sent_ms{0};
};

// ---------------------------------------------------------------------------------------------
// "Nest in charge" decisions (used from the YAML). Pure logic, no bus access, so it is testable
// on a PC. Mode bytes: 0x01 Standby, 0x09 Continuous LOW, 0x0B MED, 0x0A HIGH.
//
//  - While the Nest calls: speed by outdoor temperature (>= 5 C HIGH, 5..-10 C MED, < -10 C LOW,
//    1 C either side of each threshold as hysteresis). Call ends: Standby.
//  - Outdoor temperature = intake temperature, but only once both fans have moved air for 2 min
//    (when idle the intake sensor reads warm air sitting in the unit). Until then: MED when starting
//    from Standby or another mode; a Continuous speed already running is kept (a reboot during a
//    call changes nothing). The first decision after the temperature becomes known is applied at
//    once (it replaces a guess; the 5-min limit is only there to stop flapping).
//  - Call edges (and enabling "Nest in charge", boot, end of a bypass) act at once. During a call
//    the speed is re-checked periodically and changed at most every 5 min, and only while the ERV
//    is still in the mode we set (a manual change is respected until the next call edge).
//  - Nothing changes while the ERV is defrosting; the pending change runs when the defrost ends
//    (at most 15 min later), so a defrost is never cut short by Standby.
// ---------------------------------------------------------------------------------------------
class NestVentController {
 public:
  static constexpr uint32_t SETTLE_MS = 120000;
  static constexpr uint32_t MIN_CHANGE_MS = 300000;
  static constexpr uint32_t DEFROST_MAX_HOLD_MS = 900000;

  // Call every ~10 s with the offset-corrected intake temperature and both airflows.
  void observe(uint32_t now, float intake_c, float supply_cfm, float exhaust_cfm) {
    const bool flowing = !std::isnan(intake_c) && !std::isnan(supply_cfm) && !std::isnan(exhaust_cfm) &&
                         supply_cfm > 20.0f && exhaust_cfm > 20.0f;
    if (flowing && !this->flowing_) this->flow_since_ = now;
    this->flowing_ = flowing;
    if (flowing && now - this->flow_since_ >= SETTLE_MS) {
      if (std::isnan(this->outdoor_c_)) {
        ESP_LOGI("nest", "Outdoor temperature known: %.1f C (intake, fans running 2 min)", intake_c);
        this->fresh_outdoor_ = true;   // next speed decision may skip the 5-min limit once
      }
      this->outdoor_c_ = intake_c;
    }
  }

  // Returns the mode byte to write now, or 0 for "nothing to do".
  uint8_t decide(uint32_t now, bool call, bool edge, uint8_t current_mode, bool defrost) {
    if (defrost && !this->defrost_prev_) this->defrost_since_ = now;
    this->defrost_prev_ = defrost;
    const bool hold = defrost && (now - this->defrost_since_ < DEFROST_MAX_HOLD_MS);
    const bool continuous = current_mode == 0x09 || current_mode == 0x0B || current_mode == 0x0A;
    uint8_t target;
    if (!call) target = 0x01;
    else if (std::isnan(this->outdoor_c_) && continuous) target = current_mode;   // keep it until known
    else target = this->band_mode_();

    if (edge) this->pending_ = true;
    if (this->pending_) {
      if (hold) {
        if (!this->hold_logged_) {
          ESP_LOGI("nest", "Defrost running: %s waits until it ends", mode_name(target));
          this->hold_logged_ = true;
        }
        return 0;
      }
      this->pending_ = false;
      this->hold_logged_ = false;
      this->last_target_ = target;
      this->last_change_ms_ = now;
      if (current_mode == target) return 0;
      ESP_LOGI("nest", "Ventilation call %s -> %s (outdoor %s)", call ? "ON" : "OFF", mode_name(target),
               this->outdoor_str_().c_str());
      return target;
    }

    // Periodic re-check: only adjust the speed of a call we are running ourselves.
    if (!call || this->last_target_ == 0x01 || current_mode != this->last_target_) return 0;
    if (target == current_mode) {
      this->fresh_outdoor_ = false;   // the guess was right: nothing to correct
      return 0;
    }
    if (hold) return 0;
    if (!this->fresh_outdoor_ && now - this->last_change_ms_ < MIN_CHANGE_MS) return 0;
    ESP_LOGI("nest", "Outdoor %s -> speed %s%s", this->outdoor_str_().c_str(), mode_name(target),
             this->fresh_outdoor_ ? " (outdoor temperature just learned)" : "");
    this->fresh_outdoor_ = false;
    this->last_target_ = target;
    this->last_change_ms_ = now;
    return target;
  }

  float outdoor_c() const { return this->outdoor_c_; }
  bool pending() const { return this->pending_; }

  static const char *mode_name(uint8_t m) {
    switch (m) {
      case 0x01: return "Standby";
      case 0x09: return "Continuous LOW";
      case 0x0B: return "Continuous MED";
      case 0x0A: return "Continuous HIGH";
      default:   return "?";
    }
  }

 protected:
  uint8_t band_mode_() {
    const float t = this->outdoor_c_;
    if (std::isnan(t)) {
      this->band_ = 1;                // MED until the outdoor temperature is known
      this->band_provisional_ = true;
    } else if (this->band_ < 0 || this->band_provisional_) {
      this->band_ = t >= 5.0f ? 0 : (t >= -10.0f ? 1 : 2);
      this->band_provisional_ = false;
    } else if (this->band_ == 0 && t < 4.0f) {
      this->band_ = 1;
    } else if (this->band_ == 1 && t >= 6.0f) {
      this->band_ = 0;
    } else if (this->band_ == 1 && t < -11.0f) {
      this->band_ = 2;
    } else if (this->band_ == 2 && t >= -9.0f) {
      this->band_ = 1;
    }
    return this->band_ == 0 ? 0x0A : (this->band_ == 1 ? 0x0B : 0x09);
  }

  std::string outdoor_str_() const {
    if (std::isnan(this->outdoor_c_)) return "unknown";
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f C", this->outdoor_c_);
    return buf;
  }

  float outdoor_c_{NAN};
  bool flowing_{false};
  uint32_t flow_since_{0};
  int band_{-1};                 // 0 HIGH, 1 MED, 2 LOW
  bool band_provisional_{false};
  bool fresh_outdoor_{false};    // outdoor temperature just learned: first correction skips the 5-min limit
  bool pending_{false};
  bool hold_logged_{false};
  uint8_t last_target_{0};
  uint32_t last_change_ms_{0};
  bool defrost_prev_{false};
  uint32_t defrost_since_{0};
};

class BroanErvProxy : public Component {
 public:
  void set_uart_erv(uart::UARTComponent *u) { this->uart_a_ = u; }
  void set_uart_wall(uart::UARTComponent *u) { this->uart_b_ = u; }

  void set_wall_temp_sensor(sensor::Sensor *s) { this->wall_temp_sensor_ = s; }
  void set_wall_humidity_sensor(sensor::Sensor *s) { this->wall_humidity_sensor_ = s; }
  void set_erv_temp_sensor(sensor::Sensor *s) { this->erv_temp_sensor_ = s; }
  void set_supply_cfm_sensor(sensor::Sensor *s) { this->supply_cfm_sensor_ = s; }
  void set_exhaust_cfm_sensor(sensor::Sensor *s) { this->exhaust_cfm_sensor_ = s; }
  void set_filter_days_sensor(sensor::Sensor *s) { this->filter_days_sensor_ = s; }

  void setup() override {
    this->boot_.t0 = millis();
    this->high_freq_.start();
    this->seed_defaults_();
    this->build_poll_batches_();

    const uint32_t now = millis();
    this->bus_a_last_ms_ = now;
    this->bus_b_last_ms_ = now;
    this->last_summary_ms_ = now;
    this->last_sensor_pub_ms_ = now;

    ESP_LOGI(TAG, "Broan Dual-Bus ERV Proxy initialized (Bus A=Controller 0x12, Bus B=ERV 0x10).");
  }

  uint8_t get_current_mode_byte() const {
    auto it = this->regs_.find(0x0020);
    if (it != this->regs_.end() && !it->second.value.empty()) {
      return it->second.value[0];
    }
    return 0x08;
  }

  // Modes a user can pick on the screen (or in HA).
  static bool is_selectable_mode(uint8_t m) {
    return m == 0x01 || m == 0x08 || m == 0x09 || m == 0x0A || m == 0x0B || m == 0x0C || m == 0x0F || m == 0x11;
  }

  // The mode the user selected. While the ERV runs an automatic state instead (0x14 starting up,
  // 0x0D dehumidifying, 0x0E humidistat hold), 0x0020 shows that state; the selected mode is then
  // taken from 0x0220 (the mode the ERV returns to, seen during turbo) if it holds a selectable mode.
  // Returns 0 when neither register holds one (caller keeps showing the last known mode).
  uint8_t get_selected_mode_byte() const {
    uint8_t m = this->get_current_mode_byte();
    if (is_selectable_mode(m)) return m;
    uint8_t base = this->get_u8_reg_(0x0220);
    return is_selectable_mode(base) ? base : 0;
  }

  // Raw fan state reported by the ERV (0x0720): 0 = stopped, 1 = LOW, 4 = MED, 2 = HIGH, 3 = TURBO.
  uint8_t get_fan_state() const {
    auto it = this->regs_.find(0x0720);
    return (it != this->regs_.end() && !it->second.value.empty()) ? it->second.value[0] : 0x00;
  }

  // True when HA's Fan Speed slider does something in the current mode (Intermittent or Continuous).
  bool fan_speed_adjustable() const {
    uint8_t m = this->get_current_mode_byte();
    return m == 0x08 || m == 0x09 || m == 0x0A || m == 0x0B;
  }

  // Returns unified Fan Speed: 1 = Low, 2 = Medium, 3 = High
  uint8_t get_fan_speed_level() const {
    uint8_t m = this->get_current_mode_byte();
    if (m == 0x09) return 1; // Continuous Low
    if (m == 0x0B) return 2; // Continuous Medium
    if (m == 0x0A || m == 0x0C) return 3; // Continuous High or Turbo

    // SMART, Away, OFF: the ERV picks the speed itself, so show what it is actually running,
    // from the fan state 0x0720 (1=LOW, 4=MED, 2=HIGH, 3=TURBO, 0=stopped).
    if (m != 0x08) {
      auto st = this->regs_.find(0x0720);
      if (st != this->regs_.end() && !st->second.value.empty()) {
        switch (st->second.value[0]) {
          case 0x01: return 1;
          case 0x04: return 2;
          case 0x02: case 0x03: return 3;
          default: break;
        }
      }
      return 1;   // stopped (slider minimum)
    }

    // Intermittent speed 0x0E22 uses the same order as the mode bytes 09/0A/0B (LOW/HIGH/MED):
    //   0x00 = LOW, 0x01 = HIGH, 0x02 = MED   (i.e. 0x0E22 == continuous mode byte - 0x09)
    // Evidence: 0E22=01 runs at the same ~85 CFM as Continuous HIGH, and the ERV reports
    // 0x0720=02 (HIGH) while in INT with 0E22=01.
    auto it = this->regs_.find(0x0E22);
    if (it != this->regs_.end() && !it->second.value.empty()) {
      uint8_t spd = it->second.value[0];
      if (spd == 0x00) return 1; // Low
      if (spd == 0x02) return 2; // Med
      if (spd == 0x01) return 3; // High
    }
    return 1;
  }

  uint32_t get_intermittent_seconds() const { return this->get_u32_reg_(0x0222, 1800); }
  // Humidity control, as the VTTOUCHW writes it (captured 2026-11-02 10:48 and 11:31):
  //   0x0A22 and 0x0C22 = setpoint, float %RH (the screen always writes both, same value)
  //   0x0F22 = 01 when "DEHUM." is selected
  //   0x1022 = 01 when "HUMIDISTAT" is selected
  //   both 00 = humidity control off
  // Observed ERV modes: 0x0D = dehumidifying (fan forced to HIGH while RH > setpoint),
  // 0x0E = humidistat active (fans stopped while RH > setpoint). 0x0950 (int 40) is unrelated.
  enum HumidityMode : uint8_t { HUMIDITY_OFF = 0, HUMIDITY_DEHUM = 1, HUMIDITY_HUMIDISTAT = 2 };

  float get_humidity_setpoint() const { return this->get_f32_reg_(0x0A22, 40.0f); }
  uint8_t get_humidity_mode() const {
    if (this->get_u8_reg_(0x0F22)) return HUMIDITY_DEHUM;
    if (this->get_u8_reg_(0x1022)) return HUMIDITY_HUMIDISTAT;
    return HUMIDITY_OFF;
  }

  void write_fan_speed_from_ha(uint8_t level) {
    if (level < 1) level = 1;
    if (level > 3) level = 3;
    uint8_t m = this->get_current_mode_byte();

    // If currently in a Continuous mode, switch continuous speed directly
    if (m == 0x09 || m == 0x0B || m == 0x0A) {
      uint8_t new_mode = (level == 1) ? 0x09 : ((level == 2) ? 0x0B : 0x0A);
      ESP_LOGI(TAG, "[HA -> ERV] Changing Continuous Fan Speed to level %u (mode 0x%02X)", level, new_mode);
      this->enqueue_bus_a_write_(0x0020, {new_mode});
      this->enqueue_bus_a_write_(0x0820, {0x00});
      return;
    }

    // Only Intermittent has a stored speed to change. In SMART / Away / Turbo / OFF the ERV decides the
    // speed; writing 0x0E22 there would silently change the Intermittent preset instead.
    if (m != 0x08) {
      ESP_LOGW(TAG, "[HA -> ERV] Fan speed is set by the ERV in mode 0x%02X; ignoring level %u", m, level);
      return;
    }

    // Map level (1=Low, 2=Med, 3=High) to 0x0E22 (0x00=Low, 0x02=Med, 0x01=High), see get_fan_speed_level()
    uint8_t e22_val = (level == 1) ? 0x00 : ((level == 2) ? 0x02 : 0x01);
    ESP_LOGI(TAG, "[HA -> ERV] Setting Intermittent Fan Speed 0x0E22 = 0x%02X (level %u)", e22_val, level);
    this->enqueue_bus_a_write_(0x0322, {0x00});
    this->enqueue_bus_a_write_(0x0E22, {e22_val});
    this->enqueue_bus_a_write_(0x0222, u32_le_(this->get_intermittent_seconds()));
    this->enqueue_bus_a_write_(0x0020, {0x08});
    this->enqueue_bus_a_write_(0x0820, {0x00});
  }

  void write_mode_from_ha(uint8_t mode_byte) {
    if (mode_byte == 0x0C) {
      uint32_t sec = this->get_turbo_timeout_hours() * 3600;
      const auto dur_bytes = u32_le_(sec);
      ESP_LOGI(TAG, "[HA -> ERV] Setting Turbo Mode (0x0C) with duration 0x0022 = %u s", (unsigned) sec);
      this->enqueue_bus_a_write_(0x0020, {0x0C});
      this->enqueue_bus_a_write_(0x0022, dur_bytes);
      return;
    }

    if (mode_byte == 0x08) {
      uint32_t sec = this->get_intermittent_seconds();
      const auto t_bytes = u32_le_(sec);
      // Same final frame the VTTOUCHW sends when you pick Intermittent: 0222, 0322, 0020, 0820.
      // (The screen sets the speed in a separate, earlier 0322+0E22 write.) 0x0E22 is left
      // untouched here, so the stored intermittent speed is kept; HA changes it via Fan Speed.
      ESP_LOGI(TAG, "[HA -> ERV] Setting Intermittent Mode (0x08) with time=%us", (unsigned) sec);
      this->enqueue_bus_a_write_(0x0222, t_bytes);
      this->enqueue_bus_a_write_(0x0322, {0x00});
      this->enqueue_bus_a_write_(0x0020, {0x08});
      this->enqueue_bus_a_write_(0x0820, {0x00});
      return;
    }

    ESP_LOGI(TAG, "[HA -> ERV] Setting Operating Mode 0x0020 = 0x%02X", mode_byte);
    this->enqueue_bus_a_write_(0x0020, {mode_byte});
    this->enqueue_bus_a_write_(0x0820, {0x00});
  }

  void write_intermittent_minutes_from_ha(uint32_t minutes_per_hour) {
    uint32_t sec = minutes_per_hour * 60;
    if (this->get_intermittent_seconds() == sec) return;
    const auto bytes = u32_le_(sec);
    ESP_LOGI(TAG, "[HA -> ERV] Setting intermittent time 0x0222 = %u s (%u min/hr)", (unsigned) sec, (unsigned) minutes_per_hour);
    this->enqueue_bus_a_write_(0x0222, bytes);
    this->enqueue_bus_a_write_(0x0820, {0x00});
    this->enqueue_bus_a_write_(0x0322, {0x00});
    if (this->get_current_mode_byte() == 0x08) {
      this->enqueue_bus_a_write_(0x0020, {0x08});
    }
  }

  void write_humidity_setpoint_from_ha(float rh_percent) {
    if (std::fabs(this->get_humidity_setpoint() - rh_percent) < 0.5f) return;
    const auto bytes = f32_le_(rh_percent);
    ESP_LOGI(TAG, "[HA -> ERV] Setting humidity setpoint 0x0A22/0x0C22 = %.0f%%", rh_percent);
    this->enqueue_bus_a_write_(0x0A22, bytes);
    this->enqueue_bus_a_write_(0x0C22, bytes);
  }

  void write_humidity_mode_from_ha(uint8_t mode) {
    // Same four writes the screen sends when you save its Humidity Control page.
    static const char *const NAMES[] = {"OFF", "DEHUM", "HUMIDISTAT"};
    if (mode > HUMIDITY_HUMIDISTAT) mode = HUMIDITY_OFF;
    const auto sp = f32_le_(this->get_humidity_setpoint());
    ESP_LOGI(TAG, "[HA -> ERV] Humidity control %s", NAMES[mode]);
    this->enqueue_bus_a_write_(0x0A22, sp);
    this->enqueue_bus_a_write_(0x0C22, sp);
    this->enqueue_bus_a_write_(0x0F22, {static_cast<uint8_t>(mode == HUMIDITY_DEHUM ? 0x01 : 0x00)});
    this->enqueue_bus_a_write_(0x1022, {static_cast<uint8_t>(mode == HUMIDITY_HUMIDISTAT ? 0x01 : 0x00)});
  }

  uint32_t get_turbo_timeout_hours() const {
    uint32_t sec = this->get_u32_reg_(0x0022);
    if (sec >= 3600) return std::min<uint32_t>((sec + 1800) / 3600, 4);
    return this->turbo_hours_pref_;
  }

  void write_turbo_timeout_hours_from_ha(uint32_t hours) {
    this->turbo_hours_pref_ = hours;
    uint32_t sec = hours * 3600;
    const auto bytes = u32_le_(sec);
    ESP_LOGI(TAG, "[HA -> ERV] Setting Turbo/Override duration 0x0022 = %u s (%u h)", (unsigned) sec, (unsigned) hours);
    this->enqueue_bus_a_write_(0x0022, bytes);
    if (this->get_current_mode_byte() == 0x0C) {
      this->enqueue_bus_a_write_(0x0020, {0x0C});
    }
  }

  // Same sequence the screen sends for a filter reset (captured 2026-11-02 10:50):
  // 0x0930 = filter interval (s), then 0x0130 = 1 (reset trigger), then 0x0830 = interval (time remaining).
  void reset_filter_timer_from_ha() {
    uint32_t interval = this->get_u32_reg_(0x0930, 7776000);
    const auto bytes = u32_le_(interval);
    ESP_LOGI(TAG, "[HA -> ERV] Resetting filter timer to %u s (%.0f days)", (unsigned) interval, interval / 86400.0f);
    this->enqueue_bus_a_write_(0x0930, bytes);
    this->enqueue_bus_a_write_(0x0130, {0x01});
    this->enqueue_bus_a_write_(0x0830, bytes);
  }

  // Register dump, printed a few lines per loop so it never stalls the buses (a one-shot dump of
  // ~70 registers blocked the loop for 100+ ms, close to the screen's ~450 ms reply timeout).
  void dump_status() {
    this->print_summary_();
    this->print_boot_report_();
    ESP_LOGI(TAG, "================ PROXY REGISTER DUMP (%u regs) ================", (unsigned) this->regs_.size());
    this->dump_active_ = true;
    this->dump_next_key_ = 0;
  }

  // ---- Relay link control (D-229 on G7). Called from YAML around every relay switch. ----
  //
  // Relays OFF = the ESP is physically off both buses (screen wired straight to the ERV).
  // While the link is down the proxy sends nothing, refuses HA writes and only discards RX noise.

  // Call right BEFORE the relays drop. Stops all transmitting, then waits (max max_ms) until both
  // lines have been quiet for 12 ms, so the contacts open between frames instead of mid-frame.
  // Our own frames are never cut: write_array() + flush() return only once the last byte is out.
  void quiesce_for_disconnect(uint32_t max_ms = 150) {
    this->link_active_ = false;
    this->pending_tx_a_.clear();
    this->pending_tx_b_.clear();
    const uint32_t start = millis();
    uint32_t last_byte = start;
    uint8_t b;
    while (millis() - start < max_ms) {
      bool got = false;
      while (this->uart_a_ != nullptr && this->uart_a_->available() && this->uart_a_->read_byte(&b)) got = true;
      while (this->uart_b_ != nullptr && this->uart_b_->available() && this->uart_b_->read_byte(&b)) got = true;
      const uint32_t t = millis();
      if (got) last_byte = t;
      else if (t - last_byte >= 12) break;   // both buses idle between frames
      delay(1);
    }
    ESP_LOGI(TAG, "[Link] Buses quiet after %u ms -> relays can drop", (unsigned) (millis() - start));
  }

  // Call right AFTER the relays switch (true = ESP back in the path). Going up resets both buses to
  // their start-up state: the screen is not served until Bus A has re-read the ERV, so it never
  // sees values from before the bypass (the screen may have changed modes while it was direct).
  void set_link_active(bool on) {
    const uint32_t now = millis();
    this->link_active_ = on;
    this->pending_tx_a_.clear();
    this->pending_tx_b_.clear();
    this->rx_buf_a_.clear();
    this->rx_buf_b_.clear();
    this->urgent_poll_keys_.clear();
    this->refresh_keys_.clear();
    this->dynamic_extra_keys_.clear();
    this->bus_b_held_read_keys_.clear();
    // Writes queued before the bypass are dropped: they would land on top of whatever the screen
    // set while it was in control.
    this->bus_a_write_queue_.clear();
    this->write_guard_.clear();
    if (!on) {
      ESP_LOGI(TAG, "[Link] DOWN: proxy off the buses (screen <-> ERV direct)");
      return;
    }
    this->bus_a_state_ = BusAState::LISTEN_FOR_ERV;
    this->bus_a_last_ms_ = now;
    this->bus_a_initial_sweep_done_ = false;
    this->current_batch_idx_ = 0;
    this->resync_pending_ = this->boot_.sweep != 0;   // not the first link-up since boot
    this->bus_b_state_ = BusBState::WAIT_BUS_A_INIT;
    this->bus_b_last_ms_ = now;
    this->bus_b_missed_offers_ = 0;
    this->bus_b_ping_candidate_ = 0x12;
    this->last_rx_a_ms_ = now;
    this->last_rx_b_ms_ = now;
    this->link_up_ms_ = now;
    this->link_up_cycles_ = this->bus_a_cycles_;
    ESP_LOGI(TAG, "[Link] UP: re-reading the ERV before serving the screen");
  }

  bool is_link_active() const { return this->link_active_; }

  void loop() override {
    const uint32_t now = millis();

    if (!this->link_active_) {
      // Off the buses: just discard anything the UARTs pick up (relay bounce, noise).
      uint8_t b;
      while (this->uart_a_ != nullptr && this->uart_a_->available()) this->uart_a_->read_byte(&b);
      while (this->uart_b_ != nullptr && this->uart_b_->available()) this->uart_b_->read_byte(&b);
      this->rx_buf_a_.clear();
      this->rx_buf_b_.clear();
      if (now - this->last_summary_ms_ >= 15000) {
        this->last_summary_ms_ = now;
        ESP_LOGI(TAG, "PROXY | Link DOWN (bypass): screen <-> ERV direct, showing last known values");
      }
      if (this->dump_active_) this->dump_some_registers_();
      return;
    }

    // ---- BUS A (Real ERV) RX + TX + State Machine ----
    this->drain_uart_(this->uart_a_, this->rx_buf_a_, this->last_rx_a_ms_, this->bus_a_rx_bytes_, now);
    if (!this->rx_buf_a_.empty() && (now - this->last_rx_a_ms_ > 60)) {
      this->rx_buf_a_.clear();
    }
    this->process_rx_a_(now);

    // Signed difference: still correct when millis() wraps after 49.7 days.
    if (!this->pending_tx_a_.empty() && (int32_t) (now - this->tx_ready_a_ms_) >= 0) {
      this->uart_a_->write_array(this->pending_tx_a_);
      this->uart_a_->flush();
      uint32_t done_ms = millis();
      if (this->recent_tx_a_.size() >= 4) this->recent_tx_a_.pop_front();
      this->recent_tx_a_.push_back(SentFrameRecord{this->pending_tx_a_, done_ms});
      this->pending_tx_a_.clear();
      this->drain_uart_(this->uart_a_, this->rx_buf_a_, this->last_rx_a_ms_, this->bus_a_rx_bytes_, done_ms);
    }
    this->run_bus_a_state_machine_(now);

    // ---- BUS B (VTTOUCHW Wall Screen) RX + TX + State Machine ----
    this->drain_uart_(this->uart_b_, this->rx_buf_b_, this->last_rx_b_ms_, this->bus_b_rx_bytes_, now);
    if (!this->rx_buf_b_.empty() && (now - this->last_rx_b_ms_ > 60)) {
      // Right after the relays close, a fragment of a frame already on the wire is expected.
      if (now - this->link_up_ms_ < 200) {
        ESP_LOGD(TAG, "[Bus B] Dropping %u bytes caught as the relays closed", (unsigned) this->rx_buf_b_.size());
      } else {
        ESP_LOGW(TAG, "[Bus B] Dropping %u unparsed RX bytes: [%s]",
                 (unsigned) this->rx_buf_b_.size(),
                 format_hex_(this->rx_buf_b_.data(), std::min<size_t>(this->rx_buf_b_.size(), 16)).c_str());
      }
      this->rx_buf_b_.clear();
    }
    this->process_rx_b_(now);

    // Listen before talk: never start a frame while the VTTOUCHW is still sending one
    // (that collision is what corrupts its frames and makes it fall back to Ping re-discovery).
    bool bus_b_busy = this->uart_b_->available() > 0 || !this->rx_buf_b_.empty() ||
                      (millis() - this->last_rx_b_ms_ < 3);
    const int32_t b_late_ms = (int32_t) (now - this->tx_ready_b_ms_);   // wrap-safe
    if (!this->pending_tx_b_.empty() && b_late_ms >= 0 && (!bus_b_busy || b_late_ms > 100)) {
      if (bus_b_busy) ESP_LOGW(TAG, "[Bus B] Line still busy after 100 ms, sending anyway");
      this->uart_b_->write_array(this->pending_tx_b_);
      this->uart_b_->flush();
      uint32_t done_ms = millis();
      if (this->recent_tx_b_.size() >= 4) this->recent_tx_b_.pop_front();
      this->recent_tx_b_.push_back(SentFrameRecord{this->pending_tx_b_, done_ms});
      this->pending_tx_b_.clear();
      this->drain_uart_(this->uart_b_, this->rx_buf_b_, this->last_rx_b_ms_, this->bus_b_rx_bytes_, done_ms);
    }
    this->run_bus_b_state_machine_(now);

    // ---- Periodic HA Sensor Publish (every 10s) ----
    if (now - this->last_sensor_pub_ms_ >= 10000) {
      this->last_sensor_pub_ms_ = now;
      this->publish_sensors_();
    }

    // ---- Periodic 15s Status Line ----
    if (now - this->last_summary_ms_ >= 15000) {
      this->last_summary_ms_ = now;
      this->print_summary_();
    }

    if (this->dump_active_) this->dump_some_registers_();
  }

 protected:
  HighFrequencyLoopRequester high_freq_;
  uart::UARTComponent *uart_a_{nullptr};
  uart::UARTComponent *uart_b_{nullptr};

  sensor::Sensor *wall_temp_sensor_{nullptr};
  sensor::Sensor *wall_humidity_sensor_{nullptr};
  sensor::Sensor *erv_temp_sensor_{nullptr};
  sensor::Sensor *supply_cfm_sensor_{nullptr};
  sensor::Sensor *exhaust_cfm_sensor_{nullptr};
  sensor::Sensor *filter_days_sensor_{nullptr};

  std::map<uint16_t, RegisterRecord> regs_;
  std::deque<PendingWrite> bus_a_write_queue_;
  // While a write to `key` is in progress, ERV read replies that still carry the old value are
  // ignored. The ERV acknowledges a write immediately but applies it 100-450 ms later (measured on
  // the user's unit), so the guard lasts until the ERV reports the written value, with a deadline:
  // 3 s while queued, then 1.5 s after the ERV's ack. After the deadline the ERV's value wins.
  struct WriteGuard { uint32_t deadline; std::vector<uint8_t> expect; };
  std::map<uint16_t, WriteGuard> write_guard_;
  std::set<uint16_t> urgent_poll_keys_;
  std::set<uint16_t> refresh_keys_;   // re-read right after an acknowledged write
  std::set<uint16_t> dynamic_extra_keys_;
  uint32_t dynamic_extra_expire_ms_{0};
  std::vector<std::vector<uint16_t>> poll_batches_;
  size_t current_batch_idx_{0};
  uint32_t turbo_hours_pref_{2};

  enum class BusAState {
    LISTEN_FOR_ERV,
    HAVE_TOKEN_SEND_HB,
    WAIT_HB_ACK,
    HAVE_TOKEN_SEND_WRITE,
    WAIT_WRITE_ACK,
    HAVE_TOKEN_SEND_READ,
    WAIT_READ_ACK,
    HAVE_TOKEN_YIELD,
  };

  BusAState bus_a_state_{BusAState::LISTEN_FOR_ERV};
  uint8_t bus_a_addr_{0x12};
  uint32_t bus_a_last_ms_{0};
  uint32_t bus_a_last_hb_ms_{0};
  uint32_t bus_a_cycles_{0};
  uint32_t bus_a_chk_err_{0};
  uint32_t bus_a_rx_bytes_{0};
  bool bus_a_initial_sweep_done_{false};
  std::map<uint16_t, uint32_t> watch_last_ms_;   // change watcher: last log time per register
  uint32_t watch_suppressed_{0};                  // changes not logged (rate limit) since the last status line
  bool link_active_{false};
  uint32_t link_up_ms_{0};
  uint32_t link_up_cycles_{0};   // Bus A cycle count at link up       // when the relays last connected (fragment caught then is expected)      // off until YAML switches the relays on (set_link_active(true))
  bool resync_pending_{false};   // next sweep follows a bypass: re-read non-polled registers too
  std::vector<uint8_t> rx_buf_a_;
  std::vector<uint8_t> pending_tx_a_;
  std::deque<SentFrameRecord> recent_tx_a_;
  uint32_t last_rx_a_ms_{0};
  uint32_t tx_ready_a_ms_{0};

  enum class BusBState {
    WAIT_BUS_A_INIT,
    DISCOVERY_PING,
    WAIT_PING_REPLY,
    OFFER_TOKEN,
    WAIT_TOKEN_ACCEPT,
    CONTROLLER_HAS_TOKEN,
    HOLD_FOR_CACHE_MISS,
  };

  BusBState bus_b_state_{BusBState::WAIT_BUS_A_INIT};
  uint8_t bus_b_ctrl_addr_{0x12};
  uint8_t bus_b_ping_candidate_{0x12};
  uint8_t bus_b_missed_offers_{0};
  uint32_t bus_b_last_ms_{0};
  uint32_t bus_b_cycles_{0};
  uint32_t bus_b_chk_err_{0};
  uint32_t bus_b_rx_bytes_{0};
  uint32_t bus_b_echoes_{0};
  uint32_t bus_b_cache_misses_{0};
  uint32_t bus_b_offer_ms_{0};
  uint32_t bus_b_accept_max_ms_{0};   // slowest token accept since the last status line
  uint32_t bus_b_accept_late_{0};     // accepts slower than 50 ms since the last status line
  uint32_t bus_b_holds_{0};           // cache-miss holds since the last status line
  uint32_t bus_b_hold_max_ms_{0};     // longest of them
  uint32_t bus_b_hold_timeouts_{0};   // holds that ended without the ERV's answer

  // Boot milestones. Logged on every status line for the first 3 minutes (and in the register dump),
  // because the log viewer usually connects a few seconds after boot and misses the live lines.
  struct BootReport {
    uint32_t t0{0}, erv_token{0}, sweep{0}, screen_frame{0}, screen_accept{0}, startup_end{0};
    uint32_t refreshed{0}, offers{0}, pings{0};
    bool startup_mode{false};
    std::vector<uint16_t> differs;
  } boot_;
  std::vector<uint16_t> bus_b_held_read_keys_;
  uint8_t bus_b_held_sender_{0x12};
  uint32_t bus_b_hold_start_ms_{0};
  std::vector<uint8_t> rx_buf_b_;
  std::vector<uint8_t> pending_tx_b_;
  std::deque<SentFrameRecord> recent_tx_b_;
  uint32_t last_rx_b_ms_{0};
  uint32_t tx_ready_b_ms_{0};

  uint32_t last_summary_ms_{0};
  bool dump_active_{false};
  uint16_t dump_next_key_{0};
  uint32_t last_sensor_pub_ms_{0};

  static void drain_uart_(uart::UARTComponent *u, std::vector<uint8_t> &buf, uint32_t &last_ms, uint32_t &byte_counter, uint32_t now) {
    if (u == nullptr) return;
    while (u->available()) {
      uint8_t b;
      if (u->read_byte(&b)) {
        buf.push_back(b);
        last_ms = now;
        byte_counter++;
      }
    }
  }

  static std::string format_hex_(const uint8_t *data, size_t len) {
    char buf[16];
    std::string out;
    for (size_t i = 0; i < len; i++) {
      snprintf(buf, sizeof(buf), "%s%02X", (i == 0 ? "" : " "), data[i]);
      out += buf;
    }
    return out;
  }

  // 32-bit little-endian value as the 4 bytes the ERV uses for integer registers.
  static std::vector<uint8_t> u32_le_(uint32_t v) {
    return {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)};
  }
  // 32-bit little-endian IEEE float, as used for setpoints and sensor readings.
  static std::vector<uint8_t> f32_le_(float f) {
    uint32_t v;
    std::memcpy(&v, &f, 4);
    return u32_le_(v);
  }

  // Registers the ERV changes by itself right after a write: a countdown timer and one-shot
  // triggers. Holding back older values for them would only produce a false "ERV kept" warning.
  static bool no_write_guard_(uint16_t key) { return key == 0x0830 || key == 0x0130 || key == 0x1600; }

  static uint8_t calc_checksum_(const uint8_t *data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return static_cast<uint8_t>(1 - sum);
  }

  static bool match_and_consume_echo_(std::deque<SentFrameRecord> &recent, const uint8_t *frame_ptr, size_t total_len, uint32_t now) {
    while (!recent.empty() && (now - recent.front().sent_ms) > 50) {
      recent.pop_front();
    }
    for (auto it = recent.begin(); it != recent.end(); ++it) {
      if (it->bytes.size() == total_len && std::equal(it->bytes.begin(), it->bytes.end(), frame_ptr)) {
        recent.erase(it);
        return true;
      }
    }
    return false;
  }

  // The length field is one byte: a payload over this would wrap it and put a malformed frame on the bus.
  static constexpr size_t MAX_PAYLOAD = 250;

  void queue_frame_a_(uint8_t target, const std::vector<uint8_t> &payload, uint32_t delay_ms = 3) {
    if (payload.size() > MAX_PAYLOAD) {
      ESP_LOGW(TAG, "[Bus A] Payload of %u bytes to 0x%02X is too long, frame dropped",
               (unsigned) payload.size(), target);
      return;
    }
    std::vector<uint8_t> frame;
    frame.reserve(payload.size() + 7);
    frame.push_back(0x01);
    frame.push_back(target);
    frame.push_back(this->bus_a_addr_);
    frame.push_back(0x01);
    frame.push_back(static_cast<uint8_t>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(calc_checksum_(frame.data(), frame.size()));
    frame.push_back(0x04);
    this->pending_tx_a_ = std::move(frame);
    this->tx_ready_a_ms_ = millis() + delay_ms;
  }

  void queue_frame_b_(uint8_t target, const std::vector<uint8_t> &payload, uint32_t delay_ms = 3) {
    if (payload.size() > MAX_PAYLOAD) {
      ESP_LOGW(TAG, "[Bus B] Payload of %u bytes to 0x%02X is too long, frame dropped",
               (unsigned) payload.size(), target);
      return;
    }
    std::vector<uint8_t> frame;
    frame.reserve(payload.size() + 7);
    frame.push_back(0x01);
    frame.push_back(target);
    frame.push_back(0x10);
    frame.push_back(0x01);
    frame.push_back(static_cast<uint8_t>(payload.size()));
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(calc_checksum_(frame.data(), frame.size()));
    frame.push_back(0x04);
    this->pending_tx_b_ = std::move(frame);
    this->tx_ready_b_ms_ = millis() + delay_ms;
  }

  void enqueue_bus_a_write_(uint16_t key, const std::vector<uint8_t> &val) {
    if (key == 0x0050) return;
    if (!this->link_active_) {
      ESP_LOGW(TAG, "[HA -> ERV] Write to 0x%04X ignored: proxy is bypassed (screen in control)", key);
      return;
    }
    if (!val.empty()) {
      // Show the new value right away and ignore older ERV values until it applies the write
      // (safety expiry 3 s if the write is ever lost).
      // 0x0020 and 0x0120 both report the mode byte. 0x0220 is NOT predicted: in turbo the ERV keeps
      // the previous mode there (the mode it returns to). 0x0720 (fan state) isn't predicted either;
      // both are re-read after the ERV's ack.
      const uint32_t dl = millis() + 3000;
      auto &rec = this->regs_[key];
      rec.key = key;
      rec.fetched_from_erv = true;
      if (!no_write_guard_(key)) this->write_guard_[key] = WriteGuard{dl, val};
      rec.value = val;
      if (key == 0x0020) {
        this->regs_[0x0120].value = val;
        this->write_guard_[0x0120] = WriteGuard{dl, val};
      }
      // Coalesce: a newer value replaces one still waiting in the queue.
      for (auto &w : this->bus_a_write_queue_) {
        if (w.key == key && !w.value.empty()) {
          w.value = val;
          return;
        }
      }
    }
    this->bus_a_write_queue_.push_back(PendingWrite{key, val});
  }

  // ---- BUS A PARSER & STATE MACHINE ----
  void process_rx_a_(uint32_t now) {
    while (this->rx_buf_a_.size() >= 7) {
      if (this->rx_buf_a_[0] != 0x01) {
        this->rx_buf_a_.erase(this->rx_buf_a_.begin());
        continue;
      }
      uint8_t target = this->rx_buf_a_[1];
      uint8_t sender = this->rx_buf_a_[2];
      uint8_t proto  = this->rx_buf_a_[3];
      uint8_t plen   = this->rx_buf_a_[4];

      if (proto != 0x01 || plen == 0 || target > 0x1F || sender > 0x1F) {
        this->rx_buf_a_.erase(this->rx_buf_a_.begin());
        continue;
      }
      size_t total_len = 5 + size_t(plen) + 2;
      if (this->rx_buf_a_.size() < total_len) return;

      uint8_t expected_chk = calc_checksum_(this->rx_buf_a_.data(), total_len - 2);
      uint8_t actual_chk   = this->rx_buf_a_[total_len - 2];
      uint8_t eot          = this->rx_buf_a_[total_len - 1];

      if (eot != 0x04) {
        this->rx_buf_a_.erase(this->rx_buf_a_.begin());
        continue;
      }
      if (expected_chk != actual_chk) {
        this->bus_a_chk_err_++;
        this->rx_buf_a_.erase(this->rx_buf_a_.begin());
        continue;
      }
      if (match_and_consume_echo_(this->recent_tx_a_, this->rx_buf_a_.data(), total_len, now)) {
        this->rx_buf_a_.erase(this->rx_buf_a_.begin(), this->rx_buf_a_.begin() + total_len);
        continue;
      }

      std::vector<uint8_t> payload(this->rx_buf_a_.begin() + 5, this->rx_buf_a_.begin() + 5 + plen);
      this->rx_buf_a_.erase(this->rx_buf_a_.begin(), this->rx_buf_a_.begin() + total_len);

      if (sender == 0x10 && target == this->bus_a_addr_) {
        this->handle_frame_from_erv_(payload, now);
      }
    }
  }

  void handle_frame_from_erv_(const std::vector<uint8_t> &payload, uint32_t now) {
    uint8_t msg_type = payload[0];

    switch (msg_type) {
      case 0x02: {
        ESP_LOGI(TAG, "[Bus A] ERV pinged 0x%02X -> replying 03 'Ping'", this->bus_a_addr_);
        this->queue_frame_a_(0x10, {0x03, 'P', 'i', 'n', 'g'}, 3);
        this->bus_a_state_ = BusAState::LISTEN_FOR_ERV;
        this->bus_a_last_ms_ = now;
        break;
      }

      case 0x04: {
        if (!this->boot_.erv_token) this->boot_.erv_token = now;
        this->queue_frame_a_(0x10, {0x05}, 3);
        this->bus_a_cycles_++;
        this->bus_a_last_ms_ = now;

        if (now - this->bus_a_last_hb_ms_ >= 10000) {
          this->bus_a_state_ = BusAState::HAVE_TOKEN_SEND_HB;
        } else if (!this->bus_a_write_queue_.empty()) {
          this->bus_a_state_ = BusAState::HAVE_TOKEN_SEND_WRITE;
        } else {
          this->bus_a_state_ = BusAState::HAVE_TOKEN_SEND_READ;
        }
        break;
      }

      case 0x41: {
        this->bus_a_last_ms_ = now;
        if (this->bus_a_state_ == BusAState::WAIT_HB_ACK) {
          this->bus_a_last_hb_ms_ = now;
          if (!this->bus_a_write_queue_.empty()) {
            this->bus_a_state_ = BusAState::HAVE_TOKEN_SEND_WRITE;
          } else {
            this->bus_a_state_ = BusAState::HAVE_TOKEN_SEND_READ;
          }
        } else if (this->bus_a_state_ == BusAState::WAIT_WRITE_ACK) {
          // The ERV accepted these keys. It applies them a little later, so keep the guard until it
          // reports the new value (max 1.5 s from now) and re-read them on the next READ.
          for (size_t i = 1; i + 1 < payload.size(); i += 2) {
            uint16_t k = (uint16_t(payload[i]) << 8) | payload[i + 1];
            bool still_queued = false;
            for (const auto &w : this->bus_a_write_queue_) if (w.key == k) still_queued = true;
            if (still_queued) continue;   // a newer value for this key is still waiting to be sent
            std::vector<uint16_t> keys{k};
            if (k == 0x0020) keys = {0x0020, 0x0120, 0x0220, 0x0720};
            for (uint16_t gk : keys) {
              auto g = this->write_guard_.find(gk);
              if (g != this->write_guard_.end()) g->second.deadline = now + 1500;
              this->refresh_keys_.insert(gk);
            }
          }
          if (!this->bus_a_write_queue_.empty()) {
            this->bus_a_state_ = BusAState::HAVE_TOKEN_SEND_WRITE;
          } else {
            this->bus_a_state_ = BusAState::HAVE_TOKEN_SEND_READ;
          }
        }
        break;
      }

      case 0x21: {
        this->bus_a_last_ms_ = now;
        size_t idx = 1;
        while (idx + 2 < payload.size()) {
          uint16_t k = (uint16_t(payload[idx]) << 8) | payload[idx + 1];
          uint8_t vlen = payload[idx + 2];
          idx += 3;
          if (idx + vlen > payload.size()) break;

          auto g = this->write_guard_.find(k);
          if (g != this->write_guard_.end()) {
            bool matches = g->second.expect.size() == vlen &&
                           std::equal(g->second.expect.begin(), g->second.expect.end(), payload.begin() + idx);
            if (matches) {
              this->write_guard_.erase(g);                 // ERV now reports what we wrote
            } else if ((int32_t) (now - g->second.deadline) < 0) {
              idx += vlen;                                 // old value, ERV hasn't applied the write yet
              continue;
            } else {
              ESP_LOGW(TAG, "[Bus A] ERV kept 0x%04X = %s instead of the value written; using the ERV's value", k,
                       format_hex_(payload.data() + idx, std::min<size_t>(vlen, 8)).c_str());
              this->write_guard_.erase(g);
            }
          }
          auto &rec = this->regs_[k];
          rec.key = k;
          if ((k == 0x0020 || k == 0x0620 || k == 0x0720 || k == 0x0E22 || k == 0x0322 || k == 0x0F22 ||
               k == 0x1022) && vlen == 1 &&
              (rec.value.size() != 1 || rec.value[0] != payload[idx])) {
            ESP_LOGI(TAG, "[ERV state] 0x%04X: %s -> %02X  (supply %.1f CFM)", k,
                     rec.value.size() == 1 ? format_hex_(rec.value.data(), 1).c_str() : "--",
                     payload[idx], this->get_f32_reg_(0x0510));
          }
          // Only registers outside the poll batches are worth checking: the polled ones carry live values
          // (temperatures, airflow, mode, timers) that are expected to differ from any snapshot.
          if (rec.seeded && !rec.fetched_from_erv && !rec.value.empty() && !this->in_poll_batches_(k) &&
              (rec.value.size() != vlen || !std::equal(rec.value.begin(), rec.value.end(), payload.begin() + idx))) {
            ESP_LOGI(TAG, "[Bus A] 0x%04X: ERV value differs from the built-in starting value (now using the ERV's)", k);
            if (this->boot_.differs.size() < 24) this->boot_.differs.push_back(k);
          }
          // Change watcher: any other polled register that changes is logged (max once a minute per
          // register). Meant to catch status flags we haven't mapped yet, e.g. defrost on a cold night.
          if (this->bus_a_initial_sweep_done_ && rec.fetched_from_erv && !rec.value.empty() &&
              this->in_poll_batches_(k) && !watch_ignored_(k) &&
              (rec.value.size() != vlen || !std::equal(rec.value.begin(), rec.value.end(), payload.begin() + idx))) {
            uint32_t &last = this->watch_last_ms_[k];
            if (last == 0 || now - last >= 60000) {
              last = now;
              ESP_LOGI(TAG, "[Watch] 0x%04X: %s -> %s | intake %.1f C (raw) | supply %.1f / exhaust %.1f CFM", k,
                       format_hex_(rec.value.data(), std::min<size_t>(rec.value.size(), 8)).c_str(),
                       format_hex_(payload.data() + idx, std::min<size_t>(vlen, 8)).c_str(),
                       this->get_f32_reg_(0x01E0), this->get_f32_reg_(0x0510), this->get_f32_reg_(0x0610));
            } else {
              this->watch_suppressed_++;
            }
          }
          if (k == 0x0020 && vlen == 1) {
            if (payload[idx] == 0x14) this->boot_.startup_mode = true;
            else if (this->boot_.startup_mode && !this->boot_.startup_end) this->boot_.startup_end = now;
          }
          rec.value.assign(payload.begin() + idx, payload.begin() + idx + vlen);
          rec.fetched_from_erv = true;
          this->urgent_poll_keys_.erase(k);
          idx += vlen;
        }

        if (this->bus_b_state_ == BusBState::HOLD_FOR_CACHE_MISS && this->urgent_poll_keys_.empty()) {
          this->note_hold_(now, false);
          this->send_bus_b_read_response_(this->bus_b_held_sender_, this->bus_b_held_read_keys_);
          this->bus_b_state_ = BusBState::CONTROLLER_HAS_TOKEN;
          this->bus_b_last_ms_ = now;
        }

        this->bus_a_state_ = BusAState::HAVE_TOKEN_YIELD;
        break;
      }

      case 0x05: {
        this->bus_a_state_ = BusAState::LISTEN_FOR_ERV;
        this->bus_a_last_ms_ = now;
        break;
      }

      default:
        break;
    }
  }

  void run_bus_a_state_machine_(uint32_t now) {
    if (!this->pending_tx_a_.empty()) return;

    if (this->bus_a_state_ != BusAState::LISTEN_FOR_ERV && (now - this->bus_a_last_ms_ >= 150)) {
      this->bus_a_state_ = BusAState::HAVE_TOKEN_YIELD;
    }

    switch (this->bus_a_state_) {
      case BusAState::LISTEN_FOR_ERV:
        break;

      case BusAState::HAVE_TOKEN_SEND_HB: {
        this->queue_frame_a_(0x10, {0x40, 0x00, 0x50, 0x00}, 3);
        this->bus_a_state_ = BusAState::WAIT_HB_ACK;
        this->bus_a_last_ms_ = now;
        break;
      }

      case BusAState::HAVE_TOKEN_SEND_WRITE: {
        if (this->bus_a_write_queue_.empty()) {
          this->bus_a_state_ = BusAState::HAVE_TOKEN_SEND_READ;
          break;
        }
        std::vector<uint8_t> pl = {0x40};
        size_t count = 0;
        while (!this->bus_a_write_queue_.empty() && count < 4) {
          auto w = this->bus_a_write_queue_.front();
          if (pl.size() + 3 + w.value.size() > 200) break;
          this->bus_a_write_queue_.pop_front();
          pl.push_back((w.key >> 8) & 0xFF);
          pl.push_back(w.key & 0xFF);
          pl.push_back(static_cast<uint8_t>(w.value.size()));
          pl.insert(pl.end(), w.value.begin(), w.value.end());
          count++;
        }
        this->queue_frame_a_(0x10, pl, 3);
        this->bus_a_state_ = BusAState::WAIT_WRITE_ACK;
        this->bus_a_last_ms_ = now;
        break;
      }

      case BusAState::HAVE_TOKEN_SEND_READ: {
        std::vector<uint8_t> pl = {0x20};
        // Keys the screen asked for (info pages) are polled for 60 s after its last request, then dropped.
        if (!this->dynamic_extra_keys_.empty() && (int32_t) (now - this->dynamic_extra_expire_ms_) >= 0)
          this->dynamic_extra_keys_.clear();

        if (this->urgent_poll_keys_.empty() && !this->refresh_keys_.empty()) {
          size_t count = 0;
          for (auto it = this->refresh_keys_.begin(); it != this->refresh_keys_.end() && count < 16; count++) {
            pl.push_back((*it >> 8) & 0xFF);
            pl.push_back(*it & 0xFF);
            it = this->refresh_keys_.erase(it);
          }
        } else if (!this->urgent_poll_keys_.empty()) {
          size_t count = 0;
          for (uint16_t k : this->urgent_poll_keys_) {
            pl.push_back((k >> 8) & 0xFF);
            pl.push_back(k & 0xFF);
            if (++count >= 16) break;
          }
        } else if (!this->dynamic_extra_keys_.empty() &&
                   (this->bus_a_cycles_ % 2 == 1)) {
          size_t count = 0;
          for (uint16_t k : this->dynamic_extra_keys_) {
            pl.push_back((k >> 8) & 0xFF);
            pl.push_back(k & 0xFF);
            if (++count >= 18) break;
          }
        } else {
          const auto &batch = this->poll_batches_[this->current_batch_idx_];
          for (uint16_t k : batch) {
            pl.push_back((k >> 8) & 0xFF);
            pl.push_back(k & 0xFF);
          }
          this->current_batch_idx_ = (this->current_batch_idx_ + 1) % this->poll_batches_.size();
          if (this->current_batch_idx_ == 0 && !this->bus_a_initial_sweep_done_) {
            this->bus_a_initial_sweep_done_ = true;
            // Starting values for registers outside the poll batches (boot-only IDs, installer menu)
            // are read once from the ERV, so the cache holds this unit's live values from the start.
            // 0x1600 is the fault-log clear trigger: never touched by the proxy itself.
            // After a bypass, every register outside the poll batches is re-read too (installer and
            // humidity settings may have been changed on the screen while it talked to the ERV directly).
            const bool resync = this->resync_pending_;
            this->resync_pending_ = false;
            size_t n = 0;
            for (const auto &kv : this->regs_) {
              if ((resync ? !this->in_poll_batches_(kv.first) : !kv.second.fetched_from_erv) &&
                  kv.first != 0x1600 && kv.first != 0x0450 && kv.first != 0x0550) {
                this->refresh_keys_.insert(kv.first);
                n++;
              }
            }
            if (resync) {
              ESP_LOGI(TAG, "[Bus A] Re-sync after bypass: poll sweep done, re-reading %u more registers "
                            "before the screen is served.", (unsigned) n);
            } else {
              this->boot_.sweep = now;
              this->boot_.refreshed = n;
              ESP_LOGI(TAG, "[Bus A] Initial register sweep from real ERV complete! Enabling Bus B. "
                            "Reading %u more starting-value registers once.", (unsigned) n);
            }
          }
        }

        this->queue_frame_a_(0x10, pl, 3);
        this->bus_a_state_ = BusAState::WAIT_READ_ACK;
        this->bus_a_last_ms_ = now;
        break;
      }

      case BusAState::WAIT_HB_ACK:
      case BusAState::WAIT_WRITE_ACK:
      case BusAState::WAIT_READ_ACK:
        break;

      case BusAState::HAVE_TOKEN_YIELD: {
        this->queue_frame_a_(0x10, {0x04}, 3);
        this->bus_a_state_ = BusAState::LISTEN_FOR_ERV;
        this->bus_a_last_ms_ = now;
        break;
      }
    }
  }

  // ---- BUS B PARSER & STATE MACHINE (Impersonates ERV 0x10 to VTTOUCHW) ----
  void process_rx_b_(uint32_t now) {
    while (this->rx_buf_b_.size() >= 7) {
      if (this->rx_buf_b_[0] != 0x01) {
        this->rx_buf_b_.erase(this->rx_buf_b_.begin());
        continue;
      }
      uint8_t target = this->rx_buf_b_[1];
      uint8_t sender = this->rx_buf_b_[2];
      uint8_t proto  = this->rx_buf_b_[3];
      uint8_t plen   = this->rx_buf_b_[4];

      if (proto != 0x01 || plen == 0 || target > 0x1F || sender > 0x1F) {
        this->rx_buf_b_.erase(this->rx_buf_b_.begin());
        continue;
      }
      size_t total_len = 5 + size_t(plen) + 2;
      if (this->rx_buf_b_.size() < total_len) return;

      uint8_t expected_chk = calc_checksum_(this->rx_buf_b_.data(), total_len - 2);
      uint8_t actual_chk   = this->rx_buf_b_[total_len - 2];
      uint8_t eot          = this->rx_buf_b_[total_len - 1];

      if (eot != 0x04) {
        this->rx_buf_b_.erase(this->rx_buf_b_.begin());
        continue;
      }
      if (expected_chk != actual_chk) {
        this->bus_b_chk_err_++;
        this->rx_buf_b_.erase(this->rx_buf_b_.begin());
        continue;
      }
      if (match_and_consume_echo_(this->recent_tx_b_, this->rx_buf_b_.data(), total_len, now)) {
        this->rx_buf_b_.erase(this->rx_buf_b_.begin(), this->rx_buf_b_.begin() + total_len);
        this->bus_b_echoes_++;
        continue;
      }

      std::vector<uint8_t> payload(this->rx_buf_b_.begin() + 5, this->rx_buf_b_.begin() + 5 + plen);
      this->rx_buf_b_.erase(this->rx_buf_b_.begin(), this->rx_buf_b_.begin() + total_len);

      if (target == 0x10) {
        // Until the cache holds fresh ERV values (boot, or right after a bypass while the screen may
        // still be mid-turn from talking to the ERV directly) the screen gets no answers at all.
        // It times out and waits for our token offer, which comes as soon as the re-read is done.
        if (this->bus_b_state_ == BusBState::WAIT_BUS_A_INIT) continue;
        this->handle_frame_from_wall_(sender, payload, now);
      }
    }
  }

  void send_bus_b_read_response_(uint8_t target, std::vector<uint16_t> req_keys) {
    std::stable_sort(req_keys.begin(), req_keys.end(), [](uint16_t a, uint16_t b) {
      return (a & 0xFF) > (b & 0xFF);
    });

    std::vector<uint8_t> resp;
    resp.push_back(0x21);
    for (uint16_t k : req_keys) {
      auto it = this->regs_.find(k);
      if (it != this->regs_.end()) {
        resp.push_back((k >> 8) & 0xFF);
        resp.push_back(k & 0xFF);
        resp.push_back(static_cast<uint8_t>(it->second.value.size()));
        resp.insert(resp.end(), it->second.value.begin(), it->second.value.end());
      }
    }
    this->queue_frame_b_(target, resp, 3);
  }

  void handle_frame_from_wall_(uint8_t sender, const std::vector<uint8_t> &payload, uint32_t now) {
    uint8_t msg_type = payload[0];
    if (!this->boot_.screen_frame) this->boot_.screen_frame = now;
    if (msg_type == 0x03 && !this->boot_.screen_accept) this->boot_.pings++;

    switch (msg_type) {
      case 0x03: {
        this->bus_b_ctrl_addr_ = sender;
        this->bus_b_missed_offers_ = 0;
        ESP_LOGI(TAG, ">>> [Bus B] VTTOUCHW answered Ping at 0x%02X! Starting Bus B token loop.", sender);
        this->bus_b_state_ = BusBState::OFFER_TOKEN;
        this->bus_b_last_ms_ = now;
        break;
      }

      case 0x05: {
        if (this->bus_b_state_ == BusBState::WAIT_TOKEN_ACCEPT) {
          uint32_t lat = now - this->bus_b_offer_ms_;
          if (lat > this->bus_b_accept_max_ms_) this->bus_b_accept_max_ms_ = lat;
          if (lat > 50) this->bus_b_accept_late_++;
          if (!this->boot_.screen_accept) this->boot_.screen_accept = now;
          if (this->bus_b_cycles_ == 0) {
            ESP_LOGI(TAG, ">>> [Bus B] VTTOUCHW (0x%02X) accepted first token offer!", sender);
          }
          this->bus_b_missed_offers_ = 0;
          this->bus_b_state_ = BusBState::CONTROLLER_HAS_TOKEN;
          this->bus_b_last_ms_ = now;
        }
        break;
      }

      case 0x04: {
        this->queue_frame_b_(sender, {0x05}, 3);
        this->bus_b_cycles_++;
        this->bus_b_state_ = BusBState::OFFER_TOKEN;
        this->bus_b_last_ms_ = now;
        break;
      }

      case 0x20: {
        // The screen is using its turn even if we never saw its 0x05 (lost or late): don't re-offer.
        if (this->bus_b_state_ == BusBState::WAIT_TOKEN_ACCEPT) this->bus_b_state_ = BusBState::CONTROLLER_HAS_TOKEN;
        this->bus_b_last_ms_ = now;
        std::vector<uint16_t> req_keys;
        bool any_missing = false;

        for (size_t i = 1; i + 1 < payload.size(); i += 2) {
          uint16_t k = (uint16_t(payload[i]) << 8) | payload[i + 1];
          req_keys.push_back(k);

          if (!this->is_in_core_batches_(k)) {
            this->dynamic_extra_keys_.insert(k);
            this->dynamic_extra_expire_ms_ = now + 60000;
          }

          auto it = this->regs_.find(k);
          if (it == this->regs_.end()) {
            RegisterRecord rec;
            rec.key = k;
            rec.value.assign(guess_len_(k), 0x00);
            rec.fetched_from_erv = false;
            this->regs_[k] = std::move(rec);
            this->urgent_poll_keys_.insert(k);
            any_missing = true;
            this->bus_b_cache_misses_++;
            ESP_LOGW(TAG, "[Bus B] Cache miss on unknown reg 0x%04X -> holding Bus B reply for Bus A urgent fetch", k);
          } else {
            it->second.read_count_wall++;
          }
        }

        if (any_missing && this->bus_a_cycles_ > 0) {
          this->bus_b_held_sender_ = sender;
          this->bus_b_held_read_keys_ = std::move(req_keys);
          this->bus_b_hold_start_ms_ = now;
          this->bus_b_state_ = BusBState::HOLD_FOR_CACHE_MISS;
        } else {
          this->send_bus_b_read_response_(sender, std::move(req_keys));
        }
        break;
      }

      case 0x40: {
        if (this->bus_b_state_ == BusBState::WAIT_TOKEN_ACCEPT) this->bus_b_state_ = BusBState::CONTROLLER_HAS_TOKEN;
        this->bus_b_last_ms_ = now;
        std::vector<uint8_t> ack = {0x41};

        size_t idx = 1;
        while (idx + 2 < payload.size()) {
          uint16_t k   = (uint16_t(payload[idx]) << 8) | payload[idx + 1];
          uint8_t vlen = payload[idx + 2];
          idx += 3;
          if (idx + vlen > payload.size()) break;

          std::vector<uint8_t> val(payload.begin() + idx, payload.begin() + idx + vlen);
          idx += vlen;

          ack.push_back((k >> 8) & 0xFF);
          ack.push_back(k & 0xFF);

          if (k != 0x0050) {
            this->regs_[k].write_count_wall++;
            if (k != 0x0450 && k != 0x0550) {
              ESP_LOGI(TAG, "[VTTOUCHW -> ERV] Forwarding write reg 0x%04X (len=%u): [%s]",
                       k, vlen, format_hex_(val.data(), val.size()).c_str());
            }
            this->enqueue_bus_a_write_(k, val);
          }
        }

        this->queue_frame_b_(sender, ack, 3);
        break;
      }

      default:
        break;
    }
  }

  void run_bus_b_state_machine_(uint32_t now) {
    if (!this->pending_tx_b_.empty()) return;

    switch (this->bus_b_state_) {
      case BusBState::WAIT_BUS_A_INIT: {
        // Serve the screen only once the cache holds fresh ERV values: the poll sweep is done and the
        // one-off re-reads are through. The 1.5 s fallback keeps the screen from waiting forever.
        const bool ready = this->bus_a_initial_sweep_done_ && this->refresh_keys_.empty();
        if (ready || (now - this->bus_b_last_ms_ >= 1500)) {
          if (ready) {
            ESP_LOGI(TAG, "[Bus B] Cache ready %u ms after link up (%u ERV turns)",
                     (unsigned) (now - this->link_up_ms_), (unsigned) (this->bus_a_cycles_ - this->link_up_cycles_));
          } else {
            ESP_LOGW(TAG, "[Bus B] Serving the screen after the 1.5 s fallback: sweep %s, %u one-off reads and "
                          "%u urgent reads still pending, %u ERV turns since link up, Bus A state %d",
                     this->bus_a_initial_sweep_done_ ? "done" : "NOT done", (unsigned) this->refresh_keys_.size(),
                     (unsigned) this->urgent_poll_keys_.size(),
                     (unsigned) (this->bus_a_cycles_ - this->link_up_cycles_), (int) this->bus_a_state_);
          }
          ESP_LOGI(TAG, "[Bus B] Starting VTTOUCHW discovery/token loop (target 0x12)");
          this->bus_b_missed_offers_ = 0;
          this->bus_b_state_ = BusBState::OFFER_TOKEN;
          this->bus_b_last_ms_ = now;
        }
        break;
      }

      case BusBState::DISCOVERY_PING: {
        this->queue_frame_b_(this->bus_b_ping_candidate_, {0x02, 'P', 'i', 'n', 'g'}, 2);
        this->bus_b_state_ = BusBState::WAIT_PING_REPLY;
        this->bus_b_last_ms_ = now;
        break;
      }

      case BusBState::WAIT_PING_REPLY: {
        if (now - this->bus_b_last_ms_ >= 100) {
          if (this->bus_b_ping_candidate_ == 0x12) {
            this->bus_b_ping_candidate_ = 0x11;
            this->bus_b_state_ = BusBState::DISCOVERY_PING;
          } else {
            this->bus_b_ping_candidate_ = 0x12;
            this->bus_b_state_ = BusBState::OFFER_TOKEN;
          }
          this->bus_b_last_ms_ = now;
        }
        break;
      }

      case BusBState::OFFER_TOKEN: {
        if (now - this->bus_b_last_ms_ >= 15) {
          this->queue_frame_b_(this->bus_b_ctrl_addr_, {0x04}, 2);
          if (!this->boot_.screen_accept) this->boot_.offers++;
          this->bus_b_state_ = BusBState::WAIT_TOKEN_ACCEPT;
          this->bus_b_last_ms_ = now;
          this->bus_b_offer_ms_ = now;
        }
        break;
      }

      case BusBState::WAIT_TOKEN_ACCEPT: {
        // The screen normally accepts within a few ms, but it can be slow (e.g. while redrawing after
        // a mode change). Re-offering after only 50 ms made our 0x04 collide with its late 0x05.
        uint32_t last = this->bus_b_last_ms_;
        if ((int32_t) (this->last_rx_b_ms_ - last) > 0) last = this->last_rx_b_ms_;
        if (now - last >= 250) {
          this->bus_b_missed_offers_++;
          if (this->bus_b_missed_offers_ >= 3) {
            this->bus_b_missed_offers_ = 0;
            this->bus_b_ping_candidate_ = 0x12;
            this->bus_b_state_ = BusBState::DISCOVERY_PING;
          } else {
            this->bus_b_state_ = BusBState::OFFER_TOKEN;
          }
          this->bus_b_last_ms_ = now;
        }
        break;
      }

      case BusBState::HOLD_FOR_CACHE_MISS: {
        // The ERV offers us a turn about every 90 ms, so a fetch can take ~150 ms in the worst case.
        // 250 ms leaves room for that and still answers well inside the screen's ~450 ms timeout.
        if (now - this->bus_b_hold_start_ms_ >= 250) {
          this->note_hold_(now, true);
          this->urgent_poll_keys_.clear();
          this->send_bus_b_read_response_(this->bus_b_held_sender_, this->bus_b_held_read_keys_);
          this->bus_b_state_ = BusBState::CONTROLLER_HAS_TOKEN;
          this->bus_b_last_ms_ = now;
        }
        break;
      }

      case BusBState::CONTROLLER_HAS_TOKEN: {
        // Count from the last byte heard, not just the last complete frame, and give the screen
        // more time: it has been measured holding the token for 800+ ms.
        uint32_t last = this->bus_b_last_ms_;
        if ((int32_t) (this->last_rx_b_ms_ - last) > 0) last = this->last_rx_b_ms_;
        if (now - last >= 1200) {
          ESP_LOGI(TAG, "[Bus B] VTTOUCHW silent for %u ms while holding the token -> offering it again",
                   (unsigned) (now - last));
          this->bus_b_state_ = BusBState::OFFER_TOKEN;
          this->bus_b_last_ms_ = now;
        }
        break;
      }
    }
  }

  void build_poll_batches_() {
    this->poll_batches_.push_back({
      0x0020, 0x0022, 0x0120, 0x0122, 0x0220, 0x0620, 0x0720, 0x0820,
      0x01E0, 0x0222, 0x0322, 0x0A22, 0x0C22, 0x0E22,
      0x0F22, 0x1022, 0x0830, 0x0930
    });
    this->poll_batches_.push_back({
      0x0030, 0x0230, 0x0330, 0x0430, 0x0421, 0x0521,
      0x0621, 0x0C21, 0x0D21, 0x0850, 0x0950, 
      0x1250, 0x1350, 0x1450, 0x1950, 0x2050, 0x2150
    });
    this->poll_batches_.push_back({
      0x1200, 0x1400, 0x1A00, 0x0AF0, 0x0510, 0x0610, 0x0100
    });
  }

  // Registers the change watcher skips: they move all the time (airflow, temperatures, humidity,
  // filter countdown) or already have their own "[ERV state]" line.
  static bool watch_ignored_(uint16_t k) {
    switch (k) {
      case 0x0510: case 0x0610: case 0x01E0: case 0x0450: case 0x0550: case 0x0830:
      case 0x1400:   // counts up once a second (ERV time since power-on)
      case 0x0020: case 0x0120: case 0x0620: case 0x0720: case 0x0E22: case 0x0322: case 0x0F22: case 0x1022:
        return true;
      default:
        return false;
    }
  }

  bool in_poll_batches_(uint16_t key) const {
    for (const auto &b : this->poll_batches_)
      for (uint16_t k : b)
        if (k == key) return true;
    return false;
  }

  bool is_in_core_batches_(uint16_t key) const {
    if (key == 0x0BF0) return true;
    for (const auto &b : this->poll_batches_) {
      for (uint16_t k : b) {
        if (k == key) return true;
      }
    }
    return false;
  }

  static size_t guess_len_(uint16_t key) {
    switch (key & 0xFF) {
      case 0x20: case 0x21: return 1;
      case 0x00: case 0x10: case 0x50: case 0xE0: return 4;
      case 0x22: return (key == 0x0322 || key == 0x0E22 || key == 0x0F22 || key == 0x1022) ? 1 : 4;
      case 0x30: return (key == 0x0030 || key == 0x0130 || key == 0x0230) ? 1 : 4;
      default:   return 1;
    }
  }

  void seed_reg_(uint16_t key, std::initializer_list<uint8_t> bytes) {
    RegisterRecord rec;
    rec.key = key;
    rec.value.assign(bytes.begin(), bytes.end());
    rec.seeded = true;
    this->regs_[key] = std::move(rec);
  }

  void seed_defaults_() {
    seed_reg_(0x0020, {0x08});
    seed_reg_(0x0022, {0x00, 0x00, 0x00, 0x00});
    seed_reg_(0x0030, {0x00});
    seed_reg_(0x0100, {0x01, 0xCC, 0x28});
    seed_reg_(0x0120, {0x08});   // mirrors the 0x0020 seed
    seed_reg_(0x01E0, {0x95, 0x3A, 0x85, 0x41});
    seed_reg_(0x0220, {0x08});
    seed_reg_(0x0222, {0x60, 0x09, 0x00, 0x00});
    seed_reg_(0x0230, {0x01});
    seed_reg_(0x0322, {0x00});
    seed_reg_(0x0330, {0x00, 0x00, 0x00, 0x00});
    seed_reg_(0x0421, {0x00});
    seed_reg_(0x0422, {0x00, 0x00, 0x9E, 0x42});
    seed_reg_(0x0430, {0x00, 0x00, 0x00, 0x00});
    seed_reg_(0x0450, {0x79, 0x4C, 0x3A, 0x42});
    seed_reg_(0x0510, {0xCA, 0xEC, 0xA9, 0x42});
    seed_reg_(0x0521, {0x00});
    seed_reg_(0x0550, {0x44, 0x6C, 0xBA, 0x41});
    seed_reg_(0x0610, {0xCA, 0x02, 0xAA, 0x42});
    seed_reg_(0x0620, {0x00});
    seed_reg_(0x0621, {0x00});
    seed_reg_(0x0622, {0x00, 0x00, 0x9E, 0x42});
    seed_reg_(0x0720, {0x02});
    seed_reg_(0x0820, {0x00});
    seed_reg_(0x0822, {0x00, 0x00, 0x9E, 0x42});
    seed_reg_(0x0830, {0x69, 0xBB, 0x6F, 0x00});
    seed_reg_(0x0850, {0x01});
    seed_reg_(0x0930, {0x00, 0xA7, 0x76, 0x00});
    seed_reg_(0x0950, {0x28, 0x00, 0x00, 0x00});
    seed_reg_(0x0A22, {0x00, 0x00, 0x20, 0x42});
    seed_reg_(0x0A50, {0x00, 0x00, 0x96, 0x42});
    seed_reg_(0x0AF0, {
      0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    });
    seed_reg_(0x0B50, {0x00, 0x00, 0x96, 0x42});
    seed_reg_(0x0BF0, {
      0x53, 0x54, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    });
    seed_reg_(0x0C21, {0x01});
    seed_reg_(0x0C22, {0x00, 0x00, 0x20, 0x42});
    seed_reg_(0x0C50, {0x00, 0x00, 0x0C, 0x42});
    seed_reg_(0x0D21, {0x01});
    seed_reg_(0x0D50, {0x00, 0x00, 0x96, 0x42});
    seed_reg_(0x0E22, {0x01});
    seed_reg_(0x0E50, {0x00, 0x00, 0xAA, 0x42});
    seed_reg_(0x0F22, {0x00});
    seed_reg_(0x0F50, {0x00, 0x00, 0xAA, 0x42});
    seed_reg_(0x1022, {0x00});
    seed_reg_(0x1050, {0x00, 0x00, 0x34, 0x42});
    seed_reg_(0x1150, {0x38, 0x0C, 0xF1, 0x42});
    seed_reg_(0x1200, {0xFF, 0xFF, 0xFF, 0xFF});
    seed_reg_(0x1250, {0x02});
    seed_reg_(0x1350, {0x03});
    seed_reg_(0x1400, {0x78, 0x00, 0x00, 0x00});
    seed_reg_(0x1450, {0x00});
    seed_reg_(0x1600, {});
    seed_reg_(0x1610, {0x37, 0xCC, 0x04, 0x43});
    seed_reg_(0x1650, {0x00});
    seed_reg_(0x1710, {0x38, 0x0C, 0xF1, 0x42});
    seed_reg_(0x1750, {0x00, 0x00, 0xAA, 0x42});
    seed_reg_(0x1850, {0x00, 0x00, 0xAA, 0x42});
    seed_reg_(0x1950, {0x00});
    seed_reg_(0x1A00, {0xFF, 0xFF, 0xFF, 0xFF});
    seed_reg_(0x2050, {0x01});
    seed_reg_(0x2150, {0x01});
  }

  float get_f32_reg_(uint16_t key, float fallback = 0.0f) const {
    auto it = this->regs_.find(key);
    if (it != this->regs_.end() && it->second.value.size() == 4) {
      float f = 0.0f;
      std::memcpy(&f, it->second.value.data(), 4);
      return f;
    }
    return fallback;
  }

  uint8_t get_u8_reg_(uint16_t key, uint8_t fallback = 0) const {
    auto it = this->regs_.find(key);
    return (it != this->regs_.end() && !it->second.value.empty()) ? it->second.value[0] : fallback;
  }

  uint32_t get_u32_reg_(uint16_t key, uint32_t fallback = 0) const {
    auto it = this->regs_.find(key);
    if (it != this->regs_.end() && it->second.value.size() == 4) {
      const auto &v = it->second.value;
      return uint32_t(v[0]) | (uint32_t(v[1]) << 8) | (uint32_t(v[2]) << 16) | (uint32_t(v[3]) << 24);
    }
    return fallback;
  }

  void publish_sensors_() {
    // Room humidity / temperature come from the screen (it writes them), not from the ERV. Until it
    // has, the cache only holds the built-in starting values, so nothing is published (HA keeps the
    // last real value through a reboot).
    if (this->wall_humidity_sensor_ != nullptr && this->regs_[0x0450].write_count_wall > 0) {
      this->wall_humidity_sensor_->publish_state(this->get_f32_reg_(0x0450));
    }
    if (this->wall_temp_sensor_ != nullptr && this->regs_[0x0550].write_count_wall > 0) {
      this->wall_temp_sensor_->publish_state(this->get_f32_reg_(0x0550));
    }
    if (this->erv_temp_sensor_ != nullptr) {
      this->erv_temp_sensor_->publish_state(this->get_f32_reg_(0x01E0));
    }
    if (this->supply_cfm_sensor_ != nullptr) {
      this->supply_cfm_sensor_->publish_state(this->get_f32_reg_(0x0510));
    }
    if (this->exhaust_cfm_sensor_ != nullptr) {
      this->exhaust_cfm_sensor_->publish_state(this->get_f32_reg_(0x0610));
    }
    if (this->filter_days_sensor_ != nullptr) {
      uint32_t sec = this->get_u32_reg_(0x0830);
      this->filter_days_sensor_->publish_state(static_cast<float>(sec) / 86400.0f);
    }
  }

  void note_hold_(uint32_t now, bool timed_out) {
    uint32_t ms = now - this->bus_b_hold_start_ms_;
    this->bus_b_holds_++;
    if (ms > this->bus_b_hold_max_ms_) this->bus_b_hold_max_ms_ = ms;
    if (timed_out) {
      this->bus_b_hold_timeouts_++;
      ESP_LOGW(TAG, "[Bus B] ERV did not answer a cache-miss fetch within %u ms; replying with cached/placeholder bytes",
               (unsigned) ms);
    }
  }

  void print_boot_report_() {
    const auto &b = this->boot_;
    auto rel = [&](uint32_t t) -> std::string {
      if (!t) return "not yet";
      char buf[16];
      snprintf(buf, sizeof(buf), "+%u ms", (unsigned) (t - b.t0));
      return buf;
    };
    ESP_LOGI(TAG, "BOOT | ERV first turn %s | first full read %s, %u extra registers read once, %u static values differed | "
                  "screen first frame %s, first token accepted %s after %u offers / %u ping replies",
             rel(b.erv_token).c_str(), rel(b.sweep).c_str(), (unsigned) b.refreshed, (unsigned) b.differs.size(),
             rel(b.screen_frame).c_str(), rel(b.screen_accept).c_str(), (unsigned) b.offers, (unsigned) b.pings);
    if (b.startup_mode) {
      ESP_LOGI(TAG, "BOOT | ERV reported start-up mode 0x14 until %s", rel(b.startup_end).c_str());
    }
    if (!b.differs.empty()) {
      std::string list;
      for (uint16_t k : b.differs) { char buf[8]; snprintf(buf, sizeof(buf), " %04X", k); list += buf; }
      ESP_LOGI(TAG, "BOOT | starting values that differed from this ERV:%s", list.c_str());
    }
  }

  void print_summary_() {
    ESP_LOGI(TAG, "PROXY | BusA(ERV) Cyc:%u RxB:%u Err:%u | BusB(Wall) Cyc:%u RxB:%u Echo:%u Err:%u Miss:%u | Mode:0x%02X",
             (unsigned) this->bus_a_cycles_, (unsigned) this->bus_a_rx_bytes_, (unsigned) this->bus_a_chk_err_,
             (unsigned) this->bus_b_cycles_, (unsigned) this->bus_b_rx_bytes_, (unsigned) this->bus_b_echoes_,
             (unsigned) this->bus_b_chk_err_, (unsigned) this->bus_b_cache_misses_, this->get_current_mode_byte());
    if (millis() - this->boot_.t0 < 180000) this->print_boot_report_();
    ESP_LOGI(TAG, "       BusB token accept: slowest %u ms, %u slower than 50 ms (last 15 s)",
             (unsigned) this->bus_b_accept_max_ms_, (unsigned) this->bus_b_accept_late_);
    if (this->watch_suppressed_ > 0) {
      ESP_LOGI(TAG, "       Watch: %u more register changes not logged (max 1 per register per minute)",
               (unsigned) this->watch_suppressed_);
      this->watch_suppressed_ = 0;
    }
    if (this->bus_b_holds_ > 0) {
      ESP_LOGI(TAG, "       BusB cache-miss holds: %u, longest %u ms, %u timed out (last 15 s)",
               (unsigned) this->bus_b_holds_, (unsigned) this->bus_b_hold_max_ms_, (unsigned) this->bus_b_hold_timeouts_);
    }
    this->bus_b_accept_max_ms_ = 0;
    this->bus_b_accept_late_ = 0;
    this->bus_b_holds_ = 0;
    this->bus_b_hold_max_ms_ = 0;
    this->bus_b_hold_timeouts_ = 0;
  }

  void dump_some_registers_() {
    auto it = this->regs_.lower_bound(this->dump_next_key_);
    for (int n = 0; n < 4 && it != this->regs_.end(); n++, ++it) {
      const auto &r = it->second;
      ESP_LOGI(TAG, "  Reg 0x%04X | ERV:%s | Wall R:%4u W:%2u | Len:%3u | [%s%s]",
               it->first, r.fetched_from_erv ? "YES " : "SEED",
               (unsigned) r.read_count_wall, (unsigned) r.write_count_wall, (unsigned) r.value.size(),
               format_hex_(r.value.data(), std::min<size_t>(r.value.size(), 12)).c_str(),
               r.value.size() > 12 ? " ..." : "");
    }
    if (it == this->regs_.end()) {
      ESP_LOGI(TAG, "===============================================================");
      this->dump_active_ = false;
    } else {
      this->dump_next_key_ = it->first;
    }
  }
};

}  // namespace broan_erv_proxy
}  // namespace esphome
