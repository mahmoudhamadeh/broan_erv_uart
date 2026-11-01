#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

#include <algorithm>
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
  uint32_t last_erv_update_ms{0};
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

  uint32_t get_intermittent_seconds() const {
    auto it = this->regs_.find(0x0222);
    if (it != this->regs_.end() && it->second.value.size() == 4) {
      const auto &v = it->second.value;
      return uint32_t(v[0]) | (uint32_t(v[1]) << 8) | (uint32_t(v[2]) << 16) | (uint32_t(v[3]) << 24);
    }
    return 2400;
  }

  uint32_t get_humidity_setpoint() const {
    auto it = this->regs_.find(0x0950);
    if (it != this->regs_.end() && it->second.value.size() == 4) {
      const auto &v = it->second.value;
      return uint32_t(v[0]) | (uint32_t(v[1]) << 8) | (uint32_t(v[2]) << 16) | (uint32_t(v[3]) << 24);
    }
    return 40;
  }

  void write_mode_from_ha(uint8_t mode_byte) {
    if (this->get_current_mode_byte() == mode_byte) return;
    ESP_LOGI(TAG, "[HA -> ERV] Setting operating mode 0x0020 = 0x%02X", mode_byte);
    this->enqueue_bus_a_write_(0x0020, {mode_byte});
    this->enqueue_bus_a_write_(0x0820, {0x00});
  }

  void write_intermittent_minutes_from_ha(uint32_t minutes_per_hour) {
    uint32_t sec = minutes_per_hour * 60;
    if (this->get_intermittent_seconds() == sec) return;
    std::vector<uint8_t> bytes = {
        static_cast<uint8_t>(sec & 0xFF),
        static_cast<uint8_t>((sec >> 8) & 0xFF),
        static_cast<uint8_t>((sec >> 16) & 0xFF),
        static_cast<uint8_t>((sec >> 24) & 0xFF),
    };
    ESP_LOGI(TAG, "[HA -> ERV] Setting intermittent time 0x0222 = %u s (%u min/hr)", (unsigned) sec, (unsigned) minutes_per_hour);
    this->enqueue_bus_a_write_(0x0222, bytes);
  }

  void write_humidity_setpoint_from_ha(uint32_t rh_percent) {
    if (this->get_humidity_setpoint() == rh_percent) return;
    std::vector<uint8_t> bytes = {
        static_cast<uint8_t>(rh_percent & 0xFF),
        static_cast<uint8_t>((rh_percent >> 8) & 0xFF),
        static_cast<uint8_t>((rh_percent >> 16) & 0xFF),
        static_cast<uint8_t>((rh_percent >> 24) & 0xFF),
    };
    ESP_LOGI(TAG, "[HA -> ERV] Setting humidity setpoint 0x0950 = %u%%", (unsigned) rh_percent);
    this->enqueue_bus_a_write_(0x0950, bytes);
  }
  
  uint32_t get_turbo_timeout_hours() const {
    auto it = this->regs_.find(0x0330);
    if (it != this->regs_.end() && it->second.value.size() == 4) {
      const auto &v = it->second.value;
      uint32_t sec = uint32_t(v[0]) | (uint32_t(v[1]) << 8) | (uint32_t(v[2]) << 16) | (uint32_t(v[3]) << 24);
      if (sec >= 3600) return sec / 3600;
    }
    return 4; // Default to 4 hours if unset
  }

  void write_turbo_timeout_hours_from_ha(uint32_t hours) {
    uint32_t sec = hours * 3600;
    std::vector<uint8_t> bytes = {
        static_cast<uint8_t>(sec & 0xFF),
        static_cast<uint8_t>((sec >> 8) & 0xFF),
        static_cast<uint8_t>((sec >> 16) & 0xFF),
        static_cast<uint8_t>((sec >> 24) & 0xFF),
    };
    ESP_LOGI(TAG, "[HA -> ERV] Setting Turbo/Override timeout 0x0330 = %u s (%u h)", (unsigned) sec, (unsigned) hours);
    this->enqueue_bus_a_write_(0x0330, bytes);
    this->enqueue_bus_a_write_(0x0430, bytes);
  }

  void reset_filter_timer_from_ha() {
    uint32_t reset_sec = this->get_u32_reg_(0x0930, 7776000);
    std::vector<uint8_t> bytes = {
        static_cast<uint8_t>(reset_sec & 0xFF),
        static_cast<uint8_t>((reset_sec >> 8) & 0xFF),
        static_cast<uint8_t>((reset_sec >> 16) & 0xFF),
        static_cast<uint8_t>((reset_sec >> 24) & 0xFF),
    };
    ESP_LOGI(TAG, "[HA -> ERV] Resetting filter remaining timer 0x0830 to %u s", (unsigned) reset_sec);
    this->enqueue_bus_a_write_(0x0830, bytes);
  }

  void dump_status() { this->print_summary_(millis(), true); }

  void loop() override {
    const uint32_t now = millis();

    // ---- BUS A (Real ERV) RX + TX + State Machine ----
    this->drain_uart_(this->uart_a_, this->rx_buf_a_, this->last_rx_a_ms_, this->bus_a_rx_bytes_, now);
    if (!this->rx_buf_a_.empty() && (now - this->last_rx_a_ms_ > 60)) {
      this->rx_buf_a_.clear();
    }
    this->process_rx_a_(now);

    if (!this->pending_tx_a_.empty() && (now >= this->tx_ready_a_ms_)) {
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
      ESP_LOGW(TAG, "[Bus B] Dropping %u unparsed RX bytes: [%s]",
               (unsigned) this->rx_buf_b_.size(),
               format_hex_(this->rx_buf_b_.data(), std::min<size_t>(this->rx_buf_b_.size(), 16)).c_str());
      this->rx_buf_b_.clear();
    }
    this->process_rx_b_(now);

    if (!this->pending_tx_b_.empty() && (now >= this->tx_ready_b_ms_)) {
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
      this->print_summary_(now, false);
    }
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
  std::set<uint16_t> urgent_poll_keys_;
  std::set<uint16_t> dynamic_extra_keys_;
  uint32_t dynamic_extra_expire_ms_{0};
  std::vector<std::vector<uint16_t>> poll_batches_;
  size_t current_batch_idx_{0};

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
  std::vector<uint16_t> bus_b_held_read_keys_;
  uint8_t bus_b_held_sender_{0x12};
  uint32_t bus_b_hold_start_ms_{0};
  std::vector<uint8_t> rx_buf_b_;
  std::vector<uint8_t> pending_tx_b_;
  std::deque<SentFrameRecord> recent_tx_b_;
  uint32_t last_rx_b_ms_{0};
  uint32_t tx_ready_b_ms_{0};

  uint32_t last_summary_ms_{0};
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

  void queue_frame_a_(uint8_t target, const std::vector<uint8_t> &payload, uint32_t delay_ms = 3) {
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

    if (!val.empty()) {
      auto &rec = this->regs_[key];
      rec.key = key;
      rec.value = val;
      rec.fetched_from_erv = true;
      if (key == 0x0020 || key == 0x0220) {
        this->regs_[0x0020].value = val;
        this->regs_[0x0120].value = val;
        this->regs_[0x0220].value = val;
      }
    }

    if (!val.empty()) {
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

          auto &rec = this->regs_[k];
          rec.key = k;
          rec.value.assign(payload.begin() + idx, payload.begin() + idx + vlen);
          rec.fetched_from_erv = true;
          rec.last_erv_update_ms = now;
          this->urgent_poll_keys_.erase(k);
          idx += vlen;
        }

        if (this->bus_b_state_ == BusBState::HOLD_FOR_CACHE_MISS && this->urgent_poll_keys_.empty()) {
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

        if (!this->urgent_poll_keys_.empty()) {
          size_t count = 0;
          for (uint16_t k : this->urgent_poll_keys_) {
            pl.push_back((k >> 8) & 0xFF);
            pl.push_back(k & 0xFF);
            if (++count >= 16) break;
          }
        } else if (!this->dynamic_extra_keys_.empty() && (now < this->dynamic_extra_expire_ms_) &&
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
            ESP_LOGI(TAG, "[Bus A] Initial register sweep from real ERV complete! Enabling Bus B.");
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
        if (this->bus_a_initial_sweep_done_ || (now - this->bus_b_last_ms_ >= 1500)) {
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
        if (now - this->bus_b_last_ms_ >= 45) {
          // Alternate rapidly between offering token to 0x12 and pinging 0x12 (plus sweeping 0x11)
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
          this->bus_b_state_ = BusBState::WAIT_TOKEN_ACCEPT;
          this->bus_b_last_ms_ = now;
        }
        break;
      }

      case BusBState::WAIT_TOKEN_ACCEPT: {
        if (now - this->bus_b_last_ms_ >= 50) {
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
        if (now - this->bus_b_hold_start_ms_ >= 150) {
          this->urgent_poll_keys_.clear();
          this->send_bus_b_read_response_(this->bus_b_held_sender_, this->bus_b_held_read_keys_);
          this->bus_b_state_ = BusBState::CONTROLLER_HAS_TOKEN;
          this->bus_b_last_ms_ = now;
        }
        break;
      }

      case BusBState::CONTROLLER_HAS_TOKEN: {
        if (now - this->bus_b_last_ms_ >= 600) {
          this->bus_b_state_ = BusBState::OFFER_TOKEN;
          this->bus_b_last_ms_ = now;
        }
        break;
      }
    }
  }

  void build_poll_batches_() {
    this->poll_batches_.push_back({
      0x0020, 0x0120, 0x0220, 0x0620, 0x0720, 0x0820,
      0x01E0, 0x0222, 0x0322, 0x0A22, 0x0C22, 0x0E22,
      0x0F22, 0x1022, 0x0830, 0x0930
    });
    this->poll_batches_.push_back({
      0x0030, 0x0230, 0x0330, 0x0430, 0x0421, 0x0521,
      0x0621, 0x0C21, 0x0D21, 0x0850, 0x0950, 0x1250,
      0x1350, 0x1450, 0x1950, 0x2050, 0x2150
    });
    this->poll_batches_.push_back({
      0x1200, 0x1400, 0x1A00, 0x0AF0, 0x0510, 0x0610, 0x0100
    });
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
    this->regs_[key] = std::move(rec);
  }

  void seed_defaults_() {
    seed_reg_(0x0020, {0x08});
    seed_reg_(0x0022, {});
    seed_reg_(0x0030, {0x00});
    seed_reg_(0x0100, {0x01, 0xCC, 0x28});
    seed_reg_(0x0120, {0x08});
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

  uint32_t get_u32_reg_(uint16_t key, uint32_t fallback = 0) const {
    auto it = this->regs_.find(key);
    if (it != this->regs_.end() && it->second.value.size() == 4) {
      const auto &v = it->second.value;
      return uint32_t(v[0]) | (uint32_t(v[1]) << 8) | (uint32_t(v[2]) << 16) | (uint32_t(v[3]) << 24);
    }
    return fallback;
  }

  void publish_sensors_() {
    if (this->wall_humidity_sensor_ != nullptr) {
      this->wall_humidity_sensor_->publish_state(this->get_f32_reg_(0x0450));
    }
    if (this->wall_temp_sensor_ != nullptr) {
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

  void print_summary_(uint32_t /*now*/, bool full_dump) {
    ESP_LOGI(TAG, "PROXY | BusA(ERV) Cyc:%u RxB:%u Err:%u | BusB(Wall) Cyc:%u RxB:%u Echo:%u Err:%u | Mode:0x%02X",
             (unsigned) this->bus_a_cycles_, (unsigned) this->bus_a_rx_bytes_, (unsigned) this->bus_a_chk_err_,
             (unsigned) this->bus_b_cycles_, (unsigned) this->bus_b_rx_bytes_, (unsigned) this->bus_b_echoes_,
             (unsigned) this->bus_b_chk_err_, this->get_current_mode_byte());

    if (!full_dump) return;

    ESP_LOGI(TAG, "================ STAGE 2 PROXY REGISTER DUMP ================");
    for (const auto &kv : this->regs_) {
      const auto &r = kv.second;
      char hex[64] = {0};
      size_t max_b = std::min<size_t>(r.value.size(), 12);
      size_t pos = 0;
      for (size_t i = 0; i < max_b; i++) {
        pos += snprintf(hex + pos, sizeof(hex) - pos, "%s%02X", (i == 0 ? "" : " "), r.value[i]);
      }
      ESP_LOGI(TAG, "  Reg 0x%04X | ERV:%s | Wall R:%4u W:%2u | Len:%3u | [%s%s]",
               r.key, r.fetched_from_erv ? "YES" : "SEED",
               (unsigned) r.read_count_wall, (unsigned) r.write_count_wall,
               (unsigned) r.value.size(), hex, (r.value.size() > 12 ? " ..." : ""));
    }
    ESP_LOGI(TAG, "=============================================================");
  }
};

}  // namespace broan_erv_proxy
}  // namespace esphome