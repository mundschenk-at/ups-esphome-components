#include "protocol_eaton.h"
#include "ups_hid.h"
#include "constants_hid.h"
#include "constants_ups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_err.h"
#include "esp_log_buffer.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cctype>

namespace esphome {
namespace ups_hid {

static const char *const EATON_TAG = "ups_hid.eaton_hid";

// Present Status bitmasks
static const uint8_t EATON_PRESENT_AC_PRESENT               = 0x01;
static const uint8_t EATON_PRESENT_BELOW_REMAINING_CAPACITY = 0x02;
static const uint8_t EATON_PRESENT_CHARGING                 = 0x04;
static const uint8_t EATON_PRESENT_COMMUNICATION_LOST       = 0x08;
static const uint8_t EATON_PRESENT_DISCHARGING              = 0x10;
static const uint8_t EATON_PRESENT_GOOD                     = 0x20;
static const uint8_t EATON_PRESENT_INTERNAL_FAILURE         = 0x40;
static const uint8_t EATON_PRESENT_NEED_REPLACEMENT         = 0x80;

bool EatonProtocol::detect() {
  ESP_LOGD(EATON_TAG, "Detecting Eaton HID protocol");

  // Check device connection status first
  if (!parent_->is_device_connected()) {
    ESP_LOGD(EATON_TAG, "Device not connected, skipping protocol detection");
    return false;
  }

  // Give device time to initialize after connection (same as APC)
  vTaskDelay(pdMS_TO_TICKS(timing::USB_INITIALIZATION_DELAY_MS));

  // Test multiple report IDs that are known to work with Eaton devices
  // Based on NUT debug logs
  const uint8_t test_report_ids[] = {
    BATTERY_RUNTIME_REPORT_ID,
    PRESENT_STATUS_REPORT_ID,
    OUTPUT_VOLTAGE_REPORT_ID,
    BATTERY_SYSTEM_REPORT_ID
  };

  HidReport test_report;

  for (uint8_t report_id : test_report_ids) {
    // Check device connection before each report attempt
    if (!parent_->is_device_connected()) {
      ESP_LOGD(EATON_TAG, "Device disconnected during protocol detection");
      return false;
    }

    ESP_LOGD(EATON_TAG, "Testing report ID 0x%02X...", report_id);

    if (read_hid_report(report_id, test_report)) {
      ESP_LOGI(EATON_TAG, "Eaton HID protocol detected via report 0x%02X (%zu bytes)",
               report_id, test_report.data.size());
      return true;
    }

    // Small delay between attempts
    vTaskDelay(pdMS_TO_TICKS(timing::REPORT_RETRY_DELAY_MS));
  }

  ESP_LOGD(EATON_TAG, "Eaton HID protocol not detected");
  return false;
}

bool EatonProtocol::initialize() {
  ESP_LOGI(EATON_TAG, "Initializing Eaton HID protocol");

  // Reset scaling factors
  battery_voltage_scale_ = 1.0f;
  battery_scale_checked_ = false;

  return true;
}

bool EatonProtocol::read_data(UpsData &data) {
  ESP_LOGD(EATON_TAG, "Reading Eaton HID data");

  bool success = false;

  // Read general device information
  HidReport device_info_report;
  if (read_hid_report(DEVICE_INFORMATION_REPORT_ID, device_info_report)) {
    parse_device_information_report(device_info_report, data);
    success = true;
  }

  // Core sensors (essential for operation)
  // Read battery capacity limits (Report 0x08)
  HidReport battery_capacity_report;
  if (read_hid_report(BATTERY_CAPACITY_REPORT_ID, battery_capacity_report)) {
    parse_battery_capacity_report(battery_capacity_report, data);
    success = true;
  }

  // Read battery level and runtime (Report 0x06)
  HidReport battery_runtime_report;
  if (read_hid_report(BATTERY_RUNTIME_REPORT_ID, battery_runtime_report)) {
    parse_battery_runtime_report(battery_runtime_report, data);
    success = true;
  }

  // Read status flags (Report 0x01)
  HidReport status_report;
  if (read_hid_report(PRESENT_STATUS_REPORT_ID, status_report)) {
    parse_present_status_report(status_report, data);
    success = true;
  }

  // Read input voltage status report (Report 0x03)
  HidReport voltage_status_report;
  if (read_hid_report(VOLTAGE_STATUS_REPORT_ID, voltage_status_report)) {
    parse_voltage_status_report(voltage_status_report, data);
    success = true;
  }

  // Read output voltage (Report 0x0e)
  HidReport output_voltage_report;
  if (read_hid_report(OUTPUT_VOLTAGE_REPORT_ID, output_voltage_report)) {
    parse_output_voltage_report(output_voltage_report, data);
    success = true;
  }

  // Read output voltage nominal (Report 0x12)
  HidReport config_voltage_report;
  if (read_hid_report(CONFIG_VOLTAGE_REPORT_ID, config_voltage_report)) {
    parse_config_voltage_report(config_voltage_report, data);
  }

  // Read load percentage (Report 0x07)
  HidReport battery_system_report;
  if (read_hid_report(BATTERY_SYSTEM_REPORT_ID, battery_system_report)) {
    parse_battery_system_report(battery_system_report, data);
    success = true;
  }

  // Additional sensors (enhance functionality)
  // Read battery voltage (Report 0x0a)
/*
  HidReport battery_voltage_report;
  if (read_hid_report(BATTERY_VOLTAGE_REPORT_ID, battery_voltage_report)) {
    parse_battery_voltage_report(battery_voltage_report, data);
  }

  // Read battery voltage nominal (Report 0x09)
  HidReport battery_voltage_nominal_report;
  if (read_hid_report(BATTERY_VOLTAGE_NOMINAL_REPORT_ID, battery_voltage_nominal_report)) {
    parse_battery_voltage_nominal_report(battery_voltage_nominal_report, data);
  }
*/

  // Read input transfer limits (Report 0x13)
  HidReport input_transfer_high_report;
  if (read_hid_report(INPUT_TRANSFER_HIGH_REPORT_ID, input_transfer_high_report)) {
    parse_input_transfer_high_report(input_transfer_high_report, data);
  }

  // Read input transfer limits (Report 0x14)
  HidReport input_transfer_low_report;
  if (read_hid_report(INPUT_TRANSFER_LOW_REPORT_ID, input_transfer_low_report)) {
    parse_input_transfer_low_report(input_transfer_low_report, data);
  }

  // Read delay settings (Reports 0x09, 0x0a)
  HidReport delay_shutdown_report;
  if (read_hid_report(DELAY_SHUTDOWN_REPORT_ID, delay_shutdown_report)) {
    parse_delay_shutdown_report(delay_shutdown_report, data);
  }

  HidReport delay_start_report;
  if (read_hid_report(DELAY_START_REPORT_ID, delay_start_report)) {
    parse_delay_start_report(delay_start_report, data);
  }

  // Read nominal power (Report 0x18)
  /*HidReport realpower_nominal_report;
  if (read_hid_report(REALPOWER_NOMINAL_REPORT_ID, realpower_nominal_report)) {
    parse_realpower_nominal_report(realpower_nominal_report, data);
  }

  // Read input sensitivity (Report 0x1a)
  HidReport input_sensitivity_report;
  if (read_hid_report(INPUT_SENSITIVITY_REPORT_ID, input_sensitivity_report)) {
    parse_input_sensitivity_report(input_sensitivity_report, data);
  }

  // Read overload status (Report 0x17)
  HidReport overload_report;
  if (read_hid_report(OVERLOAD_REPORT_ID, overload_report)) {
    parse_overload_report(overload_report, data);
  }
*/
  // Read beeper status (Report 0x1f)
  HidReport beeper_status_report;
  if (read_hid_report(BEEPER_STATUS_REPORT_ID, beeper_status_report)) {
    parse_beeper_status_report(beeper_status_report, data);
  }

  // Set frequency to NaN - not available for Eaton CP1500 model
  // Try to read frequency from HID reports
  read_frequency_data(data);

  if (success) {
    ESP_LOGD(EATON_TAG, "Eaton data read completed successfully");
  } else {
    ESP_LOGW(EATON_TAG, "Failed to read any Eaton HID reports");
    // Leave manufacturer and model unset when HID communication fails
    data.device.manufacturer.clear();
    data.device.model.clear();
  }

  return success;
}

bool EatonProtocol::read_hid_report(uint8_t report_id, HidReport &report) {
  // Check device connection before any HID communication
  if (!parent_->is_device_connected()) {
    ESP_LOGV(EATON_TAG, "Device not connected, skipping HID report 0x%02X", report_id);
    return false;
  }

  uint8_t buffer[limits::MAX_HID_REPORT_SIZE]; // Maximum HID report size
  size_t buffer_len = sizeof(buffer);
  esp_err_t ret;

  // Add debug info about parent device state
  ESP_LOGD(EATON_TAG, "Attempting to read report 0x%02X from parent device", report_id);

  // Eaton devices primarily use Feature Reports (0x03) - based on NUT debug logs
  ret = parent_->hid_get_report(HID_REPORT_TYPE_FEATURE, report_id, buffer, &buffer_len, parent_->get_protocol_timeout());
  if (ret == ESP_OK && buffer_len > 0) {
    report.report_id = report_id;
    report.data.assign(buffer, buffer + buffer_len);
    ESP_LOGD(EATON_TAG, "READ SUCCESS: Report 0x%02X (%zu bytes)", report_id, buffer_len);
    return true;
  }

  // Log the specific error for Feature Report
  ESP_LOGD(EATON_TAG, "Feature Report 0x%02X failed: %s", report_id, esp_err_to_name(ret));

  // Check connection again before trying Input report
  if (!parent_->is_device_connected()) {
    ESP_LOGV(EATON_TAG, "Device disconnected during HID communication for report 0x%02X", report_id);
    return false;
  }

  // Fallback: try Input Report (0x01) for real-time data
  buffer_len = sizeof(buffer);
  ret = parent_->hid_get_report(HID_REPORT_TYPE_INPUT, report_id, buffer, &buffer_len, parent_->get_protocol_timeout());
  if (ret == ESP_OK && buffer_len > 0) {
    report.report_id = report_id;
    report.data.assign(buffer, buffer + buffer_len);
    ESP_LOGD(EATON_TAG, "READ SUCCESS (Input): Report 0x%02X (%zu bytes)", report_id, buffer_len);
    return true;
  }

  // Log the specific error for Input Report
  ESP_LOGD(EATON_TAG, "Input Report 0x%02X failed: %s", report_id, esp_err_to_name(ret));
  ESP_LOGV(EATON_TAG, "Failed to read report 0x%02X: %s", report_id, esp_err_to_name(ret));
  return false;
}

bool EatonProtocol::read_usb_descriptor(uint8_t index, std::string &string, const std::string &info) {
    esp_err_t ret = parent_->usb_get_string_descriptor(index, string);

    if (ret == ESP_OK && !string.empty()) {
      ESP_LOGI(EATON_TAG, "Successfully read Eaton %s from USB descriptor: \"%s\"", info.c_str(), string.c_str());
      return true;
    }

    string.clear();  // Set to unset state instead of hardcoded fallback
    ESP_LOGW(EATON_TAG, "Failed to read USB descriptor: %s, leaving %s unset", esp_err_to_name(ret), info.c_str());
    return false;
}

void EatonProtocol::parse_device_information_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 8) {
    ESP_LOGW(EATON_TAG, "Device information report too short: %zu bytes", report.data.size());
    return;
  }

  // NUT debug: ReportID: 0x10, Offset: 0, Value: 5 (iDeviceChemistry)
  // These are all USB string descriptor indices.
  uint8_t device_chemistry_index = report.data[1];
  uint8_t manufacturer_index = report.data[2];
  uint8_t model_index = report.data[3];
  uint8_t product_index = report.data[4];
  uint8_t reference_number_index = report.data[5];
  uint8_t serial_number_index = report.data[6];
  uint8_t version_index = report.data[7];

  // Use real USB string descriptor reading - this will get the battery chemistry
  // NUT shows: UPS.PowerSummary.iDeviceChemistry = "PbAcid"
  std::string device_chemistry;
  if (read_usb_descriptor(device_chemistry_index, device_chemistry, "battery chemistry")) {
    data.battery.type = device_chemistry;

    // Normalize battery chemistry names
    if ( data.battery.type == "PbAc" ) {
      data.battery.type = battery_chemistry::LEAD_ACID;
    }
  } else {
    data.battery.type = battery_chemistry::UNKNOWN;
  }

  // NUT shows: UPS.PowerSummary.iManufacturer, Value: 1 → "EATON"
  std::string manufacturer;
  read_usb_descriptor(manufacturer_index, manufacturer, "manufacturer");
  data.device.manufacturer = manufacturer;

  // NUT shows: UPS.PowerSummary.iModel, Value: 2 → "XXXX"
  std::string model;
  read_usb_descriptor(model_index, model, "model");

  // NUT shows: UPS.PowerSummary.iProduct, Value: 3 → "Eaton 3S"
  std::string product;
  read_usb_descriptor(product_index, product, "product");

  // The actual model needs iModel and iProduct für Eaton devices
  data.device.model = product + " " + model;

  // NUT shows: UPS.PowerSummary.iReferenceNumber, Value: 7 → "XXXX"
  std::string reference_number;
  read_usb_descriptor(reference_number_index, reference_number, "reference number");
  // Read just for debugging

  // NUT shows: UPS.PowerSummary.iSerialNumber, Value: 4 → "XXXX"
  std::string serial_number;
  read_usb_descriptor(serial_number_index, serial_number, "serial number");
  data.device.serial_number = serial_number;

  // NUT shows: UPS.PowerSummary.iVersion, Value: 6 → "XXXX"
  std::string version;
  read_usb_descriptor(version_index, version, "version");
  data.device.firmware_version = version;
}

void EatonProtocol::parse_battery_runtime_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 5) {
    ESP_LOGW(EATON_TAG, "Battery runtime report too short: %zu bytes", report.data.size());
    return;
  }

  // Path: UPS.PowerSummary.RemainingCapacity, Type: Feature, ReportID: 0x06, Offset: 0, Size: 8
  // Path: UPS.PowerSummary.RunTimeToEmpty, Type: Feature, ReportID: 0x06, Offset: 8, Size: 32

  // NUT mapping:
  // Offset 0 (byte 1): RemainingCapacity (battery %) - Size: 8
  // Offset 8 (bytes 2-4): RunTimeToEmpty - Size: 32, little-endian (IN SECONDS)
  uint8_t battery_percentage = report.data[1];
  uint16_t runtime_seconds = report.data[2] | (report.data[3] << 8 ) | (report.data[4] << 16 );

  // Clamp battery to 100% like NUT does
  data.battery.level = static_cast<float>(battery_percentage > battery::MAX_LEVEL_PERCENT ? battery::MAX_LEVEL_PERCENT : battery_percentage);

  // CRITICAL FIX: Convert runtime from seconds to minutes
  // Eaton reports runtime in seconds, but ESPHome expects minutes
  data.battery.runtime_minutes = static_cast<float>(runtime_seconds) / 60.0f;

  // Extract runtime low threshold if available (offset 24 = bytes 4-5)
  // FIXME - some other report
  if (report.data.size() >= 6) {
    uint16_t runtime_low_seconds = report.data[4] | (report.data[5] << 8);
    data.battery.runtime_low = static_cast<float>(runtime_low_seconds) / 60.0f;  // Convert to minutes
    ESP_LOGD(EATON_TAG, "Battery: %.0f%%, Runtime: %.1f min (%.0f sec), Runtime Low: %.1f min",
             data.battery.level, data.battery.runtime_minutes, static_cast<float>(runtime_seconds), data.battery.runtime_low);
  } else {
    ESP_LOGD(EATON_TAG, "Battery: %.0f%%, Runtime: %.1f min (%.0f sec raw: %02X %02X%02X)",
             data.battery.level, data.battery.runtime_minutes, static_cast<float>(runtime_seconds), battery_percentage, report.data[3], report.data[2]);
  }
}

void EatonProtocol::parse_present_status_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 3) {
    ESP_LOGW(EATON_TAG, "Present status report too short: %zu bytes", report.data.size());
    return;
  }

  // Based on NUT debug logs - bit flags in first byte
  uint8_t status_byte = report.data[1];

  // Path: UPS.PowerSummary.PresentStatus.ACPresent, Type: Feature, ReportID: 0x01, Offset: 0, Size: 1
  // Path: UPS.PowerSummary.PresentStatus.BelowRemainingCapacityLimit, Type: Feature, ReportID: 0x01, Offset: 1, Size: 1,
  // Path: UPS.PowerSummary.PresentStatus.Charging, Type: Feature, ReportID: 0x01, Offset: 2, Size: 1
  // Path: UPS.PowerSummary.PresentStatus.CommunicationLost, Type: Feature, ReportID: 0x01, Offset: 3
  // Path: UPS.PowerSummary.PresentStatus.Discharging, Type: Feature, ReportID: 0x01, Offset: 4, Size: 1
  // Path: UPS.PowerSummary.PresentStatus.Good, Type: Feature, ReportID: 0x01, Offset: 5, Size: 1
  // Path: UPS.PowerSummary.PresentStatus.InternalFailure, Type: Feature, ReportID: 0x01, Offset: 6, Size: 1
  // Path: UPS.PowerSummary.PresentStatus.NeedReplacement, Type: Feature, ReportID: 0x01, Offset: 7, Size: 1
  // Path: UPS.PowerSummary.PresentStatus.Overload, Type: Feature, ReportID: 0x01, Offset: 8, Size: 8

  // Parse status bits (based on HID paths from debug)
  bool ac_present         = status_byte & EATON_PRESENT_AC_PRESENT;
  bool below_capacity     = status_byte & EATON_PRESENT_BELOW_REMAINING_CAPACITY;
  bool charging           = status_byte & EATON_PRESENT_CHARGING;
  bool communication_lost = status_byte & EATON_PRESENT_COMMUNICATION_LOST;
  bool discharging        = status_byte & EATON_PRESENT_DISCHARGING;
  bool good               = status_byte & EATON_PRESENT_GOOD;
  bool internal_failure   = status_byte & EATON_PRESENT_INTERNAL_FAILURE;
  bool need_replacement   = status_byte & EATON_PRESENT_NEED_REPLACEMENT;

  bool overload           = report.data[2] != 0;
  bool shutdown_imminent  = report.data[3] != 0;

  // Update power status based on AC presence (as described in MGE-SHUT documentation)
  if (ac_present) {
    data.power.input_voltage = parent_->get_fallback_nominal_voltage();  // Use configured fallback voltage when AC present
    data.power.status = status::ONLINE;
  } else {
    data.power.input_voltage = NAN;     // No AC input
    data.power.status = status::ON_BATTERY;
  }

  if (overload) {
    data.power.status += " - Overload";
  }

  // Set battery status based on charging/discharging state
  if (charging) {
    data.battery.status = battery_status::CHARGING;
  } else if (discharging || !ac_present) {
    data.battery.status = battery_status::DISCHARGING;
  /*} else if (fully_charged) {
    data.battery.status = battery_status::FULLY_CHARGED;*/
  } else {
    data.battery.status = battery_status::NORMAL;
  }

  // Handle battery issues
  if (below_capacity || shutdown_imminent) {
    data.battery.charge_low = battery::LOW_THRESHOLD_PERCENT;  // Indicate low battery threshold
    if (shutdown_imminent) {
      data.battery.status += battery_status::SHUTDOWN_IMMINENT_SUFFIX;
    }
  }

  if (need_replacement) {
    data.battery.status += battery_status::REPLACE_BATTERY_SUFFIX;
  }

  ESP_LOGD(EATON_TAG, "Status: AC:%s Charging:%s Discharging:%s OnBatt:%s Overload:%s BattStatus:\"%s\" ShutdownImm:%s (raw: 0x%X)",
           ac_present ? "Yes" : "No",
           charging ? "Yes" : "No",
           discharging ? "Yes" : "No",
           (!ac_present || discharging) ? "Yes" : "No",
           overload ? "Yes" : "No",
           data.battery.status.c_str(),
           shutdown_imminent ? "Yes" : "No",
           status_byte);
}

void EatonProtocol::parse_voltage_status_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 3) {
    ESP_LOGW(EATON_TAG, "Voltage status report too short: %zu bytes", report.data.size());
    return;
  }

  // Path: UPS.BatterySystem.Charger.PresentStatus.VoltageTooHigh, Type: Feature, ReportID: 0x03, Offset: 0, Size: 8
  // Path: UPS.BatterySystem.Charger.PresentStatus.VoltageTooLow, Type: Feature, ReportID: 0x03, Offset: 8, Size: 8
  uint8_t voltage_too_high   = report.data[1];
  uint8_t voltage_too_low    = report.data[2];
  std::string voltage_status = {};

  if (voltage_too_high == 0x00 && voltage_too_low == 0x00) {
    voltage_status = "good";
  } else if (voltage_too_high > 0) {
    voltage_status = "critical-high";
  } else {
    voltage_status = "critical-low";
  }

  ESP_LOGD(EATON_TAG, "Input voltage status: %s (raw status high: %zu, raw status low: %zu", voltage_status.c_str(), voltage_too_high, voltage_too_low);
}

void EatonProtocol::parse_output_voltage_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 3) {
    ESP_LOGW(EATON_TAG, "Input voltage report too short: %zu bytes", report.data.size());
    return;
  }

  // Path: UPS.PowerConverter.Output.Voltage, Type: Feature, ReportID: 0x0e, Offset: 0, Size: 16
  // NUT debug: Report 0x0e, Value: 231 (matches our 0x00E6 = 230)
  // Data format: [ID, volt_low, volt_high] - 16-bit little endian
  uint16_t voltage_raw = report.data[1] | (report.data[2] << 8);
  // Input voltage is in volts directly, no scaling needed (unlike battery voltage)
  data.power.output_voltage = static_cast<float>(voltage_raw);

  ESP_LOGD(EATON_TAG, "Output voltage: %.1fV (raw: 0x%02X%02X = %d)",
           data.power.output_voltage, report.data[2], report.data[1], voltage_raw);
}

void EatonProtocol::parse_battery_system_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 8) {
    ESP_LOGW(EATON_TAG, "Load percentage report too short: %zu bytes", report.data.size());
    return;
  }

  // Path: UPS.BatterySystem.Charger.Status, Type: Feature, ReportID: 0x07, Offset: 0, Size: 8
  // Path: UPS.OutletSystem.Outlet.[1].Status, Type: Feature, ReportID: 0x07, Offset: 8, Size: 8
  // Path: UPS.OutletSystem.Outlet.[2].Status, Type: Feature, ReportID: 0x07, Offset: 16, Size: 8
  // Path: UPS.PowerSummary.OverallAlarm.Code, Type: Feature, ReportID: 0x07, Offset: 24, Size: 8
  // Path: UPS.PowerSummary.Mode, Type: Feature, ReportID: 0x07, Offset: 32, Size: 8
  // Path: UPS.PowerSummary.PercentLoad, Type: Feature, ReportID: 0x07, Offset: 40, Size: 8
  // Path: UPS.PowerSummary.Status, Type: Feature, ReportID: 0x07, Offset: 48, Size: 8

  ESP_LOGD(EATON_TAG, "UPS.BatterySystem.Charger.Status: 0x%02X", report.data[1]);
  ESP_LOGD(EATON_TAG, "UPS.OutletSystem.Outlet.[1].Status: 0x%02X", report.data[2]);
  ESP_LOGD(EATON_TAG, "UPS.OutletSystem.Outlet.[2].Status: 0x%02X", report.data[3]);
  ESP_LOGD(EATON_TAG, "UPS.PowerSummary.OverallAlarm.Code: 0x%02X", report.data[4]);
  ESP_LOGD(EATON_TAG, "UPS.PowerSummary.Mode: 0x%02X", report.data[5]);
  ESP_LOGD(EATON_TAG, "PowerSummary.PercentLoad: 0x%02X", report.data[6]);
  ESP_LOGD(EATON_TAG, "UPS.PowerSummary.Status: 0x%02X", report.data[7]);

  // NUT debug: Report 0x07, Value: 6 (our raw: 0x07 = 7%)
  // Data format: [ID, load%] - single byte
  uint8_t load_percent = report.data[6];
  data.power.load_percent = static_cast<float>(load_percent);

  ESP_LOGD(EATON_TAG, "Load: %.0f%% (raw: 0x%02X = %d)",
           data.power.load_percent, load_percent, load_percent);
}

void EatonProtocol::check_battery_voltage_scaling(float battery_voltage, float nominal_voltage) {
  if (battery_scale_checked_) {
    return;
  }

  // NUT implements scaling check: if voltage > 1.4 * nominal, apply 2/3 scaling
  const float sanity_ratio = 1.4f;

  if (battery_voltage > (nominal_voltage * sanity_ratio)) {
    ESP_LOGI(EATON_TAG, "Battery voltage %.1fV exceeds %.1fV * %.1f, applying 2/3 scaling",
             battery_voltage, nominal_voltage, sanity_ratio);
    battery_voltage_scale_ = 2.0f / 3.0f;
  } else {
    ESP_LOGD(EATON_TAG, "Battery voltage %.1fV is within normal range, no scaling needed",
             battery_voltage);
    battery_voltage_scale_ = 1.0f;
  }

  battery_scale_checked_ = true;
}

/*
void EatonProtocol::parse_battery_voltage_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 2) {
    ESP_LOGW(EATON_TAG, "Battery voltage nominal report too short: %zu bytes", report.data.size());
    return;
  }

  ESP_LOGD(EATON_TAG, "Battery voltage nominal size: %zu", report.data.size());

  // NUT debug shows: Report 0x09, Value: 24 (ConfigVoltage)
  // Some Eaton models report in decivolts (240 = 24.0V)
  uint8_t voltage_raw = report.data[1];
  data.battery.voltage_nominal = static_cast<float>(voltage_raw) / battery::VOLTAGE_SCALE_FACTOR;

  ESP_LOGD(EATON_TAG, "Battery voltage nominal: %.0fV (raw: 0x%02X = %d)",
           data.battery.voltage_nominal, voltage_raw, voltage_raw);
}
*/

void EatonProtocol::parse_beeper_status_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 2) {
    ESP_LOGW(EATON_TAG, "Beeper status report too short: %zu bytes", report.data.size());
    return;
  }

  // Path: UPS.PowerSummary.AudibleAlarmControl, Type: Feature, ReportID: 0x1f, Offset: 0, Size: 8
  // NUT debug shows: Report 0x0c, Value: 2 (AudibleAlarmControl)
  uint8_t beeper_raw = report.data[1];

  // Map NUT values: 1=disabled, 2=enabled, 3=muted
  switch (beeper_raw) {
    case beeper::CONTROL_DISABLE:
      data.config.beeper_status = beeper::ACTION_DISABLE;
      break;
    case beeper::CONTROL_ENABLE:
      data.config.beeper_status = "enabled";
      break;
    case beeper::CONTROL_MUTE:
      data.config.beeper_status = beeper::ACTION_MUTE;
      break;
    default:
      data.config.beeper_status = sensitivity::UNKNOWN;
      break;
  }

  ESP_LOGD(EATON_TAG, "Beeper status: %s (raw: 0x%02X = %d)",
           data.config.beeper_status.c_str(), beeper_raw, beeper_raw);
}

void EatonProtocol::parse_config_voltage_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 2) {
    ESP_LOGW(EATON_TAG, "Config voltage report too short: %zu bytes", report.data.size());
    return;
  }

  ESP_LOGD(EATON_TAG, "Config voltage size: %zu", report.data.size());

  // Path: UPS.Flow.[4].ConfigVoltage, Type: Feature, ReportID: 0x12, Offset: 0, Size: 8

  // NUT debug shows: Report 0x0e, Value: 230 (ConfigVoltage)
  uint8_t voltage_raw = report.data[1];
  data.power.output_voltage_nominal = static_cast<float>(voltage_raw);

  ESP_LOGD(EATON_TAG, "Output voltage nominal: %.0fV (raw: 0x%02X = %d)",
           data.power.output_voltage_nominal, voltage_raw, voltage_raw);
}

void EatonProtocol::parse_input_transfer_high_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 3) {
    ESP_LOGW(EATON_TAG, "Input transfer high report too short: %zu bytes", report.data.size());
    return;
  }

  // NUT debug shows: Report 0x13
  // Path: UPS.PowerConverter.Output.HighVoltageTransfer, Type: Feature, ReportID: 0x13, Offset: 0, Size: 16
  // Offset 0, Size 16: HighVoltageTransfer = 260
  uint16_t high_transfer = report.data[1] | (report.data[2] << 8);

  data.power.input_transfer_high = static_cast<float>(high_transfer);

  ESP_LOGD(EATON_TAG, "Input transfer limits: High=%.0fV",
           data.power.input_transfer_high);
}


void EatonProtocol::parse_input_transfer_low_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 2) {
    ESP_LOGW(EATON_TAG, "Input transfer low report too short: %zu bytes", report.data.size());
    return;
  }

  // NUT debug shows: Report 0x14
  // Path: UPS.PowerConverter.Output.LowVoltageTransfer, Type: Feature, ReportID: 0x14, Offset: 0, Size: 8
  // Offset 0, Size 8: LowVoltageTransfer = 170
  uint16_t low_transfer = report.data[1];

  data.power.input_transfer_low = static_cast<float>(low_transfer);

  ESP_LOGD(EATON_TAG, "Input transfer limits: Low=%.0fV",
           data.power.input_transfer_low);
}


void EatonProtocol::parse_delay_shutdown_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 5) {
    ESP_LOGW(EATON_TAG, "Delay shutdown report too short: %zu bytes", report.data.size());
    return;
  }

  ESP_LOG_BUFFER_HEXDUMP(EATON_TAG, report.data.data(), sizeof(report.data), ESP_LOG_DEBUG);

  // Path: UPS.PowerSummary.DelayBeforeShutdown, Type: Feature, ReportID: 0x09, Offset: 0, Size: 32

  // NUT debug: ReportID: 0x09, Value: -1 (but NUT output shows ups.delay.shutdown: 20)
  // This suggests NUT does additional processing/conversion
  // Data format: [ID, delay_low, delay_high] - 16-bit signed little endian
  int32_t delay_raw = static_cast<int32_t>(report.data[1] | (report.data[2] << 8 ) | (report.data[3] << 16 ) | (report.data[4] << 24 ));

  if (delay_raw == -1) {
    // When disabled, use NUT default for Eaton (DEFAULT_OFFDELAY = 20)
    data.config.delay_shutdown = defaults::EATON_SHUTDOWN_DELAY_SEC;
    ESP_LOGD(EATON_TAG, "UPS delay shutdown: %d seconds (default, raw: %d)", defaults::EATON_SHUTDOWN_DELAY_SEC, delay_raw);
  } else {
    data.config.delay_shutdown = delay_raw;
    ESP_LOGI(EATON_TAG, "UPS delay shutdown: %d seconds (raw: %d)", data.config.delay_shutdown, delay_raw);
  }
}

void EatonProtocol::parse_delay_start_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 5) {
    ESP_LOGW(EATON_TAG, "Delay start report too short: %zu bytes", report.data.size());
    return;
  }

  ESP_LOG_BUFFER_HEXDUMP(EATON_TAG, report.data.data(), sizeof(report.data), ESP_LOG_DEBUG);

  // Path: UPS.PowerSummary.DelayBeforeStartup, Type: Feature, ReportID: 0x0a, Offset: 0, Size: 32

  // NUT debug: ReportID: 0x0a, Value: -1 (but NUT output shows ups.delay.start: 30)
  // This suggests NUT does additional processing/conversion
  // Data format: [ID, delay_low, delay_high] - 16-bit signed little endian
  //int16_t delay_raw = static_cast<int16_t>(report.data[1] | (report.data[2] << 8));
  int32_t delay_raw = static_cast<int32_t>(report.data[1] | (report.data[2] << 8 ) | (report.data[3] << 16 ) | (report.data[4] << 24 ));

  if (delay_raw == -1) {
    // When disabled, use NUT default for Eaton (DEFAULT_ONDELAY = 30)
    data.config.delay_start = defaults::EATON_STARTUP_DELAY_SEC;
    ESP_LOGD(EATON_TAG, "UPS delay start: %d seconds (default, raw: %d)", defaults::EATON_STARTUP_DELAY_SEC, delay_raw);
  } else {
    data.config.delay_start = delay_raw;
    ESP_LOGI(EATON_TAG, "UPS delay start: %d seconds (raw: %d)", data.config.delay_start, delay_raw);
  }
}

/*void EatonProtocol::parse_realpower_nominal_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 3) {
    ESP_LOGW(EATON_TAG, "Real power nominal report too short: %zu bytes", report.data.size());
    return;
  }

  // NUT debug shows: Report 0x18, Value: 900 (ConfigActivePower)
  uint16_t power_raw = report.data[1] | (report.data[2] << 8);
  data.power.realpower_nominal = static_cast<float>(power_raw);

  ESP_LOGD(EATON_TAG, "UPS nominal real power: %.0fW", data.power.realpower_nominal);
}

void EatonProtocol::parse_input_sensitivity_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 2) {
    ESP_LOGW(EATON_TAG, "Input sensitivity report too short: %zu bytes", report.data.size());
    return;
  }

  // NUT debug shows: Report 0x1a, Value: 1 (CPSInputSensitivity)
  uint8_t sensitivity_raw = report.data[1];
  ESP_LOGD(EATON_TAG, "Raw Eaton sensitivity from report 0x1a: 0x%02X (%d)", sensitivity_raw, sensitivity_raw);

  // DYNAMIC SENSITIVITY MAPPING: Handle known Eaton values with intelligent fallbacks
  switch (sensitivity_raw) {
    case 0:
      data.config.input_sensitivity = sensitivity::HIGH;
      ESP_LOGI(EATON_TAG, "Eaton input sensitivity: high (raw: %d)", sensitivity_raw);
      break;
    case 1:
      data.config.input_sensitivity = sensitivity::NORMAL;
      ESP_LOGI(EATON_TAG, "Eaton input sensitivity: normal (raw: %d)", sensitivity_raw);
      break;
    case 2:
      data.config.input_sensitivity = sensitivity::LOW;
      ESP_LOGI(EATON_TAG, "Eaton input sensitivity: low (raw: %d)", sensitivity_raw);
      break;
    default:
      // DYNAMIC HANDLING: For unknown values, provide better context
      if (sensitivity_raw >= 100) {
        // Large values might indicate report format issue
        ESP_LOGW(EATON_TAG, "Unexpected large Eaton sensitivity value: %d (0x%02X) - possible report format issue",
                 sensitivity_raw, sensitivity_raw);

        // Try alternative parsing - some models might use different byte
        if (report.data.size() >= 3) {
          uint8_t alt_value = report.data[2];
          ESP_LOGD(EATON_TAG, "Trying alternative sensitivity parsing from byte[2]: %d", alt_value);

          if (alt_value <= 2) {
            sensitivity_raw = alt_value;
            switch (sensitivity_raw) {
              case 0: data.config.input_sensitivity = sensitivity::HIGH; break;
              case 1: data.config.input_sensitivity = sensitivity::NORMAL; break;
              case 2: data.config.input_sensitivity = sensitivity::LOW; break;
            }
            ESP_LOGI(EATON_TAG, "Eaton input sensitivity (alt parsing): %s (raw: %d)",
                     data.config.input_sensitivity.c_str(), sensitivity_raw);
            return;
          }
        }

        // Fallback for problematic values
        data.config.input_sensitivity = sensitivity::NORMAL;
        ESP_LOGW(EATON_TAG, "Using default 'normal' sensitivity due to unexpected value: %d", sensitivity_raw);
      } else {
        // Values 3-99 - extend mapping for future Eaton models
        if (sensitivity_raw == 3) {
          data.config.input_sensitivity = sensitivity::AUTO;
          ESP_LOGI(EATON_TAG, "Eaton input sensitivity: auto (raw: %d)", sensitivity_raw);
        } else {
          data.config.input_sensitivity = sensitivity::UNKNOWN;
          ESP_LOGW(EATON_TAG, "Unknown Eaton sensitivity value: %d - please report this for future support",
                   sensitivity_raw);
        }
      }
      break;
  }
}
*/


/*
void EatonProtocol::parse_overload_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 2) {
    ESP_LOGW(EATON_TAG, "Overload report too short: %zu bytes", report.data.size());
    return;
  }

  // NUT debug shows: Report 0x17, Offset: 1, Value: 0 (UPS.Output.Overload)
  // Offset 1 means it's in the second byte of the report
  uint8_t overload_byte = report.data[1];
  bool overload = (overload_byte & 0x01) != 0;  // Check bit 0 (Offset 1 in NUT = bit 0)

  if (overload) {
    data.power.status += " - Overload";
    ESP_LOGW(EATON_TAG, "Eaton UPS OVERLOAD detected (raw: 0x%02X)", overload_byte);
  } else {
    // Remove overload from status if it was there
    size_t pos = data.power.status.find(" - Overload");
    if (pos != std::string::npos) {
      data.power.status.erase(pos, 11);  // Length of " - Overload"
    }
    ESP_LOGD(EATON_TAG, "Eaton UPS overload status: normal (raw: 0x%02X)", overload_byte);
  }
}
*/
void EatonProtocol::read_missing_dynamic_values(UpsData &data) {
  ESP_LOGD(EATON_TAG, "Reading Eaton missing dynamic values from NUT analysis...");



  // 3. Battery runtime low threshold is already available in existing Report 0x08
  // It's at offset 24 and already parsed in parse_battery_runtime_report
  // Just need to extract it properly

  // 4. Try to read manufacturing date (based on NUT: UPS.PowerSummary.iOEMInformation)
  // Eaton manufacturing date might be in reports 0x04, 0x05, or similar to APC reports
  /*std::vector<uint8_t> mfr_date_reports = {0x04, 0x05, 0x06, 0x19, 0x1c, 0x1d, 0x1e, 0x1f, 0x20};
  for (uint8_t report_id : mfr_date_reports) {
    HidReport mfr_date_report;
    if (read_hid_report(report_id, mfr_date_report)) {
      parse_manufacturing_date_report(mfr_date_report, data);
      break; // Found manufacturing date, stop trying other reports
    }
  }*/

  // 5. Set static/derived values based on NUT behavior
  data.test.ups_test_result = test::RESULT_NO_TEST;  // Default test result

  // NOTE: battery_status is now properly set based on charging state in parse_present_status_report

  // 5. Timer values represent active countdown (negative when no countdown active)
  // NUT shows: ups.timer.shutdown: -60, ups.timer.start: -60
  data.test.timer_shutdown = -data.config.delay_shutdown;  // Negative indicates no active countdown
  data.test.timer_start = -data.config.delay_start;        // Negative indicates no active countdown
  data.test.timer_reboot = defaults::REBOOT_TIMER_DEFAULT;  // Eaton doesn't have separate reboot timer, use default

  ESP_LOGD(EATON_TAG, "Completed reading Eaton missing dynamic values");
}

void EatonProtocol::parse_battery_capacity_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 2) {
    ESP_LOGW(EATON_TAG, "Battery capacity limits report too short: %zu bytes", report.data.size());
    return;
  }

  // Path: UPS.PowerSummary.RemainingCapacityLimit, Type: Feature, ReportID: 0x08, Offset: 0, Size: 8

  uint8_t remaining_limit = report.data[1]; // Offset 8 bits = byte 0 + 1
  data.battery.charge_low = static_cast<float>(remaining_limit);
  ESP_LOGI(EATON_TAG, "Eaton Battery charge low threshold: %.0f%% (raw: %d)",
            data.battery.charge_low, remaining_limit);

  // Extract full charge capacity (offset 40 = byte 6) - this is NOT used for battery.status
  // FullChargeCapacity represents maximum capacity (100%), not current status
  /*if (report.data.size() > 6) {
    uint8_t full_charge_capacity = report.data[6]; // Offset 40 bits = byte 5 + 1
    ESP_LOGD(EATON_TAG, "Eaton FullChargeCapacity: %d%% (always 100%% for healthy battery)", full_charge_capacity);
    // Note: battery_status is now set from charging state in parse_present_status_report
  }*/
}

bool EatonProtocol::beeper_enable() {
  ESP_LOGD(EATON_TAG, "Sending Eaton beeper enable command");

  // Eaton DEVICE SPECIFIC: From NUT debug, device uses report ID 0x0c
  // NUT shows: "UPS.PowerSummary.AudibleAlarmControl, Type: Feature, ReportID: 0x0c"
  ESP_LOGD(EATON_TAG, "Trying beeper enable with report ID 0x%02X", BEEPER_STATUS_REPORT_ID);

  uint8_t beeper_data[2] = {BEEPER_STATUS_REPORT_ID, beeper::CONTROL_ENABLE};  // Report ID, Value=2 (enabled)

  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE, BEEPER_STATUS_REPORT_ID, beeper_data, sizeof(beeper_data), parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(EATON_TAG, "Eaton beeper enabled successfully with report ID 0x%02X", BEEPER_STATUS_REPORT_ID);
    return true;
  } else {
    ESP_LOGW(EATON_TAG, "Failed to enable Eaton beeper with report ID 0x%02X: %s", BEEPER_STATUS_REPORT_ID, esp_err_to_name(ret));
    return false;
  }
}

bool EatonProtocol::beeper_disable() {
  ESP_LOGD(EATON_TAG, "Sending Eaton beeper disable command");

  // Eaton DEVICE SPECIFIC: From NUT debug, device uses report ID 0x0c
  ESP_LOGD(EATON_TAG, "Trying beeper disable with report ID 0x%02X", BEEPER_STATUS_REPORT_ID);

  uint8_t beeper_data[2] = {BEEPER_STATUS_REPORT_ID, beeper::CONTROL_DISABLE};  // Report ID, Value=1 (disabled)

  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE, BEEPER_STATUS_REPORT_ID, beeper_data, sizeof(beeper_data), parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(EATON_TAG, "Eaton beeper disabled successfully with report ID 0x%02X", BEEPER_STATUS_REPORT_ID);
    return true;
  } else {
    ESP_LOGW(EATON_TAG, "Failed to disable Eaton beeper with report ID 0x%02X: %s", BEEPER_STATUS_REPORT_ID, esp_err_to_name(ret));
    return false;
  }
}

bool EatonProtocol::beeper_mute() {
  ESP_LOGD(EATON_TAG, "Sending Eaton beeper mute command");

  // MUTE FUNCTIONALITY (Value 3):
  // - Acknowledges and silences current active alarms
  // - Beeper may still sound for new critical events
  // - Different from DISABLE (1) which turns off beeper completely
  uint8_t beeper_data[2] = {BEEPER_STATUS_REPORT_ID, beeper::CONTROL_MUTE};  // Report ID, Value=3 (muted/acknowledged)

  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE, BEEPER_STATUS_REPORT_ID, beeper_data, sizeof(beeper_data), parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(EATON_TAG, "Eaton beeper muted (current alarms acknowledged) successfully");
    return true;
  } else {
    ESP_LOGW(EATON_TAG, "Failed to mute Eaton beeper: %s", esp_err_to_name(ret));
    return false;
  }
}

bool EatonProtocol::beeper_test() {
  ESP_LOGD(EATON_TAG, "Starting Eaton beeper test sequence");

  // First, read current beeper status to restore later
  HidReport current_report;
  if (!read_hid_report(BEEPER_STATUS_REPORT_ID, current_report)) {
    ESP_LOGW(EATON_TAG, "Failed to read current beeper status for test");
    return false;
  }

  uint8_t original_state = (current_report.data.size() >= 2) ? current_report.data[1] : 0x02;
  ESP_LOGI(EATON_TAG, "Original beeper state: %d", original_state);

  // For Eaton beeper test, we need to:
  // 1. Enable beeper first (if disabled)
  // 2. Wait for beeper to sound (longer delay)
  // 3. Disable beeper to stop the test sound
  // 4. Restore original state

  // FOCUS ON ENABLE/DISABLE: Since NUT shows beeper currently "enabled" (value=2)
  // Test by disabling first (may be audible), then re-enabling
  ESP_LOGI(EATON_TAG, "Step 1: Disabling beeper (from current enabled state)");
  if (!beeper_disable()) {
    ESP_LOGW(EATON_TAG, "Failed to disable beeper for test");
    return false;
  }

  ESP_LOGI(EATON_TAG, "Step 2: Waiting 3 seconds with beeper disabled");
  vTaskDelay(pdMS_TO_TICKS(3000));

  ESP_LOGI(EATON_TAG, "Step 3: Re-enabling beeper");
  if (!beeper_enable()) {
    ESP_LOGW(EATON_TAG, "Failed to re-enable beeper");
    // Don't return false - continue to restore original state
  }

  // Brief delay before restoration
  vTaskDelay(pdMS_TO_TICKS(500));

  // Step 4: Restore original beeper state
  ESP_LOGI(EATON_TAG, "Step 4: Restoring original beeper state: %d", original_state);
  uint8_t restore_data[2] = {BEEPER_STATUS_REPORT_ID, original_state};
  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE, BEEPER_STATUS_REPORT_ID, restore_data, sizeof(restore_data), parent_->get_protocol_timeout());

  if (ret == ESP_OK) {
    ESP_LOGI(EATON_TAG, "Eaton beeper test sequence completed successfully");
    return true;
  } else {
    ESP_LOGW(EATON_TAG, "Beeper test completed but failed to restore original state: %s", esp_err_to_name(ret));
    return true; // Test succeeded even if restore failed
  }
}
/*
// Test control methods implementation (based on NUT CPS-HID driver analysis)
bool EatonProtocol::start_battery_test_quick() {
  ESP_LOGI(EATON_TAG, "Initiating quick battery test");

  // Eaton uses UPS.Output.Test path, HID report ID 0x14
  // Quick test command value is 1 (from NUT test_write_info)
  uint8_t test_data[2] = {TEST_RESULT_REPORT_ID, test::COMMAND_QUICK}; // Report ID 0x14, value 1 = Quick test

  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE, TEST_RESULT_REPORT_ID, test_data, 2, parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(EATON_TAG, "Quick battery test command sent successfully");
    return true;
  } else {
    ESP_LOGW(EATON_TAG, "Failed to send quick battery test command: %s", esp_err_to_name(ret));
    return false;
  }
}

bool EatonProtocol::start_battery_test_deep() {
  ESP_LOGI(EATON_TAG, "Initiating deep battery test");

  // Deep test command value is 2 (from NUT test_write_info)
  uint8_t test_data[2] = {TEST_RESULT_REPORT_ID, test::COMMAND_DEEP}; // Report ID 0x14, value 2 = Deep test

  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE, TEST_RESULT_REPORT_ID, test_data, 2, parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(EATON_TAG, "Deep battery test command sent successfully");
    return true;
  } else {
    ESP_LOGW(EATON_TAG, "Failed to send deep battery test command: %s", esp_err_to_name(ret));
    return false;
  }
}

bool EatonProtocol::stop_battery_test() {
  ESP_LOGI(EATON_TAG, "Stopping battery test");

  // Abort test command value is 3 (from NUT test_write_info)
  uint8_t test_data[2] = {TEST_RESULT_REPORT_ID, test::COMMAND_ABORT}; // Report ID 0x14, value 3 = Abort test

  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE, TEST_RESULT_REPORT_ID, test_data, 2, parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(EATON_TAG, "Battery test stop command sent successfully");
    return true;
  } else {
    ESP_LOGW(EATON_TAG, "Failed to send battery test stop command: %s", esp_err_to_name(ret));
    return false;
  }
}

bool EatonProtocol::start_ups_test() {
  // Eaton doesn't seem to have separate UPS test from battery test
  // Redirect to battery test quick as the closest equivalent
  ESP_LOGI(EATON_TAG, "Eaton UPS test redirected to quick battery test");
  return start_battery_test_quick();
}

bool EatonProtocol::stop_ups_test() {
  // Redirect to battery test stop
  ESP_LOGI(EATON_TAG, "Eaton UPS test stop redirected to battery test stop");
  return stop_battery_test();
}

void EatonProtocol::parse_test_result_report(const HidReport &report, UpsData &data) {
  // Parse test result from Report 0x14 (UPS.Output.Test)
  // Based on NUT test_read_info lookup table
  if (report.data.size() < 2) {
    data.test.ups_test_result = test::RESULT_ERROR_READING;
    return;
  }

  // Based on NUT test_read_info lookup table for Eaton:
  // 1 = "Done and passed", 2 = "Done and warning", 3 = "Done and error"
  // 4 = "Aborted", 5 = "In progress", 6 = "No test initiated", 7 = "Test scheduled"
  uint8_t test_result_value = report.data[1];

  ESP_LOGD(EATON_TAG, "Raw test result from report 0x14: 0x%02X (%d)", test_result_value, test_result_value);

  switch (test_result_value) {
    case 1:
      data.test.ups_test_result = test::RESULT_DONE_PASSED;
      break;
    case 2:
      data.test.ups_test_result = test::RESULT_DONE_WARNING;
      break;
    case 3:
      data.test.ups_test_result = test::RESULT_DONE_ERROR;
      break;
    case 4:
      data.test.ups_test_result = test::RESULT_ABORTED;
      break;
    case 5:
      data.test.ups_test_result = test::RESULT_IN_PROGRESS;
      break;
    case 6:
      data.test.ups_test_result = test::RESULT_NO_TEST;
      break;
    case 7:
      data.test.ups_test_result = test::RESULT_SCHEDULED;
      break;
    default:
      data.test.ups_test_result = "Unknown test result (" + std::to_string(test_result_value) + ")";
      ESP_LOGW(EATON_TAG, "Unknown Eaton test result value: %d", test_result_value);
      break;
  }

  ESP_LOGI(EATON_TAG, "Eaton Test result: %s (raw: %d)", data.test.ups_test_result.c_str(), test_result_value);
}
*/
void EatonProtocol::read_frequency_data(UpsData &data) {
  // Initialize frequency to NaN
  data.power.frequency = NAN;

  // Try to read frequency from various HID report IDs
  // Eaton devices may have frequency in input/output measurement reports

  // Report IDs commonly used for frequency measurements:
  const std::vector<uint8_t> frequency_report_ids = {
    0x0d, // EATON
  };

  for (uint8_t report_id : frequency_report_ids) {
    HidReport freq_report;
    if (read_hid_report(report_id, freq_report)) {
      float frequency_value = parse_frequency_from_report(freq_report);
      if (!std::isnan(frequency_value)) {
        data.power.frequency = frequency_value;
        ESP_LOGD(EATON_TAG, "Found frequency %.1f Hz in report 0x%02X", frequency_value, report_id);
        return;
      }
    }
  }

  ESP_LOGV(EATON_TAG, "Frequency data not available from any HID report");
}

float EatonProtocol::parse_frequency_from_report(const HidReport &report) {
  if (report.data.size() < 2) {
    return NAN;
  }

if (report.data.size() >= 4) {
    uint8_t freq_byte = report.data[3];
    if (freq_byte >= FREQUENCY_MIN_VALID && freq_byte <= FREQUENCY_MAX_VALID) {
      return static_cast<float>(freq_byte);
    }
  }

  // Try different byte positions and formats commonly used for frequency
  // Eaton devices may use 0.1 Hz scaling (report 501 becomes 50.1 Hz)
/*
  // Method 1: Single byte at position 1 (common for simple frequency reports)
  if (report.data.size() >= 2) {
    uint8_t freq_byte = report.data[1];
    if (freq_byte >= FREQUENCY_MIN_VALID && freq_byte <= FREQUENCY_MAX_VALID) {
      return static_cast<float>(freq_byte);
    }
  }

  // Method 2: 16-bit little-endian value at position 1-2
  if (report.data.size() >= 3) {
    uint16_t freq_word = report.data[1] | (report.data[2] << 8);
    if (freq_word >= FREQUENCY_MIN_VALID && freq_word <= FREQUENCY_MAX_VALID) {
      return static_cast<float>(freq_word);
    }
  }

  // Method 3: 16-bit big-endian value at position 1-2
  if (report.data.size() >= 3) {
    uint16_t freq_word = (report.data[1] << 8) | report.data[2];
    if (freq_word >= FREQUENCY_MIN_VALID && freq_word <= FREQUENCY_MAX_VALID) {
      return static_cast<float>(freq_word);
    }
  }*/
/*
  // Method 4: Eaton-specific scaled frequency (0.1 factor)
  // Some Eaton models report frequency * 10 (e.g., 500 for 50.0 Hz, 600 for 60.0 Hz)
  if (report.data.size() >= 3) {
    uint16_t freq_scaled = report.data[1] | (report.data[2] << 8);
    float freq_value = static_cast<float>(freq_scaled) / 10.0f;
    if (freq_value >= FREQUENCY_MIN_VALID && freq_value <= FREQUENCY_MAX_VALID) {
      ESP_LOGD(EATON_TAG, "Applied Eaton 0.1x frequency scaling: %d -> %.1f Hz", freq_scaled, freq_value);
      return freq_value;
    }
  }

  // Method 5: Check for hundreds scaling (e.g., 5000 for 50.0 Hz)
  if (report.data.size() >= 3) {
    uint16_t freq_scaled = report.data[1] | (report.data[2] << 8);
    float freq_value = static_cast<float>(freq_scaled) / 100.0f;
    if (freq_value >= FREQUENCY_MIN_VALID && freq_value <= FREQUENCY_MAX_VALID) {
      ESP_LOGD(EATON_TAG, "Applied Eaton 0.01x frequency scaling: %d -> %.1f Hz", freq_scaled, freq_value);
      return freq_value;
    }
  }*/

  return NAN;
}
/*
void EatonProtocol::parse_manufacturing_date_report(const HidReport &report, UpsData &data) {
  if (report.data.size() < 4) {
    ESP_LOGD(EATON_TAG, "Manufacturing date report 0x%02X too short: %zu bytes", report.report_id, report.data.size());
    return;
  }

  // Try different Eaton manufacturing date formats
  // Format 1: Similar to APC - 16-bit date value (could be at different offsets)
  for (size_t offset = 1; offset <= report.data.size() - 2; offset++) {
    uint16_t date_raw = report.data[offset] | (report.data[offset + 1] << 8);
    if (date_raw > 0x0100 && date_raw < 0xFFFF) { // Reasonable date range
      // Try to decode as packed date (MMYY or YYMM format)
      uint8_t byte1 = date_raw & 0xFF;
      uint8_t byte2 = (date_raw >> 8) & 0xFF;

      // Try MMYY format (month/year)
      if (byte1 >= 1 && byte1 <= 12 && byte2 >= 0 && byte2 <= 99) {
        int year = 2000 + byte2; // Assume 2000s
        int month = byte1;
        char date_str[16];
        snprintf(date_str, sizeof(date_str), "%04d/%02d", year, month);

        data.battery.mfr_date = std::string(date_str);
        ESP_LOGI(EATON_TAG, "Eaton Battery manufacturing date: %s (raw: 0x%04X from report 0x%02X)",
                 data.battery.mfr_date.c_str(), date_raw, report.report_id);
        return;
      }

      // Try YYMM format (year/month)
      if (byte2 >= 1 && byte2 <= 12 && byte1 >= 0 && byte1 <= 99) {
        int year = 2000 + byte1; // Assume 2000s
        int month = byte2;
        char date_str[16];
        snprintf(date_str, sizeof(date_str), "%04d/%02d", year, month);

        data.battery.mfr_date = std::string(date_str);
        ESP_LOGI(EATON_TAG, "Eaton Battery manufacturing date: %s (raw: 0x%04X from report 0x%02X)",
                 data.battery.mfr_date.c_str(), date_raw, report.report_id);
        return;
      }
    }
  }

  ESP_LOGD(EATON_TAG, "Could not decode manufacturing date from report 0x%02X (%zu bytes)",
           report.report_id, report.data.size());
}
            */
/*
bool EatonProtocol::read_timer_data(UpsData &data) {
  ESP_LOGD(EATON_TAG, "Reading Eaton timer countdown data");

  HidReport delay_shutdown_report;
  HidReport delay_start_report;
  bool success = false;

  // Read delay shutdown report (timer countdown data)
  if (read_hid_report(DELAY_SHUTDOWN_REPORT_ID, delay_shutdown_report)) {
    // Parse the delay configuration (this updates data.config.delay_shutdown)
    parse_delay_shutdown_report(delay_shutdown_report, data);

    // For Eaton, follow NUT behavior: timer shows negative of delay configuration during normal operation
    // From CP_NUT_debug.md: ups.delay.shutdown: 60, ups.timer.shutdown: -60 (normal operation)
    // During actual shutdown, the timer would show positive countdown values

    // In normal operation, Eaton timers should be negative of the configured delay
    if (data.config.delay_shutdown > 0) {
      // Normal operation - timer is inactive, show negative of configuration value
      data.test.timer_shutdown = -data.config.delay_shutdown;
      ESP_LOGV(EATON_TAG, "Timer shutdown inactive: %d (config: %d)",
               data.test.timer_shutdown, data.config.delay_shutdown);
    } else {
      // Use default if no configuration available
      //data.test.timer_shutdown = -defaults::EATON_SHUTDOWN_DELAY_SEC;
      ESP_LOGV(EATON_TAG, "Timer shutdown inactive (default): %d", data.test.timer_shutdown);
    }

    // TODO: During actual UPS shutdown, we would need to detect the active countdown state
    // This would require monitoring for changing values or specific status indicators
    // For now, we follow NUT's normal operation behavior

    success = true;
  }

  // Read delay start report (timer countdown data)
  if (read_hid_report(DELAY_START_REPORT_ID, delay_start_report)) {
    // Parse the delay configuration (this updates data.config.delay_start)
    parse_delay_start_report(delay_start_report, data);

    // Follow same pattern as shutdown timer
    if (data.config.delay_start > 0) {
      // Normal operation - timer is inactive, show negative of configuration value
      data.test.timer_start = -data.config.delay_start;
      ESP_LOGV(EATON_TAG, "Timer start inactive: %d (config: %d)",
               data.test.timer_start, data.config.delay_start);
    } else {
      // Use default if no configuration available
      //data.test.timer_start = -defaults::EATON_STARTUP_DELAY_SEC;
      ESP_LOGV(EATON_TAG, "Timer start inactive (default): %d", data.test.timer_start);
    }
    success = true;
  }

  // Eaton typically doesn't have a separate reboot timer, use shutdown timer
  data.test.timer_reboot = data.test.timer_shutdown;

  if (success) {
    ESP_LOGD(EATON_TAG, "Eaton timer data updated - shutdown: %d, start: %d, reboot: %d",
             data.test.timer_shutdown, data.test.timer_start, data.test.timer_reboot);
  }

  return success;
}
*/

// Delay configuration methods
bool EatonProtocol::set_shutdown_delay(int seconds) {
  ESP_LOGI(EATON_TAG, "Setting shutdown delay to %d seconds", seconds);

  // Validate range (0-600 seconds = 0-10 minutes), but allow -1 to disable
  if (seconds < -1 || seconds > 600) {
    ESP_LOGW(EATON_TAG, "Shutdown delay %d seconds out of range (-1 to 600)", seconds);
    return false;
  }

  // Check if device supports SET_REPORT operations
  if (!parent_->is_connected()) {
    ESP_LOGW(EATON_TAG, "Cannot set shutdown delay - device not connected");
    return false;
  }

  // Prepare HID SET_REPORT data for shutdown delay
  // Format: Report ID 0x15, 4 bytes little-endian seconds value
  /*uint8_t delay_data[4];
  delay_data[0] = seconds & 0xFF;           // Low byte
  delay_data[1] = (seconds >> 8) & 0xFF;    // High byte
  delay_data[2] = (seconds >> 16) & 0xFF;    // High byte
  delay_data[3] = (seconds >> 24) & 0xFF;    // High byte
*/
  uint8_t delay_data[5] = {
    DELAY_SHUTDOWN_REPORT_ID,
    seconds & 0xFF,
    (seconds >> 8) & 0xFF,
    (seconds >> 16) & 0xFF,
    (seconds >> 24) & 0xFF
  };
/*
  // Restore default FIXME
  uint8_t delay_data[5] = {
    DELAY_SHUTDOWN_REPORT_ID,
    0xFF,
    0xFF,
    0xFF,
    0xFF
  };
*/
  ESP_LOGD(EATON_TAG, "Writing shutdown delay: Report 0x%02X, Value: %d (0x%02X 0x%02X 0x%02X 0x%02X)",
           DELAY_SHUTDOWN_REPORT_ID, seconds, delay_data[1], delay_data[2], delay_data[3], delay_data[4]);

  // Attempt SET_REPORT via control transfer (works even on INPUT-ONLY devices sometimes)
  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE, DELAY_SHUTDOWN_REPORT_ID,
                                         delay_data, sizeof(delay_data), parent_->get_protocol_timeout());

  if (ret == ESP_OK) {
    ESP_LOGI(EATON_TAG, "Shutdown delay set successfully to %d seconds", seconds);
    return true;
  } else if (ret == ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(EATON_TAG, "Device does not support delay configuration (INPUT-ONLY device)");
    return false;
  } else {
    ESP_LOGW(EATON_TAG, "Failed to set shutdown delay: %s", esp_err_to_name(ret));
    return false;
  }
}

bool EatonProtocol::set_start_delay(int seconds) {
  ESP_LOGI(EATON_TAG, "Setting start delay to %d seconds", seconds);

  // Validate range (0-600 seconds = 0-10 minutes), but allow -1 to disable
  if (seconds < -1 || seconds > 600) {
    ESP_LOGW(EATON_TAG, "Start delay %d seconds out of range (-1 to 600)", seconds);
    return false;
  }

  // Check if device supports SET_REPORT operations
  if (!parent_->is_connected()) {
    ESP_LOGW(EATON_TAG, "Cannot set start delay - device not connected");
    return false;
  }

  // APC uses report 0x40 for reboot/start delay
  // Format: Report ID 0x40, 1 byte seconds value (limited to 255 seconds)
  uint8_t delay_data[1];
  delay_data[0] = std::min(seconds, 255);  // Limit to 255 for single byte

  ESP_LOGD(EATON_TAG, "Writing start delay: Report 0x%02X, Value: %d (0x%02X)",
           DELAY_START_REPORT_ID, delay_data[0], delay_data[0]);

  // Attempt SET_REPORT via control transfer
  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE, DELAY_START_REPORT_ID,
                                         delay_data, 1, parent_->get_protocol_timeout());

  if (ret == ESP_OK) {
    ESP_LOGI(EATON_TAG, "Start delay set successfully to %d seconds", delay_data[0]);
    return true;
  } else if (ret == ESP_ERR_NOT_SUPPORTED) {
    ESP_LOGW(EATON_TAG, "Device does not support delay configuration (INPUT-ONLY device)");
    return false;
  } else {
    ESP_LOGW(EATON_TAG, "Failed to set start delay: %s", esp_err_to_name(ret));
    return false;
  }
}

bool EatonProtocol::set_reboot_delay(int seconds) {
  ESP_LOGI(EATON_TAG, "Setting reboot delay to %d seconds", seconds);

  // Eaton typically uses shutdown delay for reboot timing
  // But some models may have a separate reboot delay report
  // For now, we'll set both shutdown and start delays for reboot

  bool shutdown_ok = set_shutdown_delay(seconds);
  bool start_ok = set_start_delay(seconds);

  if (shutdown_ok && start_ok) {
    ESP_LOGI(EATON_TAG, "Reboot delay set successfully to %d seconds", seconds);
    return true;
  } else {
    ESP_LOGW(EATON_TAG, "Failed to set reboot delay completely");
    return false;
  }
}


}  // namespace ups_hid
}  // namespace esphome

// Protocol Factory Self-Registration
#include "protocol_factory.h"

namespace esphome {
namespace ups_hid {

// Creator function for Eaton protocol
std::unique_ptr<UpsProtocolBase> create_eaton_protocol(UpsHidComponent* parent) {
    return std::make_unique<EatonProtocol>(parent);
}

} // namespace ups_hid
} // namespace esphome

// Register Eaton protocol for vendor ID 0x0463
REGISTER_UPS_PROTOCOL_FOR_VENDOR(0x0463, eaton_hid_protocol, esphome::ups_hid::create_eaton_protocol, "Eaton HID Protocol", "Eaton HID protocol with comprehensive sensor support and test functionality", 100);
