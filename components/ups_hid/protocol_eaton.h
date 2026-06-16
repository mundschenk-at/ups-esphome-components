#pragma once

#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

/**
 * @brief Eaton HID Protocol Implementation
 */
class EatonProtocol : public UpsProtocolBase {
 public:
  EatonProtocol(UpsHidComponent *parent) : UpsProtocolBase(parent) {}

  bool detect() override;
  bool initialize() override;
  bool read_data(UpsData &data) override;
  DeviceInfo::DetectedProtocol get_protocol_type() const override { return DeviceInfo::PROTOCOL_EATON_HID; }
  std::string get_protocol_name() const override { return "Eaton HID"; }

  // Beeper control methods
  bool beeper_enable() override;
  bool beeper_disable() override;
  bool beeper_mute() override;
  bool beeper_test() override;

  // Low capacity limit setting methods (should be declared in base method)
  bool set_remaining_capacity_limit(uint8_t percent);

  /*
  // UPS and battery test methods
  bool start_battery_test_quick() override;
  bool start_battery_test_deep() override;
  bool stop_battery_test() override;
  bool start_ups_test() override;
  bool stop_ups_test() override;
*/
  // Timer polling for real-time countdown
  //bool read_timer_data(UpsData &data) override;

  // Delay configuration methods
  bool set_shutdown_delay(int seconds) override;
  bool set_start_delay(int seconds) override;
  bool set_reboot_delay(int seconds) override;

 private:
  // Report ID constants (based on NUT debug logs)
  static const uint8_t PRESENT_STATUS_REPORT_ID = 0x01;          // Status bitmap
  static const uint8_t VOLTAGE_STATUS_REPORT_ID = 0x03;          // Voltage too high/too low
  static const uint8_t BATTERY_RUNTIME_REPORT_ID = 0x06;         // Battery % + Runtime
  static const uint8_t BATTERY_SYSTEM_REPORT_ID = 0x07;          // Load percentage etc.
  static const uint8_t BATTERY_CAPACITY_REPORT_ID = 0x08;        // Battery capacity limits
  static const uint8_t DELAY_SHUTDOWN_REPORT_ID = 0x09;          // Delay before shutdown
  static const uint8_t DELAY_START_REPORT_ID = 0x0a;             // Delay before startup
  static const uint8_t BATTERY_STATIC_CAPACITY_REPORT_ID = 0x0c; // Battery design capacity etc.
  static const uint8_t OUTPUT_VOLTAGE_REPORT_ID = 0x0e;          // Output voltage
  static const uint8_t DEVICE_INFORMATION_REPORT_ID = 0x10;      // General device information
  static const uint8_t CONFIG_VOLTAGE_REPORT_ID = 0x12;          // Output voltage nominal
  static const uint8_t INPUT_TRANSFER_HIGH_REPORT_ID = 0x13;     // Input transfer limits
  static const uint8_t INPUT_TRANSFER_LOW_REPORT_ID = 0x14;      // Input transfer limits
  static const uint8_t BEEPER_STATUS_REPORT_ID = 0x1f;           // Beeper status
  static const uint8_t CAPACITY_LIMIT_SETTING_REPORT_ID = 0x22;  // Remaining capacity limit setting

  // Path: UPS.PowerSummary.PresentStatus.ACPresent, Type: Feature, ReportID: 0x01, Offset: 0, Size: 1
  // Path: UPS.PowerSummary.PresentStatus.BelowRemainingCapacityLimit, Type: Feature, ReportID: 0x01, Offset: 1, Size: 1,
  // Path: UPS.PowerSummary.PresentStatus.Charging, Type: Feature, ReportID: 0x01, Offset: 2, Size: 1
  // Path: UPS.PowerSummary.PresentStatus.CommunicationLost, Type: Feature, ReportID: 0x01, Offset: 3
  // Path: UPS.PowerSummary.PresentStatus.Discharging, Type: Feature, ReportID: 0x01, Offset: 4, Size: 1
  // Path: UPS.PowerSummary.PresentStatus.Good, Type: Feature, ReportID: 0x01, Offset: 5, Size: 1
  // Path: UPS.PowerSummary.PresentStatus.InternalFailure, Type: Feature, ReportID: 0x01, Offset: 6, Size: 1
  // Path: UPS.PowerSummary.PresentStatus.NeedReplacement, Type: Feature, ReportID: 0x01, Offset: 7, Size: 1
  // Path: UPS.PowerSummary.PresentStatus.Overload, Type: Feature, ReportID: 0x01, Offset: 8, Size: 8

  // Path: UPS.OutletSystem.Outlet.[1].PresentStatus.SwitchOn/Off, Type: Feature, ReportID: 0x02, Offset: 0, Size: 8
  // Path: UPS.OutletSystem.Outlet.[2].PresentStatus.SwitchOn/Off, Type: Feature, ReportID: 0x02, Offset: 8, Size: 8

  // Path: UPS.BatterySystem.Charger.PresentStatus.VoltageTooHigh, Type: Feature, ReportID: 0x03, Offset: 0, Size: 8
  // Path: UPS.BatterySystem.Charger.PresentStatus.VoltageTooLow, Type: Feature, ReportID: 0x03, Offset: 8, Size: 8

  // Path: UPS.PowerSummary.RemainingCapacity, Type: Feature, ReportID: 0x06, Offset: 0, Size: 8
  // Path: UPS.PowerSummary.RunTimeToEmpty, Type: Feature, ReportID: 0x06, Offset: 8, Size: 32

  // Path: UPS.BatterySystem.Charger.Status, Type: Feature, ReportID: 0x07, Offset: 0, Size: 8
  // Path: UPS.OutletSystem.Outlet.[1].Status, Type: Feature, ReportID: 0x07, Offset: 8, Size: 8
  // Path: UPS.OutletSystem.Outlet.[2].Status, Type: Feature, ReportID: 0x07, Offset: 16, Size: 8
  // Path: UPS.PowerSummary.OverallAlarm.Code, Type: Feature, ReportID: 0x07, Offset: 24, Size: 8
  // Path: UPS.PowerSummary.Mode, Type: Feature, ReportID: 0x07, Offset: 32, Size: 8
  // Path: UPS.PowerSummary.PercentLoad, Type: Feature, ReportID: 0x07, Offset: 40, Size: 8
  // Path: UPS.PowerSummary.Status, Type: Feature, ReportID: 0x07, Offset: 48, Size: 8

  // Path: UPS.PowerSummary.RemainingCapacityLimit, Type: Feature, ReportID: 0x08, Offset: 0, Size: 8

  // Path: UPS.PowerSummary.DelayBeforeShutdown, Type: Feature, ReportID: 0x09, Offset: 0, Size: 32

  // Path: UPS.PowerSummary.DelayBeforeStartup, Type: Feature, ReportID: 0x0a, Offset: 0, Size: 32

  // Path: UPS.OutletSystem.Outlet.[1].PresentStatus.Switchable, Type: Feature, ReportID: 0x0c, Offset: 0, Size: 8
  // Path: UPS.PowerConverter.ConverterType, Type: Feature, ReportID: 0x0c, Offset: 8, Size: 8
  // Path: UPS.PowerSummary.CapacityGranularity1, Type: Feature, ReportID: 0x0c, Offset: 16, Size: 8
  // Path: UPS.PowerSummary.CapacityMode, Type: Feature, ReportID: 0x0c, Offset: 24, Size: 8
  // Path: UPS.PowerSummary.DesignCapacity, Type: Feature, ReportID: 0x0c, Offset: 32, Size: 8
  // Path: UPS.PowerSummary.FullChargeCapacity, Type: Feature, ReportID: 0x0c, Offset: 40, Size: 8
  // Path: UPS.PowerSummary.ffff00e2, Type: Feature, ReportID: 0x0c, Offset: 48, Size: 8

  // Path: UPS.PowerConverter.Output.Voltage, Type: Feature, ReportID: 0x0e, Offset: 0, Size: 16

  // Path: UPS.PowerSummary.iDeviceChemistry, Type: Feature, ReportID: 0x10, Offset: 0, Size: 8
  // Path: UPS.PowerSummary.iManufacturer, Type: Feature, ReportID: 0x10, Offset: 8, Size: 8
  // Path: UPS.PowerSummary.iModel, Type: Feature, ReportID: 0x10, Offset: 16, Size: 8
  // Path: UPS.PowerSummary.iProduct, Type: Feature, ReportID: 0x10, Offset: 24, Size: 8
  // Path: UPS.PowerSummary.iReferenceNumber, Type: Feature, ReportID: 0x10, Offset: 32, Size: 8
  // Path: UPS.PowerSummary.iSerialNumber, Type: Feature, ReportID: 0x10, Offset: 40, Size: 8
  // Path: UPS.PowerSummary.iVersion, Type: Feature, ReportID: 0x10, Offset: 48, Size: 8

  // Path: UPS.Flow.[4].ConfigVoltage, Type: Feature, ReportID: 0x12, Offset: 0, Size: 8

  // Path: UPS.PowerConverter.Output.HighVoltageTransfer, Type: Feature, ReportID: 0x13, Offset: 0, Size: 16

  // Path: UPS.PowerConverter.Output.LowVoltageTransfer, Type: Feature, ReportID: 0x14, Offset: 0, Size: 8

  // Path: UPS.PowerSummary.Country, Type: Feature, ReportID: 0x15, Offset: 0, Size: 8

  // Path: UPS.HistorySystem.Event.[1].EventID, Type: Feature, ReportID: 0x17, Offset: 0, Size: 8
  // Path: UPS.HistorySystem.Event.[2].EventID, Type: Feature, ReportID: 0x17, Offset: 8, Size: 8
  // Path: UPS.HistorySystem.Event.[3].EventID, Type: Feature, ReportID: 0x17, Offset: 16, Size: 8
  // Path: UPS.HistorySystem.Event.[4].EventID, Type: Feature, ReportID: 0x17, Offset: 24, Size: 8
  // Path: UPS.HistorySystem.Event.[5].EventID, Type: Feature, ReportID: 0x17, Offset: 32, Size: 8

  // Path: UPS.HistorySystem.Event.[1].Code, Type: Feature, ReportID: 0x18, Offset: 0, Size: 16
  // Path: UPS.HistorySystem.Event.[2].Code, Type: Feature, ReportID: 0x18, Offset: 16, Size: 16
  // Path: UPS.HistorySystem.Event.[3].Code, Type: Feature, ReportID: 0x18, Offset: 32, Size: 16
  // Path: UPS.HistorySystem.Event.[4].Code, Type: Feature, ReportID: 0x18, Offset: 48, Size: 16
  // Path: UPS.HistorySystem.Event.[5].Code, Type: Feature, ReportID: 0x18, Offset: 64, Size: 16

  // Path: UPS.OutletSystem.Outlet.[2].PresentStatus.Switchable, Type: Feature, ReportID: 0x19, Offset: 0, Size: 8

  // Path: UPS.PowerSummary.AudibleAlarmControl, Type: Feature, ReportID: 0x1f, Offset: 0, Size: 8

  // Path: UPS.BatterySystem.Battery.AudibleAlarmControl, Type: Feature, ReportID: 0x20, Offset: 0, Size: 8

  // Path: UPS.PowerSummary.RemainingCapacityLimitSetting, Type: Feature, ReportID: 0x22, Offset: 0, Size: 8

  // Path: UPS.System.USB.Mode, Type: Feature, ReportID: 0xfd, Offset: 0, Size: 8

  // HID Report structure
  struct HidReport {
    uint8_t report_id;
    std::vector<uint8_t> data;

    HidReport() : report_id(0) {}
  };

  // Eaton-specific scaling factors
  float battery_voltage_scale_ = 1.0f;
  bool battery_scale_checked_ = false;

  // HID communication methods
  bool read_hid_report(uint8_t report_id, HidReport &report);

  // Parser methods for different reports
  void parse_present_status_report(const HidReport &report, UpsData &data);
  void parse_voltage_status_report(const HidReport &report, UpsData &data);
  void parse_battery_runtime_report(const HidReport &report, UpsData &data);
  void parse_battery_system_report(const HidReport &report, UpsData &data);
  void parse_battery_capacity_report(const HidReport &report, UpsData &data);
  void parse_delay_shutdown_report(const HidReport &report, UpsData &data);
  void parse_delay_start_report(const HidReport &report, UpsData &data);
  void parse_battery_static_capacity_report(const HidReport &report, UpsData &data);
  void parse_output_voltage_report(const HidReport &report, UpsData &data);
  void parse_device_information_report(const HidReport &repor, UpsData &data);
  void parse_config_voltage_report(const HidReport &report, UpsData &data);
  void parse_input_transfer_high_report(const HidReport &report, UpsData &data);
  void parse_input_transfer_low_report(const HidReport &report, UpsData &data);
  void parse_beeper_status_report(const HidReport &report, UpsData &data);

  // Missing dynamic values from NUT analysis
  void read_missing_dynamic_values(UpsData &data);

  // Retrieve device information strings
  bool read_usb_descriptor(uint8_t index, std::string &string, const std::string &info);

  // Eaton-specific scaling logic
  void check_battery_voltage_scaling(float battery_voltage, float nominal_voltage);

  // Frequency reading methods
  void read_frequency_data(UpsData &data);
  float parse_frequency_from_report(const HidReport &report);
};

}  // namespace ups_hid
}  // namespace esphome