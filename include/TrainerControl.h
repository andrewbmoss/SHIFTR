#ifndef TRAINERCONTROL_H
#define TRAINERCONTROL_H

// TrainerControl owns everything the bridge sends TO the smart trainer and
// everything it learns FROM the trainer (power, cadence, speed).
//
// Callers (Zwift protocol handler, FTMS emulation, buttons) only set the
// *desired* state with setSimulation()/setTargetPower()/setResistance(). The
// actual BLE writes happen in update(), which runs in the main loop. That
// gives us three properties the phone apps usually lack:
//   - one task talks to the trainer, so writes never collide
//   - bursts of Zwift updates are coalesced (only the newest command is sent)
//   - after a reconnect or lost FTMS control the last command is re-sent

#include <Arduino.h>
#include <NimBLEUUID.h>
#include <TrainerProtocol.h>

enum class TrainerProtocolType : uint8_t {
  NONE = 0,
  FTMS = 1,
  FEC = 2
};

enum class TrainerProtocolPreference : uint8_t {
  AUTO = 0,  // FTMS if available, otherwise FE-C
  FTMS = 1,
  FEC = 2
};

class TrainerControl {
 public:
  // Called by BTDeviceManager once services have been discovered.
  static void onConnected(bool hasFTMS, bool hasFEC);
  static void onDisconnected();
  // Called from the main loop.
  static void update();
  // Every BLE notification/indication from the trainer goes through here.
  static void onNotify(const NimBLEUUID& characteristicUUID, const uint8_t* data, size_t length);

  // Desired trainer state
  static void setSimulation(double gradePercent, double crr = TrainerProtocol::DEFAULT_CRR, double cw = TrainerProtocol::DEFAULT_CW);
  static void setTargetPower(double watts);
  static void setResistancePercent(double percent);
  static void setUserConfiguration(double bicycleWeightKg, double userWeightKg, double gearRatio);
  // Forget the current command (used when the trainer must not be driven, e.g. pass-through)
  static void clearCommand();

  // State
  static TrainerProtocolType getProtocol();
  static const char* getProtocolName();
  static bool hasControl();
  static String getLastCommandDescription();

  // Telemetry from the trainer
  static uint16_t getPower();
  static double getSpeedMps();
  static uint8_t getMeasuredCadence();
  // Measured cadence if the trainer reports one, otherwise estimated from wheel speed
  static uint8_t getEffectiveCadence(double physicalGearRatio);
  static bool isCadenceMeasured();
  static uint16_t getMaximumResistance();
  static uint32_t getMillisSinceLastData();
  // packets received per source since connecting, for the status page
  static String getPacketSummary();

 private:
  enum class CommandType : uint8_t { NONE, SIMULATION, TARGET_POWER, RESISTANCE };
  struct Command {
    CommandType type = CommandType::NONE;
    double value1 = 0;  // grade %, watts or resistance %
    double value2 = 0;  // crr
    double value3 = 0;  // cw
  };

  static void setCommand(const Command& command);
  static bool sendCommand(const Command& command);
  static bool sendFTMS(const std::vector<uint8_t>& data);
  static bool sendFEC(const std::vector<uint8_t>& data);
  static void handleControlPointResponse(const uint8_t* data, size_t length);
  static String describe(const Command& command);
  static bool sameCommand(const Command& a, const Command& b);

  static portMUX_TYPE lock;
  static volatile TrainerProtocolType protocol;
  static Command pendingCommand;
  static Command lastSentCommand;
  static volatile bool commandDirty;
  static volatile bool controlGranted;
  static volatile bool controlRequestPending;
  static uint32_t controlRequestedAt;
  static uint32_t lastCommandSentAt;
  static volatile bool userConfigurationDirty;
  static double userBicycleWeightKg;
  static double userWeightKg;
  static double userGearRatio;
  static bool resistanceRangeKnown;
  static int16_t resistanceRangeMinimum;
  static int16_t resistanceRangeMaximum;

  static volatile uint16_t power;
  static volatile uint32_t speedMilliMps;
  static volatile uint8_t cadence;
  static volatile bool cadenceMeasured;
  static volatile uint16_t maximumResistance;
  static volatile uint32_t lastDataAt;
};

#endif
