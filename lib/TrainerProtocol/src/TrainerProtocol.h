#ifndef TRAINERPROTOCOL_H
#define TRAINERPROTOCOL_H

// Pure encoders/decoders for the two trainer control protocols the bridge can
// speak to a smart trainer over BLE:
//   - Bluetooth FTMS (Fitness Machine Service 0x1826)
//   - ANT+ FE-C tunnelled over BLE (Tacx-style service 6E40FEC1-...)
//
// Nothing in here depends on Arduino or NimBLE so it can be unit tested on a PC.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace TrainerProtocol {

// ---------------------------------------------------------------------------
// Physical defaults (same model the trainers use for their own SIM mode)
// ---------------------------------------------------------------------------
constexpr double DEFAULT_CRR = 0.00415;  // asphalt road (FE-C default, Zwift sends ~0.004)
constexpr double DEFAULT_CW = 0.51;      // kg/m, road bike / touring position

// Grades outside this range are clamped before they are sent to the trainer.
constexpr double MIN_GRADE_PERCENT = -40.0;
constexpr double MAX_GRADE_PERCENT = 40.0;

// ---------------------------------------------------------------------------
// FTMS
// ---------------------------------------------------------------------------
enum FTMSOpCode : uint8_t {
  FTMS_REQUEST_CONTROL = 0x00,
  FTMS_RESET = 0x01,
  FTMS_SET_TARGET_RESISTANCE_LEVEL = 0x04,
  FTMS_SET_TARGET_POWER = 0x05,
  FTMS_START_OR_RESUME = 0x07,
  FTMS_STOP_OR_PAUSE = 0x08,
  FTMS_SET_INDOOR_BIKE_SIMULATION = 0x11,
  FTMS_RESPONSE_CODE = 0x80
};

enum FTMSResultCode : uint8_t {
  FTMS_RESULT_SUCCESS = 0x01,
  FTMS_RESULT_OP_CODE_NOT_SUPPORTED = 0x02,
  FTMS_RESULT_INVALID_PARAMETER = 0x03,
  FTMS_RESULT_OPERATION_FAILED = 0x04,
  FTMS_RESULT_CONTROL_NOT_PERMITTED = 0x05
};

// Fitness Machine Status op code meaning "control permission lost"
constexpr uint8_t FTMS_STATUS_RESET = 0x01;
constexpr uint8_t FTMS_STATUS_CONTROL_PERMISSION_LOST = 0xFF;

std::vector<uint8_t> ftmsRequestControl();
std::vector<uint8_t> ftmsStartOrResume();
std::vector<uint8_t> ftmsReset();
// Set Target Power (0x05): SINT16, 1 W resolution
std::vector<uint8_t> ftmsTargetPower(double watts);
// Set Indoor Bike Simulation Parameters (0x11)
//   wind speed SINT16 0.001 m/s, grade SINT16 0.01 %, Crr UINT8 0.0001, Cw UINT8 0.01 kg/m
std::vector<uint8_t> ftmsIndoorBikeSimulation(double gradePercent, double crr = DEFAULT_CRR, double cw = DEFAULT_CW, double windSpeed = 0.0);
// Set Target Resistance Level (0x04): UINT8 with 0.1 resolution (FTMS v1.0).
// percent (0..100) is mapped onto the trainer's Supported Resistance Level Range if known
// (minRaw/maxRaw in 0.1 units as read from characteristic 0x2AD6), otherwise sent as 0..100.
std::vector<uint8_t> ftmsTargetResistanceLevel(double percent, bool rangeKnown = false, int16_t minRaw = 0, int16_t maxRaw = 0);

struct IndoorBikeData {
  bool hasSpeed = false;
  bool hasCadence = false;
  bool hasPower = false;
  double speedKph = 0.0;
  double cadenceRpm = 0.0;
  int16_t powerWatts = 0;
};
// Parses an Indoor Bike Data (0x2AD2) notification. Only the fields flagged as present are set.
bool parseIndoorBikeData(const uint8_t* data, size_t length, IndoorBikeData& result);

struct ControlPointResponse {
  uint8_t requestOpCode = 0;
  uint8_t resultCode = 0;
};
// Parses a Fitness Machine Control Point (0x2AD9) indication: [0x80, request op code, result]
bool parseControlPointResponse(const uint8_t* data, size_t length, ControlPointResponse& result);

struct ResistanceLevelRange {
  int16_t minimum = 0;    // 0.1 units
  int16_t maximum = 0;    // 0.1 units
  uint16_t increment = 0; // 0.1 units
};
// Parses Supported Resistance Level Range (0x2AD6)
bool parseResistanceLevelRange(const uint8_t* data, size_t length, ResistanceLevelRange& result);

// ---------------------------------------------------------------------------
// Cycling Power Service (0x1818), Cycling Power Measurement (0x2A63)
// ---------------------------------------------------------------------------
struct CyclingPowerData {
  int16_t powerWatts = 0;
  bool hasCrank = false;
  uint16_t crankRevolutions = 0;   // cumulative, wraps at 65536
  uint16_t crankEventTime = 0;     // 1/1024 s, wraps
  bool hasWheel = false;
  uint32_t wheelRevolutions = 0;   // cumulative
  uint16_t wheelEventTime = 0;     // 1/2048 s, wraps
};
bool parseCyclingPowerMeasurement(const uint8_t* data, size_t length, CyclingPowerData& result);

// Turns cumulative crank revolutions + event times into RPM.
// Holds the last value between events and drops to 0 when no new revolution arrives for a while.
class CrankCadence {
 public:
  explicit CrankCadence(uint32_t timeoutMs = 3000) : timeoutMs(timeoutMs) {}
  // returns the current cadence in RPM
  double update(uint16_t revolutions, uint16_t eventTime1024, uint32_t nowMs);
  double current(uint32_t nowMs) const;
  void reset();

 private:
  uint32_t timeoutMs;
  bool initialised = false;
  uint16_t lastRevolutions = 0;
  uint16_t lastEventTime = 0;
  uint32_t lastChangeMs = 0;
  double cadence = 0;
};

// ---------------------------------------------------------------------------
// FE-C over BLE
// ---------------------------------------------------------------------------
uint8_t fecChecksum(const std::vector<uint8_t>& message);
// Page 49: target power in 0.25 W
std::vector<uint8_t> fecTargetPower(double watts);
// Page 51: track resistance, grade in 0.01 % offset by -200 %, Crr in 5e-5
std::vector<uint8_t> fecTrackResistance(double gradePercent, double crr = DEFAULT_CRR);
// Page 48: basic resistance in 0.5 % steps (0..200)
std::vector<uint8_t> fecBasicResistance(double percent);
// Page 55: user configuration
std::vector<uint8_t> fecUserConfiguration(double bicycleWeightKg, double userWeightKg, double wheelDiameterMeters, double gearRatio);
// Page 70: request page 54 (capabilities)
std::vector<uint8_t> fecCapabilitiesRequest();

struct FECData {
  uint8_t page = 0;
  bool hasSpeed = false;
  bool hasCadence = false;
  bool hasPower = false;
  bool hasMaximumResistance = false;
  double speedMps = 0.0;
  uint8_t cadenceRpm = 0;
  uint16_t powerWatts = 0;
  uint16_t maximumResistanceNewton = 0;
};
// Parses a 13 byte FE-C over BLE message as notified on 6E40FEC2-...
bool parseFECMessage(const uint8_t* data, size_t length, FECData& result);

// Hex dump for diagnostics, e.g. "44 00 28 0A"
std::string toHex(const uint8_t* data, size_t length);

}  // namespace TrainerProtocol

#endif
