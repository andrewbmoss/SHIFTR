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

// ---------------------------------------------------------------------------
// FTMS spin down (op 0x13 on the control point, status 0x14 on Fitness Machine Status)
// ---------------------------------------------------------------------------
constexpr uint8_t FTMS_SPIN_DOWN_CONTROL = 0x13;
constexpr uint8_t FTMS_SPIN_DOWN_START = 0x01;
constexpr uint8_t FTMS_SPIN_DOWN_IGNORE = 0x02;
constexpr uint8_t FTMS_STATUS_SPIN_DOWN = 0x14;
constexpr uint8_t FTMS_SPIN_DOWN_STATUS_REQUESTED = 0x01;
constexpr uint8_t FTMS_SPIN_DOWN_STATUS_SUCCESS = 0x02;
constexpr uint8_t FTMS_SPIN_DOWN_STATUS_ERROR = 0x03;
constexpr uint8_t FTMS_SPIN_DOWN_STATUS_STOP_PEDALING = 0x04;
std::vector<uint8_t> ftmsSpinDownControl(uint8_t parameter);
// [0x80, 0x13, result, target speed low (UINT16 0.01 km/h), target speed high] - targets only on success
bool parseSpinDownResponse(const uint8_t* data, size_t length, uint8_t& result, double& lowKph, double& highKph);

// The spin down procedure as seen from the client: start -> trainer answers with a speed range ->
// rider speeds up -> trainer says "stop pedalling" -> rider coasts -> trainer reports success or error.
class SpinDown {
 public:
  enum class State : uint8_t { IDLE = 0, STARTING, SPEED_UP, COAST, SUCCESS, FAILED };
  static constexpr uint32_t ANSWER_TIMEOUT_MS = 5000;
  static constexpr uint32_t PROCEDURE_TIMEOUT_MS = 120000;
  static constexpr uint32_t RESULT_SHOWN_MS = 60000;

  void start(uint32_t now);
  void onResponse(uint8_t result, double lowKph, double highKph, uint32_t now);
  void onStatus(uint8_t status, uint32_t now);
  void tick(uint32_t now);
  void cancel(uint32_t now);
  // procedure running: normal trainer commands must wait
  bool active() const { return (state == State::STARTING) || (state == State::SPEED_UP) || (state == State::COAST); }
  State getState() const { return state; }
  const char* getStateName() const;
  double getTargetLowKph() const { return targetLow; }
  double getTargetHighKph() const { return targetHigh; }
  // fixed text (no allocation: may be called inside a critical section)
  const char* getMessage() const { return message; }
  // set when the trainer itself asked for a spin down (status 0x14 0x01 while idle)
  bool wasRequestedByTrainer() const { return requestedByTrainer; }

 private:
  State state = State::IDLE;
  uint32_t startedAt = 0;
  uint32_t stateAt = 0;
  double targetLow = 0;
  double targetHigh = 0;
  bool requestedByTrainer = false;
  const char* message = "";
  void fail(const char* reason, uint32_t now);
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
  uint16_t flags = 0;
  int16_t powerWatts = 0;
  bool hasBalance = false;
  double balancePercent = 0;       // share of power from the reference pedal, %
  bool balanceReferenceLeft = false;
  bool hasAccumulatedTorque = false;
  uint16_t accumulatedTorque = 0;  // 1/32 Nm, wraps
  bool torqueFromCrank = false;    // false = wheel based
  bool hasCrank = false;
  uint16_t crankRevolutions = 0;   // cumulative, wraps at 65536
  uint16_t crankEventTime = 0;     // 1/1024 s, wraps
  bool hasWheel = false;
  uint32_t wheelRevolutions = 0;   // cumulative
  uint16_t wheelEventTime = 0;     // 1/2048 s, wraps
  bool hasAccumulatedEnergy = false;
  uint16_t accumulatedEnergyKJ = 0;
  bool offsetCompensationIndicator = false;
};
bool parseCyclingPowerMeasurement(const uint8_t* data, size_t length, CyclingPowerData& result);
// Cycling Power Feature (0x2A65) bits as readable names, e.g. "pedal_balance, crank_revolutions"
std::string describeCyclingPowerFeatures(uint32_t features);
constexpr uint32_t CP_FEATURE_OFFSET_COMPENSATION = 1u << 9;
// Cycling Power Control Point (0x2A66)
constexpr uint8_t CP_OP_START_OFFSET_COMPENSATION = 0x0C;
constexpr uint8_t CP_OP_RESPONSE = 0x20;

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
// Page 50: wind resistance, Cw in 0.01 kg/m, no wind, no drafting
std::vector<uint8_t> fecWindResistance(double cw);
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

// Replace power (and cadence) in a trainer notification before it is forwarded to Zwift.
// Fields that the packet doesn't carry are left alone. Returns false if the packet is too short.
bool rewriteIndoorBikeData(uint8_t* data, size_t length, int16_t powerWatts, double cadenceRpm);
bool rewriteCyclingPowerMeasurement(uint8_t* data, size_t length, int16_t powerWatts);

// Hex dump for diagnostics, e.g. "44 00 28 0A"
std::string toHex(const uint8_t* data, size_t length);

}  // namespace TrainerProtocol

#endif
