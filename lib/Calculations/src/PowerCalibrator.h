#ifndef POWERCALIBRATOR_H
#define POWERCALIBRATOR_H

// Decides what power and cadence Zwift receives, and keeps the trainer's reading aligned with
// a crank power meter. Pure C++ so it can be unit tested on a PC.
//
// Crank-equivalent power = power meter × meterScale (the meter's own calibration correction).
// The trainer measures after the chain, so crank-equivalent ≈ trainer × factor, where the
// factor starts at 1 / (1 − chain loss) and is learned continuously from steady riding
// (both readings fresh, pedalling, flywheel not speeding up or slowing down).
//   - normal riding: Zwift gets the corrected meter
//   - meter missing: Zwift gets trainer × learned factor, so the switch doesn't jump
//   - ERG: the trainer is asked for target ÷ factor, so the *meter* shows the target
//   - learned factor far from the chain-loss baseline: the trainer's calibration has
//     drifted, warn to do a spindown

#include <cstdint>

namespace PowerCal {

enum class Mode : uint8_t {
  TRAINER = 0,      // v1 behaviour: Zwift gets the trainer's raw power (learning still runs)
  POWER_METER = 1   // corrected power meter, trainer × learned factor as fallback
};

enum class Source : uint8_t { TRAINER = 0, POWER_METER = 1, TRAINER_CORRECTED = 2, NONE = 3 };

struct Config {
  Mode mode = Mode::POWER_METER;
  double meterScale = 1.055;       // multiply the meter's reading by this
  double chainLossPercent = 3.0;   // expected loss between crank and trainer
  double driftWarnPercent = 4.0;   // warn when the learned factor is this far from the baseline
  double learnSeconds = 300.0;     // time constant of the learned factor, in qualifying seconds
};

struct Input {
  bool trainerFresh = false;
  double trainerPower = 0;
  double trainerCadence = 0;
  double trainerSpeedKph = 0;
  bool meterFresh = false;
  double meterPower = 0;    // raw, before meterScale
  double meterCadence = 0;
};

struct Output {
  Source source = Source::NONE;
  double power = 0;
  double cadence = 0;
};

class Calibrator {
 public:
  static constexpr uint32_t MIN_LEARNED_SECONDS = 30;     // before this the prior factor is used
  static constexpr uint32_t WARN_AFTER_SECONDS = 120;     // no drift warning before this much evidence
  static constexpr double MIN_POWER = 60.0;
  static constexpr double MIN_CADENCE = 50.0;
  static constexpr double MAX_SPEED_STEP_KPH = 0.54;      // 0.15 m/s per second
  // the first minutes of a ride read differently (11-27: P2M/trainer 1.17 in minute 1, 1.10 in
  // minute 2, ~1.03 after), so no learning until the rider has been moving this long
  static constexpr uint32_t WARMUP_SECONDS = 120;
  static constexpr uint32_t RESTART_AFTER_STOP_SECONDS = 60;  // a stop this long starts a new warm-up
  static constexpr double MOVING_KPH = 5.0;
  static constexpr double STOPPED_KPH = 2.0;
  // the starting factor counts as this many seconds of evidence, so a few odd seconds can't swing it
  static constexpr uint32_t PRIOR_WEIGHT_SECONDS = 120;

  void configure(const Config& config);
  const Config& getConfig() const { return config; }
  // starting point: a factor saved from earlier rides, or the baseline
  void setPriorFactor(double factor);
  // forget what was learned (e.g. after a spindown); the prior becomes the baseline
  void reset();

  // call once per second: learns and returns the output
  Output update(const Input& input);
  // the output for the latest readings without learning; call as often as new data arrives
  Output compute(const Input& input) const;

  double baselineFactor() const;
  double factor() const;          // crank-equivalent ÷ trainer, the one in use
  double driftPercent() const;    // learned factor vs baseline
  bool driftWarning() const { return warning; }
  uint32_t learnedSeconds() const { return count; }
  bool warmingUp() const { return movingSeconds < WARMUP_SECONDS; }
  // trainer target that makes the crank-equivalent power equal crankTarget
  double trainerTargetFor(double crankTarget) const;

 private:
  Config config;
  double prior = 0;  // 0 = baseline
  double numerator = 0;    // EMA of crank-equivalent power
  double denominator = 0;  // EMA of trainer power
  uint32_t count = 0;
  bool warning = false;
  double speeds[6] = {0, 0, 0, 0, 0, 0};
  uint8_t speedCount = 0;
  uint32_t movingSeconds = 0;
  uint32_t stoppedSeconds = 0;
  bool steadySpeed(double speedKph);
};

}  // namespace PowerCal

#endif
