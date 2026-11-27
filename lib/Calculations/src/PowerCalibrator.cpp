#include <PowerCalibrator.h>

#include <cmath>

namespace PowerCal {

namespace {
double clampFactor(double value) {
  if (std::isnan(value) || value <= 0) return 1.0;
  if (value < 0.8) return 0.8;
  if (value > 1.25) return 1.25;
  return value;
}
}  // namespace

void Calibrator::configure(const Config& newConfig) {
  bool baselineChanged = newConfig.chainLossPercent != config.chainLossPercent;
  config = newConfig;
  if (config.meterScale <= 0) config.meterScale = 1.0;
  if (config.learnSeconds < 10) config.learnSeconds = 10;
  if (baselineChanged) warning = false;
}

void Calibrator::setPriorFactor(double factor) { prior = (factor > 0) ? clampFactor(factor) : 0; }

void Calibrator::reset() {
  prior = 0;
  numerator = 0;
  denominator = 0;
  count = 0;
  warning = false;
  speedCount = 0;
  movingSeconds = 0;
  stoppedSeconds = 0;
}

double Calibrator::baselineFactor() const {
  double loss = config.chainLossPercent / 100.0;
  if (loss < 0) loss = 0;
  if (loss > 0.2) loss = 0.2;
  return 1.0 / (1.0 - loss);
}

double Calibrator::factor() const {
  if ((count >= MIN_LEARNED_SECONDS) && (denominator > 0)) {
    return clampFactor(numerator / denominator);
  }
  return (prior > 0) ? prior : baselineFactor();
}

double Calibrator::driftPercent() const { return (factor() / baselineFactor() - 1.0) * 100.0; }

double Calibrator::trainerTargetFor(double crankTarget) const {
  if (config.mode != Mode::POWER_METER) return crankTarget;
  return crankTarget / factor();
}

bool Calibrator::steadySpeed(double speedKph) {
  // keep the last 6 speeds; steady when no step between them exceeds the limit
  for (int index = 5; index > 0; index--) speeds[index] = speeds[index - 1];
  speeds[0] = speedKph;
  if (speedCount < 6) speedCount++;
  if (speedCount < 6) return false;
  for (int index = 0; index < 5; index++) {
    if (std::fabs(speeds[index] - speeds[index + 1]) > MAX_SPEED_STEP_KPH) return false;
  }
  return true;
}

Output Calibrator::update(const Input& input) {
  bool steady = steadySpeed(input.trainerFresh ? input.trainerSpeedKph : 0);
  double crank = input.meterPower * config.meterScale;

  // warm-up: count seconds of riding; a long stop starts a new warm-up
  double speed = input.trainerFresh ? input.trainerSpeedKph : 0;
  if (speed > MOVING_KPH) {
    if (movingSeconds < WARMUP_SECONDS) movingSeconds++;
    stoppedSeconds = 0;
  } else if (speed < STOPPED_KPH) {
    if (stoppedSeconds < RESTART_AFTER_STOP_SECONDS) stoppedSeconds++;
    if (stoppedSeconds >= RESTART_AFTER_STOP_SECONDS) movingSeconds = 0;
  }

  // learn from steady pedalling only: the flywheel stores and releases energy when the speed
  // changes, and the two meters average differently
  bool qualifies = !warmingUp() && input.trainerFresh && input.meterFresh && steady && (input.meterPower > MIN_POWER) &&
                   (input.trainerPower > MIN_POWER) && (input.meterCadence > MIN_CADENCE);
  if (qualifies) {
    if (count == 0) {
      // seed with the prior so the first seconds don't swing
      double start = (prior > 0) ? prior : baselineFactor();
      numerator = input.trainerPower * start;
      denominator = input.trainerPower;
    }
    count++;
    // running mean at first (the prior counting as PRIOR_WEIGHT_SECONDS of evidence), then an
    // exponential average with the configured time constant
    double alpha = 1.0 / (double)(count + 1 + PRIOR_WEIGHT_SECONDS);
    if (alpha < 1.0 / config.learnSeconds) alpha = 1.0 / config.learnSeconds;
    numerator += alpha * (crank - numerator);
    denominator += alpha * (input.trainerPower - denominator);
  }
  double drift = std::fabs(driftPercent());
  if (!warning && (count >= WARN_AFTER_SECONDS) && (drift > config.driftWarnPercent)) {
    warning = true;
  } else if (warning && (drift < config.driftWarnPercent - 1.0)) {
    warning = false;
  }

  return compute(input);
}

Output Calibrator::compute(const Input& input) const {
  double crank = input.meterPower * config.meterScale;
  Output output;
  if (config.mode == Mode::TRAINER) {
    if (input.trainerFresh) {
      output.source = Source::TRAINER;
      output.power = input.trainerPower;
      output.cadence = input.trainerCadence;
    }
    return output;
  }
  if (input.meterFresh) {
    output.source = Source::POWER_METER;
    output.power = crank;
    // the meter counts crank revolutions; when it reports none yet, use the trainer's
    output.cadence = (input.meterCadence > 0 || !input.trainerFresh) ? input.meterCadence : input.trainerCadence;
  } else if (input.trainerFresh) {
    output.source = Source::TRAINER_CORRECTED;
    output.power = input.trainerPower * factor();
    output.cadence = input.trainerCadence;
  }
  return output;
}

}  // namespace PowerCal
