#include <Calculations.h>
#include <cmath>
#include <stdio.h>
#include <Logger.h>

double const Calculations::pi = 3.14159;
double const Calculations::gravity = 9.81;
double const Calculations::rollingResistanceCoefficient = 0.00415; // Asphalt Road
double const Calculations::windResistanceCoefficient = 0.51; // Road bike + touring position
double const Calculations::wheelDiameter = 0.7;

/**
 * Calculates the gravitational resistance based on the total weight and the grade
 * 
 * @param totalWeight Total weight of the bike and the rider in kg
 * @return Gravitational resistance in N
 */
double Calculations::calculateGravitationalResistance(double totalWeight, double grade) {
  double gravitationalResistance = gravity * sin(atan(grade/100)) * totalWeight;
  log_d("Gravitational resistance: %f", gravitationalResistance);
  return gravitationalResistance;
}

/**
 * Calculates the rolling resistance based on the total weight
 * 
 * @param totalWeight Total weight of the bike and the rider in kg
 * @return Rolling resistance in N
 */
double Calculations::calculateRollingResistance(double totalWeight) {
  double rollingResistance = gravity * totalWeight * rollingResistanceCoefficient;
  log_d("Rolling resistance: %f", rollingResistance);
  return rollingResistance;
}

/**
 * Calculates the wind resistance based on the bicycle and wind speed
 * 
 * @param bicycleSpeed Speed of bicycle in m/s
 * @param windSpeed Speed of wind in m/s (positive for headwind, negative for tailwind)
 * @return Wind resistance in N
 */
double Calculations::calculateWindResistance(double bicycleSpeed, double windSpeed, double windCoefficient) {
  double relativeSpeed = bicycleSpeed + windSpeed;
  double windResistance = 0.5 * windCoefficient * relativeSpeed * fabs(relativeSpeed);
  log_d("Wind resistance: %f", windResistance);
  return windResistance;
}

/**
 * Calculates the total resistance
 * 
 * @param totalWeight Total weight of the bike and the rider in kg
 * @param grade Grade in percentage
 * @param speed Speed of bicycle in m/s
 * @param gearRatio Requested gear ratio
 * @param defaultGearRatio Default gear ratio of the bike (chainring / sprocket)
 * @param difficulty Difficulty in percentage (0-200%)
 * @return Total resistance in N
 */
double Calculations::calculateGearedResistance(double totalWeight, double grade, double speed, double gearRatio, double defaultGearRatio, uint16_t difficulty, double rollingCoefficient, double windCoefficient) {
  // Pedal torque on a real bike is road force x gear ratio. The trainer's force acts through the
  // physical gear, so it must be road force x (virtual ratio / physical ratio) - for every part of
  // the road force, air drag included, and for negative (downhill) forces too.
  double relativeGearRatio = calculateRelativeGearRatio(gearRatio, defaultGearRatio);
  double roadForce = calculateRoadForce(totalWeight, grade, speed, difficulty, rollingCoefficient, windCoefficient);
  double gearedTotalResistance = roadForce * relativeGearRatio;
  log_d("Geared resistance: %fN (road force %fN x %f, speed %f km/h, difficulty %d)", gearedTotalResistance, roadForce, relativeGearRatio, speed * 3.6, difficulty);
  return gearedTotalResistance;
}

/**
 * Force on a real bike riding at the given speed (N): gravity (scaled by difficulty, like
 * Zwift's trainer difficulty), rolling resistance and air drag.
 */
double Calculations::calculateRoadForce(double totalWeight, double grade, double speed, uint16_t difficulty, double rollingCoefficient, double windCoefficient) {
  double angle = atan(grade / 100.0);
  double gravitational = gravity * totalWeight * sin(angle) * (difficulty / 100.0);
  double rolling = gravity * totalWeight * rollingCoefficient * cos(angle);
  double wind = calculateWindResistance(speed, 0, windCoefficient);
  return gravitational + rolling + wind;
}

/**
 * Calculates the FE-C basic resistance percentage value for the simulation
 * 
 * @param totalWeight Total weight of the bike and the rider in kg
 * @param grade Grade in percentage
 * @param measuredSpeed Measured speed from trainer in m/s
 * @param cadence Cadence in RPM
 * @param gearRatio Requested gear ratio
 * @param defaultGearRatio Default gear ratio of the bike (chainring / sprocket)
 * @param maximumResistance Maximum resistance in N from trainer
 * @param difficulty Difficulty in percentage (0-200%)
 * @return FE-C basic resistance percentage value in 0.5% (0x00 = 0%, 0xC8 = 100%)
 */
uint8_t Calculations::calculateFECBasicResistancePercentageValue(double totalWeight, double grade, double measuredSpeed, uint8_t cadence, double gearRatio, double defaultGearRatio, uint16_t maximumResistance, uint16_t difficulty) {
  uint8_t basicResistancePercentageValue = 200;

  // calculate the speed based on cadence, wheel diameter and gear ratio
  double speed = calculateSpeed(cadence, wheelDiameter, gearRatio);

  // calculate the geared total resistance
  double gearedTotalResistance = calculateGearedResistance(totalWeight, grade, speed, gearRatio, defaultGearRatio, difficulty);

  if (gearedTotalResistance < 0) {
    basicResistancePercentageValue = 0;
  } else if ((maximumResistance != 0) && (gearedTotalResistance <= maximumResistance)) {
    basicResistancePercentageValue = round(gearedTotalResistance / maximumResistance * 200);
  } 
  log_d("FE-C Basic resistance value: %d (%fN = %f%% of %dN)", basicResistancePercentageValue, gearedTotalResistance, basicResistancePercentageValue / 2.0, maximumResistance);
  return basicResistancePercentageValue;
}

/**
 * Calculates the FE-C track resistance grade for the simulation
 * 
 * @param totalWeight Total weight of the bike and the rider in kg
 * @param grade Grade in percentage
 * @param measuredSpeed Measured speed from trainer in m/s
 * @param cadence Cadence in RPM
 * @param gearRatio Requested gear ratio
 * @param defaultGearRatio Default gear ratio of the bike (chainring / sprocket)
 * @param difficulty Difficulty in percentage (0-200%)
 * @return FE-C resistance grade in 0.01% (0x4E20 = 0%)
 */
uint16_t Calculations::calculateFECTrackResistanceGrade(double totalWeight, double grade, double measuredSpeed, uint8_t cadence, double gearRatio, double defaultGearRatio, uint16_t difficulty) {
  double calculatedGrade = calculateGearedGrade(totalWeight, grade, cadence, gearRatio, defaultGearRatio, difficulty);
  uint16_t trackResistanceGrade = (uint16_t)(0x4E20 + round(calculatedGrade * 100));
  log_d("FE-C Track resistance grade: %d (%f%)", trackResistanceGrade, calculatedGrade);
  return trackResistanceGrade;
}

/**
 * Calculates the FE-C target power value for the simulation
 * 
 * @param totalWeight Total weight of the bike and the rider in kg
 * @param grade Grade in percentage
 * @param measuredSpeed Measured speed from trainer in m/s
 * @param cadence Cadence in RPM
 * @param gearRatio Requested gear ratio
 * @param defaultGearRatio Default gear ratio of the bike (chainring / sprocket)
 * @param difficulty Difficulty in percentage (0-200%)
 * @return FE-C target power value in 0.25W (0x00 = 0W)
 */
uint16_t Calculations::calculateFECTargetPowerValue(double totalWeight, double grade, double measuredSpeed, uint8_t cadence, double gearRatio, double defaultGearRatio, uint16_t difficulty) {
  double power = calculateGearedPower(totalWeight, grade, cadence, gearRatio, defaultGearRatio, difficulty);
  uint16_t targetPowerValue = round(power * 4);
  log_d("FE-C Target power value: %d (%fW)", targetPowerValue, power);
  return targetPowerValue;
}

/**
 * Calculates the speed based on the cadence, wheel diameter and gear ratio
 * 
 * @param cadence Cadence in RPM
 * @param wheelDiameter Wheel diameter in m
 * @param gearRatio Gear ratio
 * @return Speed in m/s
 */
double Calculations::calculateSpeed(double cadence, double wheelDiameter, double gearRatio) {
  return ((cadence * gearRatio) * (wheelDiameter * pi) / 60);
}

/**
 * Calculates the geared value based on the relative gear ratio
 * 
 * @param originalValue Original value (unit doesn't matter)
 * @param gearRatio Gear ratio
 * @return Geared value (unit as original value)
 */
double Calculations::calculateGearedValue(double originalValue, double gearRatio) {
  // multiply for both signs: dividing negative values (as upstream did) inverted the gear effect downhill
  double gearedValue = originalValue * gearRatio;
  log_d("Original / Ratio / Geared: %f / %f / %f", originalValue, gearRatio, gearedValue);
  return gearedValue;
}

/**
 * Calculates the relative gear ratio based on the software requested and hardware installed gear ratio
 * 
 * @param gearRatio Requested gear ratio
 * @param defaultGearRatio Default gear ratio of the bike (chainring / sprocket)
 * @return Relative gear ratio
 */
double Calculations::calculateRelativeGearRatio(double gearRatio, double defaultGearRatio) {
  double relativeGearRatio = gearRatio / defaultGearRatio;
  log_d("Relative gear ratio: %f (%f / %f)", relativeGearRatio, gearRatio, defaultGearRatio);
  return relativeGearRatio;
}
/**
 * Calculates the grade that has to be simulated on the trainer so that the
 * real (fixed) gear feels like the requested virtual gear.
 *
 * The trainer is assumed to apply F = m g sin(atan(grade)) + m g trainerCrr + 0.5 trainerCw v^2 at its
 * own flywheel speed. With trainerCrr = trainerCw = 0 (the default, and what the bridge sends) the
 * whole force is carried by the grade, so the trainer's own rolling/air model can't distort it.
 *
 * @return Grade in percent, clamped to [-40, 40]
 */
double Calculations::calculateGearedGrade(double totalWeight, double grade, double cadence, double gearRatio, double defaultGearRatio, uint16_t difficulty,
                                          double rollingCoefficient, double windCoefficient, double trainerRollingCoefficient, double trainerWindCoefficient,
                                          double trainerWeight) {
  // weight the trainer uses to turn our grade back into force (FTMS has no weight field)
  double conversionWeight = (trainerWeight > 0) ? trainerWeight : totalWeight;
  double speed = calculateSpeed(cadence, wheelDiameter, gearRatio);
  double realSpeed = calculateSpeed(cadence, wheelDiameter, defaultGearRatio);
  double trainerForce = calculateGearedResistance(totalWeight, grade, speed, gearRatio, defaultGearRatio, difficulty, rollingCoefficient, windCoefficient);
  double trainerOwnForce = gravity * conversionWeight * trainerRollingCoefficient + calculateWindResistance(realSpeed, 0, trainerWindCoefficient);
  double sine = (trainerForce - trainerOwnForce) / conversionWeight / gravity;
  if (sine > 0.99) sine = 0.99;
  if (sine < -0.99) sine = -0.99;
  double calculatedGrade = tan(asin(sine)) * 100;
  if (std::isnan(calculatedGrade)) calculatedGrade = 0;
  if (calculatedGrade > 40) calculatedGrade = 40;
  if (calculatedGrade < -40) calculatedGrade = -40;
  log_d("Geared grade: %f%% (Zwift %f%%, trainer force %fN)", calculatedGrade, grade, trainerForce);
  return calculatedGrade;
}

/**
 * Track resistance with air drag computed by the trainer.
 *
 * Pedal torque must equal road force x virtual ratio, i.e. the trainer force (acting through the
 * physical ratio) must be rho x road force(virtual speed), rho = virtual / physical ratio.
 *  - gravity + rolling don't depend on speed: rho x that goes into the grade
 *  - air drag: rho x 0.5 Cw (rho v)^2 = 0.5 (Cw rho^3) v^2 with v = the trainer's own speed, so the
 *    trainer can apply it itself from its flywheel speed. Computing it here instead (v1.5) closed a
 *    loop through 1 Hz updates and ~2 s of delay, which oscillated in the big gears.
 *  - whatever exceeds the protocol's Cw limit is added to the grade using the cadence.
 */
TrainerSimulation Calculations::calculateTrainerSimulation(double totalWeight, double grade, double cadence, double gearRatio, double defaultGearRatio, uint16_t difficulty,
                                                           double rollingCoefficient, double windCoefficient, double maxTrainerWindCoefficient,
                                                           double trainerWeight, double trainerSpeedFactor) {
  TrainerSimulation result = {0, 0, 0};
  double conversionWeight = (trainerWeight > 0) ? trainerWeight : totalWeight;
  if (trainerSpeedFactor < 0.5 || trainerSpeedFactor > 2.0 || std::isnan(trainerSpeedFactor)) trainerSpeedFactor = 1.0;
  double rho = calculateRelativeGearRatio(gearRatio, defaultGearRatio);
  double realSpeed = calculateSpeed(cadence, wheelDiameter, defaultGearRatio);

  // air drag: the trainer sees speed k x v, so it needs Cw rho^3 / k^2
  double wanted = windCoefficient * rho * rho * rho / (trainerSpeedFactor * trainerSpeedFactor);
  result.windCoefficient = wanted;
  if (result.windCoefficient > maxTrainerWindCoefficient) result.windCoefficient = maxTrainerWindCoefficient;
  if (result.windCoefficient < 0) result.windCoefficient = 0;
  // both protocols carry Cw in 0.01 kg/m: round here so the (tiny) rounding error goes into the residual
  result.windCoefficient = round(result.windCoefficient * 100.0) / 100.0;
  if (result.windCoefficient > maxTrainerWindCoefficient) result.windCoefficient = floor(maxTrainerWindCoefficient * 100.0) / 100.0;
  double trainerSpeed = realSpeed * trainerSpeedFactor;
  result.residualForce = 0.5 * (wanted - result.windCoefficient) * trainerSpeed * trainerSpeed;

  // gravity (x difficulty) + rolling, geared, plus the residual drag
  double angle = atan(grade / 100.0);
  double speedIndependent = rho * (gravity * totalWeight * sin(angle) * (difficulty / 100.0) + gravity * totalWeight * rollingCoefficient * cos(angle));
  double sine = (speedIndependent + result.residualForce) / conversionWeight / gravity;
  if (sine > 0.99) sine = 0.99;
  if (sine < -0.99) sine = -0.99;
  double calculatedGrade = tan(asin(sine)) * 100;
  if (std::isnan(calculatedGrade)) calculatedGrade = 0;
  if (calculatedGrade > 40) calculatedGrade = 40;
  if (calculatedGrade < -40) calculatedGrade = -40;
  result.grade = calculatedGrade;
  log_d("Trainer simulation: grade %f%% Cw %f (wanted %f, residual %fN), Zwift %f%%", result.grade, result.windCoefficient, wanted, result.residualForce, grade);
  return result;
}

/**
 * Calculates the power the rider would need in the virtual gear at the current cadence.
 *
 * @return Power in W (>= 0)
 */
double Calculations::calculateGearedPower(double totalWeight, double grade, double cadence, double gearRatio, double defaultGearRatio, uint16_t difficulty, double rollingCoefficient, double windCoefficient) {
  // power of the virtual bike = road force x virtual speed (= trainer force x trainer speed)
  double speed = calculateSpeed(cadence, wheelDiameter, gearRatio);
  double power = calculateRoadForce(totalWeight, grade, speed, difficulty, rollingCoefficient, windCoefficient) * speed;
  if (power < 0 || std::isnan(power)) {
    power = 0;
  }
  return power;
}

/**
 * Calculates the resistance percentage (0..100) of the trainer's maximum resistance.
 * If the maximum resistance is unknown (0) a default of 200N is assumed.
 */
double Calculations::calculateGearedResistancePercentage(double totalWeight, double grade, double cadence, double gearRatio, double defaultGearRatio, uint16_t maximumResistance, uint16_t difficulty, double rollingCoefficient, double windCoefficient) {
  if (maximumResistance == 0) {
    maximumResistance = 200;
  }
  double speed = calculateSpeed(cadence, wheelDiameter, gearRatio);
  double gearedTotalResistance = calculateGearedResistance(totalWeight, grade, speed, gearRatio, defaultGearRatio, difficulty, rollingCoefficient, windCoefficient);
  double percentage = gearedTotalResistance / maximumResistance * 100.0;
  if (percentage < 0 || std::isnan(percentage)) percentage = 0;
  if (percentage > 100) percentage = 100;
  return percentage;
}

/**
 * Estimates the cadence from the trainer's wheel speed. With a Zwift Cog the
 * physical gear never changes, so flywheel speed is proportional to cadence.
 *
 * @param speed Trainer speed in m/s
 * @param gearRatio Physical gear ratio (chainring / sprocket)
 * @return Cadence in RPM
 */
double Calculations::estimateCadenceFromSpeed(double speed, double gearRatio, double wheelDiameterMeters) {
  if (speed <= 0 || gearRatio <= 0 || wheelDiameterMeters <= 0) {
    return 0;
  }
  return speed * 60.0 / (gearRatio * wheelDiameterMeters * pi);
}
