#include <cstdint>
#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#if defined(ESP32)
  #include <Arduino.h>
#endif
// What to send to the trainer in track-resistance mode: a grade plus a wind coefficient that the
// trainer applies itself, from its own flywheel speed, many times a second.
struct TrainerSimulation {
  double grade;            // %
  double windCoefficient;  // kg/m
  double residualForce;    // N of air drag that didn't fit under the trainer's Cw limit (folded into grade)
};

class Calculations {
 public:
  static double calculateGravitationalResistance(double totalWeight, double grade);
  static double calculateRollingResistance(double totalWeight);
  // windCoefficient Cw in kg/m (air density x CdA), F = 0.5 Cw v^2 - the unit FTMS and Zwift use
  static double calculateWindResistance(double bicycleSpeed, double windSpeed, double windCoefficient = 0.51);
  static double calculateRoadForce(double totalWeight, double grade, double speed, uint16_t difficulty, double rollingCoefficient = 0.00415, double windCoefficient = 0.51);
  static double calculateGearedResistance(double totalWeight, double grade, double speed, double gearRatio, double defaultGearRatio, uint16_t difficulty, double rollingCoefficient = 0.00415, double windCoefficient = 0.51);
  static uint8_t calculateFECBasicResistancePercentageValue(double totalWeight, double grade, double measuredSpeed, uint8_t cadence, double gearRatio, double defaultGearRatio, uint16_t maximumResistance, uint16_t difficulty);
  static uint16_t calculateFECTrackResistanceGrade(double totalWeight, double grade, double measuredSpeed, uint8_t cadence, double gearRatio, double defaultGearRatio, uint16_t difficulty);
  static uint16_t calculateFECTargetPowerValue(double totalWeight, double grade, double measuredSpeed, uint8_t cadence, double gearRatio, double defaultGearRatio, uint16_t difficulty);
  static double calculateSpeed(double cadence, double wheelDiameter, double gearRatio);
  static double calculateGearedValue(double originalValue, double gearRatio);
  static double calculateRelativeGearRatio(double gearRatio, double defaultGearRatio);
  static double calculateGearedGrade(double totalWeight, double grade, double cadence, double gearRatio, double defaultGearRatio, uint16_t difficulty,
                                     double rollingCoefficient = 0.00415, double windCoefficient = 0.51, double trainerRollingCoefficient = 0, double trainerWindCoefficient = 0,
                                     double trainerWeight = 0);
  // Splits the geared road force into a speed-independent part (sent as grade) and air drag that the
  // trainer computes itself: drag x rho at virtual speed rho x v = 0.5 x (Cw x rho^3) x v^2.
  // trainerSpeedFactor = trainer reported speed / speed from cadence x physical ratio (wheel size mismatch).
  static TrainerSimulation calculateTrainerSimulation(double totalWeight, double grade, double cadence, double gearRatio, double defaultGearRatio, uint16_t difficulty,
                                                      double rollingCoefficient, double windCoefficient, double maxTrainerWindCoefficient,
                                                      double trainerWeight = 0, double trainerSpeedFactor = 1.0);
  static double calculateGearedPower(double totalWeight, double grade, double cadence, double gearRatio, double defaultGearRatio, uint16_t difficulty, double rollingCoefficient = 0.00415, double windCoefficient = 0.51);
  static double calculateGearedResistancePercentage(double totalWeight, double grade, double cadence, double gearRatio, double defaultGearRatio, uint16_t maximumResistance, uint16_t difficulty, double rollingCoefficient = 0.00415, double windCoefficient = 0.51);
  static double estimateCadenceFromSpeed(double speed, double gearRatio, double wheelDiameterMeters);

  static double const pi;
  static double const gravity;
  static double const rollingResistanceCoefficient;
  static double const windResistanceCoefficient;
  static double const wheelDiameter;

 private:
};

#endif