// PC unit tests for the FTMS / FE-C encoders and the virtual shifting physics.
// Run with: pio test -e native

#include <Calculations.h>
#include <Logger.h>
#include <TrainerProtocol.h>
#include <VirtualGears.h>
#include <PowerCalibrator.h>
#include <unity.h>

#include <cmath>
#include <vector>

using namespace TrainerProtocol;

static const double zwiftGears[] = {0.75, 0.87, 0.99, 1.11, 1.23, 1.38, 1.53, 1.68, 1.86, 2.04, 2.22, 2.40,
                                    2.61, 2.82, 3.03, 3.24, 3.49, 3.74, 3.99, 4.24, 4.54, 4.84, 5.14, 5.49};

static void assertBytes(const std::vector<uint8_t>& expected, const std::vector<uint8_t>& actual) {
  TEST_ASSERT_EQUAL_UINT32(expected.size(), actual.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected.data(), actual.data(), expected.size());
}

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------- FTMS

void test_ftms_simulation_positive_grade(void) {
  // 5 % grade, Crr 0.004, Cw 0.51 kg/m, no wind
  assertBytes({0x11, 0x00, 0x00, 0xF4, 0x01, 40, 51}, ftmsIndoorBikeSimulation(5.0, 0.004, 0.51));
}

void test_ftms_simulation_negative_grade(void) {
  // -2.5 % = -250 = 0xFF06
  assertBytes({0x11, 0x00, 0x00, 0x06, 0xFF, 42, 51}, ftmsIndoorBikeSimulation(-2.5));
}

void test_ftms_simulation_clamps_grade(void) {
  std::vector<uint8_t> data = ftmsIndoorBikeSimulation(75.0);
  TEST_ASSERT_EQUAL_INT16(4000, (int16_t)(data[3] | (data[4] << 8)));
  data = ftmsIndoorBikeSimulation(NAN);
  TEST_ASSERT_EQUAL_INT16(0, (int16_t)(data[3] | (data[4] << 8)));
}

void test_ftms_target_power(void) {
  assertBytes({0x05, 0xFA, 0x00}, ftmsTargetPower(250));
  assertBytes({0x05, 0x00, 0x00}, ftmsTargetPower(-20));
}

void test_ftms_resistance_level(void) {
  assertBytes({0x04, 50}, ftmsTargetResistanceLevel(50));
  // range 0..200 (0.1 units) -> 25 % = 50
  assertBytes({0x04, 50}, ftmsTargetResistanceLevel(25, true, 0, 200));
  assertBytes({0x04, 100}, ftmsTargetResistanceLevel(150));
}

void test_ftms_request_control(void) {
  assertBytes({0x00}, ftmsRequestControl());
}

void test_parse_indoor_bike_data_full(void) {
  // flags 0x0044: speed present (bit0 = 0), cadence and power present
  const uint8_t data[] = {0x44, 0x00, 0x28, 0x0A, 0xB4, 0x00, 0xC8, 0x00};
  IndoorBikeData result;
  TEST_ASSERT_TRUE(parseIndoorBikeData(data, sizeof(data), result));
  TEST_ASSERT_TRUE(result.hasSpeed);
  TEST_ASSERT_DOUBLE_WITHIN(0.001, 26.0, result.speedKph);
  TEST_ASSERT_TRUE(result.hasCadence);
  TEST_ASSERT_DOUBLE_WITHIN(0.001, 90.0, result.cadenceRpm);
  TEST_ASSERT_TRUE(result.hasPower);
  TEST_ASSERT_EQUAL_INT16(200, result.powerWatts);
}

void test_parse_indoor_bike_data_more_data_and_extra_fields(void) {
  // flags 0x0071: no speed (more data), total distance (3 bytes), resistance level, power
  const uint8_t data[] = {0x71, 0x00, 0x10, 0x27, 0x00, 0x14, 0x00, 0x2C, 0x01};
  IndoorBikeData result;
  TEST_ASSERT_TRUE(parseIndoorBikeData(data, sizeof(data), result));
  TEST_ASSERT_FALSE(result.hasSpeed);
  TEST_ASSERT_FALSE(result.hasCadence);
  TEST_ASSERT_TRUE(result.hasPower);
  TEST_ASSERT_EQUAL_INT16(300, result.powerWatts);
}

void test_parse_indoor_bike_data_truncated(void) {
  const uint8_t data[] = {0x44, 0x00, 0x28, 0x0A, 0xB4};
  IndoorBikeData result;
  TEST_ASSERT_FALSE(parseIndoorBikeData(data, sizeof(data), result));
}

void test_parse_control_point_response(void) {
  const uint8_t granted[] = {0x80, 0x00, 0x01};
  ControlPointResponse response;
  TEST_ASSERT_TRUE(parseControlPointResponse(granted, sizeof(granted), response));
  TEST_ASSERT_EQUAL_UINT8(FTMS_REQUEST_CONTROL, response.requestOpCode);
  TEST_ASSERT_EQUAL_UINT8(FTMS_RESULT_SUCCESS, response.resultCode);
  const uint8_t notResponse[] = {0x11, 0x00, 0x01};
  TEST_ASSERT_FALSE(parseControlPointResponse(notResponse, sizeof(notResponse), response));
}

// ---------------------------------------------------------------- Cycling Power

void test_parse_cycling_power_with_crank(void) {
  // flags 0x0020 (crank data), 215 W, 1000 crank revs, event time 0x1000
  const uint8_t data[] = {0x20, 0x00, 0xD7, 0x00, 0xE8, 0x03, 0x00, 0x10};
  CyclingPowerData result;
  TEST_ASSERT_TRUE(parseCyclingPowerMeasurement(data, sizeof(data), result));
  TEST_ASSERT_EQUAL_INT16(215, result.powerWatts);
  TEST_ASSERT_TRUE(result.hasCrank);
  TEST_ASSERT_EQUAL_UINT16(1000, result.crankRevolutions);
  TEST_ASSERT_EQUAL_UINT16(0x1000, result.crankEventTime);
}

void test_parse_cycling_power_wheel_and_crank(void) {
  // flags 0x0031: pedal balance, wheel data (6 bytes), crank data (4 bytes)
  const uint8_t data[] = {0x31, 0x00, 0x2C, 0x01, 0x32, 0x10, 0x00, 0x00, 0x00, 0x00, 0x08, 0x05, 0x00, 0x00, 0x04};
  CyclingPowerData result;
  TEST_ASSERT_TRUE(parseCyclingPowerMeasurement(data, sizeof(data), result));
  TEST_ASSERT_EQUAL_INT16(300, result.powerWatts);
  TEST_ASSERT_TRUE(result.hasWheel);
  TEST_ASSERT_EQUAL_UINT32(16, result.wheelRevolutions);
  TEST_ASSERT_TRUE(result.hasCrank);
  TEST_ASSERT_EQUAL_UINT16(5, result.crankRevolutions);
  TEST_ASSERT_EQUAL_UINT16(0x0400, result.crankEventTime);
}

void test_parse_cycling_power_only(void) {
  const uint8_t data[] = {0x00, 0x00, 0x96, 0x00};
  CyclingPowerData result;
  TEST_ASSERT_TRUE(parseCyclingPowerMeasurement(data, sizeof(data), result));
  TEST_ASSERT_EQUAL_INT16(150, result.powerWatts);
  TEST_ASSERT_FALSE(result.hasCrank);
}

void test_crank_cadence(void) {
  CrankCadence calc(3000);
  calc.update(100, 0, 0);
  // 3 revolutions in 2 s (2048/1024) = 90 rpm
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 90.0, calc.update(103, 2048, 1000));
  // same event repeated: holds the value
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 90.0, calc.update(103, 2048, 2000));
  // wrap-around of both counters: 65535 -> 1 is 2 revs, 65000 -> 300 is 836 ticks
  CrankCadence wrap(3000);
  wrap.update(65535, 65000, 0);
  TEST_ASSERT_DOUBLE_WITHIN(0.1, 2 * 60.0 * 1024.0 / 836.0, wrap.update(1, 300, 500));
  // stopped pedalling
  TEST_ASSERT_EQUAL_DOUBLE(0.0, calc.current(6000));
}

void test_to_hex(void) {
  const uint8_t data[] = {0x44, 0x00, 0xAB};
  TEST_ASSERT_EQUAL_STRING("44 00 AB", toHex(data, sizeof(data)).c_str());
}

// ---------------------------------------------------------------- FE-C

void test_fec_target_power_matches_known_capture(void) {
  // 110 W = 440 * 0.25 W = 0x01B8, XOR checksum identical to upstream SHIFTR's encoder
  assertBytes({0xA4, 0x09, 0x4E, 0x05, 0x31, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xB8, 0x01, 0x91}, fecTargetPower(110));
}

void test_fec_track_resistance(void) {
  std::vector<uint8_t> data = fecTrackResistance(0.0, 0.00415);
  TEST_ASSERT_EQUAL_UINT32(13, data.size());
  TEST_ASSERT_EQUAL_UINT8(0x33, data[4]);
  TEST_ASSERT_EQUAL_UINT16(0x4E20, data[9] | (data[10] << 8));
  TEST_ASSERT_EQUAL_UINT8(0x53, data[11]);
  std::vector<uint8_t> withoutChecksum(data.begin(), data.end() - 1);
  TEST_ASSERT_EQUAL_UINT8(fecChecksum(withoutChecksum), data[12]);
}

void test_fec_user_configuration(void) {
  // 75 kg rider, 10 kg bike (= 200 * 0.05 kg = 0x0C8), 0.7 m wheel, ratio 2.43
  std::vector<uint8_t> data = fecUserConfiguration(10.0, 75.0, 0.7, 2.43);
  TEST_ASSERT_EQUAL_UINT8(0x4F, data[2]);
  TEST_ASSERT_EQUAL_UINT8(0x37, data[4]);
  TEST_ASSERT_EQUAL_UINT16(7500, data[5] | (data[6] << 8));
  TEST_ASSERT_EQUAL_UINT8(0x80, data[8]);
  TEST_ASSERT_EQUAL_UINT8(0x0C, data[9]);
  TEST_ASSERT_EQUAL_UINT8(70, data[10]);
  TEST_ASSERT_EQUAL_UINT8(81, data[11]);
}

void test_parse_fec_stationary_bike_page(void) {
  const uint8_t data[] = {0xA4, 0x09, 0x4E, 0x05, 0x19, 0x00, 0x5A, 0x00, 0x00, 0xFA, 0x00, 0x00, 0x00};
  FECData result;
  TEST_ASSERT_TRUE(parseFECMessage(data, sizeof(data), result));
  TEST_ASSERT_TRUE(result.hasCadence);
  TEST_ASSERT_EQUAL_UINT8(90, result.cadenceRpm);
  TEST_ASSERT_TRUE(result.hasPower);
  TEST_ASSERT_EQUAL_UINT16(250, result.powerWatts);
}

// ---------------------------------------------------------------- physics

static const double totalWeight = 85.0;
static const double defaultRatio = 34.0 / 14.0;

void test_geared_grade_increases_with_gear(void) {
  double previous = -1000;
  for (double gear : zwiftGears) {
    double grade = Calculations::calculateGearedGrade(totalWeight, 3.0, 85, gear, defaultRatio, 100);
    TEST_ASSERT_FALSE(std::isnan(grade));
    TEST_ASSERT_TRUE(grade >= previous);
    previous = grade;
  }
}

void test_geared_grade_default_gear_on_flat(void) {
  // physical gear = virtual gear on a flat road: the trainer must produce exactly the road force,
  // all of it through the grade because the trainer is told Crr = Cw = 0
  double v = Calculations::calculateSpeed(85, Calculations::wheelDiameter, defaultRatio);
  double road = Calculations::calculateRoadForce(totalWeight, 0, v, 100);
  double grade = Calculations::calculateGearedGrade(totalWeight, 0.0, 85, defaultRatio, defaultRatio, 100);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, road, totalWeight * 9.81 * sin(atan(grade / 100.0)));
}

// force the trainer applies for a grade when told Crr = Cw = 0
static double trainerForce(double gradePercent) {
  return totalWeight * 9.81 * sin(atan(gradePercent / 100.0));
}

void test_pedal_torque_matches_real_bike(void) {
  // On a real bike pedal torque ~ road force x gear ratio. On the trainer it is trainer force x
  // physical ratio. These must be equal for every gear, grade and cadence (within clamping).
  const double grades[] = {-4, 0, 2, 5, 8, 12};
  const double cadences[] = {60, 85, 100};
  const double physical[] = {34.0 / 14.0, 50.0 / 14.0};
  for (double r0 : physical) {
    for (double grade : grades) {
      for (double cadence : cadences) {
        for (size_t g = 0; g < VirtualGears::AXS_GEAR_COUNT; g++) {
          double r = (double)VirtualGears::AXS_GEARS[g].chainring / VirtualGears::AXS_GEARS[g].cog;
          double v = Calculations::calculateSpeed(cadence, Calculations::wheelDiameter, r);
          double real = Calculations::calculateRoadForce(totalWeight, grade, v, 100) * r;
          double sent = Calculations::calculateGearedGrade(totalWeight, grade, cadence, r, r0, 100);
          if (fabs(sent) >= 39.9) continue;  // clamped
          TEST_ASSERT_DOUBLE_WITHIN(fabs(real) * 0.002 + 0.01, real, trainerForce(sent) * r0);
        }
      }
    }
  }
}

void test_gear_effect_includes_air_drag(void) {
  // flat road, 85 rpm: one AXS shift 50x15 -> 50x14 is 7 % faster, so air drag (v^2) grows ~15 %
  // and pedal torque by ~20 % on a real bike. Upstream only geared gravity + rolling (~ +13 %).
  double r1 = 50.0 / 15, r2 = 50.0 / 14;
  double f1 = trainerForce(Calculations::calculateGearedGrade(totalWeight, 0, 85, r1, defaultRatio, 100));
  double f2 = trainerForce(Calculations::calculateGearedGrade(totalWeight, 0, 85, r2, defaultRatio, 100));
  double v1 = Calculations::calculateSpeed(85, Calculations::wheelDiameter, r1);
  double v2 = Calculations::calculateSpeed(85, Calculations::wheelDiameter, r2);
  double expected = (Calculations::calculateRoadForce(totalWeight, 0, v2, 100) * r2) / (Calculations::calculateRoadForce(totalWeight, 0, v1, 100) * r1);
  TEST_ASSERT_DOUBLE_WITHIN(0.002, expected, f2 / f1);
  TEST_ASSERT_TRUE(f2 / f1 > 1.18);
}

void test_downhill_harder_gear_needs_less(void) {
  // with a negative road force a harder gear must give a *more* negative trainer force (upstream divided)
  double easy = Calculations::calculateGearedGrade(totalWeight, -8, 85, 50.0 / 28, defaultRatio, 100);
  double hard = Calculations::calculateGearedGrade(totalWeight, -8, 85, 50.0 / 21, defaultRatio, 100);
  TEST_ASSERT_TRUE(hard < easy);
}

void test_trainer_weight_conversion(void) {
  // a trainer that assumes 70 kg needs a steeper grade for the same force
  double own = Calculations::calculateGearedGrade(totalWeight, 4, 85, 50.0 / 17, defaultRatio, 100);
  double light = Calculations::calculateGearedGrade(totalWeight, 4, 85, 50.0 / 17, defaultRatio, 100, 0.00415, 0.51, 0, 0, 70);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, totalWeight * sin(atan(own / 100.0)), 70 * sin(atan(light / 100.0)));
}

void test_wind_coefficient_units(void) {
  // Cw in kg/m (air density x CdA): F = 0.5 Cw v^2
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.5 * 0.51 * 100.0, Calculations::calculateWindResistance(10.0, 0, 0.51));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -0.5 * 0.51 * 4.0, Calculations::calculateWindResistance(-2.0, 0, 0.51));
}

void test_axs_table_is_full_2x12_sorted(void) {
  // all 24 combinations of 50/36 x 10-32, each once, easiest to hardest
  const int cogs[12] = {10, 11, 12, 13, 14, 15, 17, 19, 21, 24, 28, 32};
  for (int ring : {36, 50}) {
    for (int cog : cogs) {
      int found = 0;
      for (size_t g = 0; g < VirtualGears::AXS_GEAR_COUNT; g++) {
        if (VirtualGears::AXS_GEARS[g].chainring == ring && VirtualGears::AXS_GEARS[g].cog == cog) found++;
      }
      TEST_ASSERT_EQUAL_INT(1, found);
    }
  }
  for (size_t g = 1; g < VirtualGears::AXS_GEAR_COUNT; g++) {
    double previous = (double)VirtualGears::AXS_GEARS[g - 1].chainring / VirtualGears::AXS_GEARS[g - 1].cog;
    double current = (double)VirtualGears::AXS_GEARS[g].chainring / VirtualGears::AXS_GEARS[g].cog;
    TEST_ASSERT_TRUE(current > previous);
  }
}

void test_axs_mapper_follows_zwift_gear(void) {
  VirtualGears::GearMapper mapper;
  mapper.setTable(VirtualGears::GearTable::AXS_50_36_10_32);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 36.0 / 32, mapper.map(VirtualGears::ZWIFT_RATIOS[0]));
  TEST_ASSERT_EQUAL_STRING("36x32 (1/24)", mapper.getLabel().c_str());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 50.0 / 10, mapper.map(VirtualGears::ZWIFT_RATIOS[23]));
  TEST_ASSERT_EQUAL_STRING("50x10 (24/24)", mapper.getLabel().c_str());
  // Zwift's default gear 12 (2.40) is 36x14
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 36.0 / 14, mapper.map(2.40));
  TEST_ASSERT_EQUAL_INT(12, mapper.getGear());
  // absolute: the same Zwift ratio always gives the same gear, whatever came before
  mapper.map(VirtualGears::ZWIFT_RATIOS[3]);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 36.0 / 14, mapper.map(2.40));
  for (size_t i = 0; i < VirtualGears::ZWIFT_GEAR_COUNT; i++) {
    mapper.map(VirtualGears::ZWIFT_RATIOS[i]);
    TEST_ASSERT_EQUAL_INT((int)i + 1, mapper.getGear());
  }
}

// force the trainer applies for a grade + Cw at its own speed (trainer Crr = 0)
static double trainerForceWithDrag(double gradePercent, double cw, double trainerSpeed, double weight = totalWeight) {
  return weight * 9.81 * sin(atan(gradePercent / 100.0)) + 0.5 * cw * trainerSpeed * trainerSpeed;
}

void test_trainer_simulation_matches_real_bike(void) {
  // pedal torque = trainer force x physical ratio must equal road force x virtual ratio,
  // whether or not the Cw fits under the protocol limit
  const double grades[] = {-6, -2, 0, 3, 7, 12};
  const double cadences[] = {60, 85, 105};
  const double physical[] = {34.0 / 14.0, 50.0 / 14.0};
  const double limits[] = {2.55, 1.86};
  for (double r0 : physical) {
    for (double limit : limits) {
      for (double grade : grades) {
        for (double cadence : cadences) {
          for (size_t g = 0; g < VirtualGears::AXS_GEAR_COUNT; g++) {
            double r = (double)VirtualGears::AXS_GEARS[g].chainring / VirtualGears::AXS_GEARS[g].cog;
            double v = Calculations::calculateSpeed(cadence, Calculations::wheelDiameter, r);
            double vReal = Calculations::calculateSpeed(cadence, Calculations::wheelDiameter, r0);
            double real = Calculations::calculateRoadForce(totalWeight, grade, v, 100, 0.004, 0.51) * r;
            TrainerSimulation sim = Calculations::calculateTrainerSimulation(totalWeight, grade, cadence, r, r0, 100, 0.004, 0.51, limit);
            TEST_ASSERT_TRUE(sim.windCoefficient <= limit + 1e-9);
            TEST_ASSERT_TRUE(sim.windCoefficient >= 0);
            if (fabs(sim.grade) >= 39.9) continue;  // clamped
            TEST_ASSERT_DOUBLE_WITHIN(fabs(real) * 0.002 + 0.01, real, trainerForceWithDrag(sim.grade, sim.windCoefficient, vReal) * r0);
          }
        }
      }
    }
  }
}

void test_trainer_simulation_grade_ignores_cadence_under_limit(void) {
  // physical 50x14, virtual 50x10: Cw x rho^3 = 0.51 x 1.4^3 = 1.40 fits, so the grade must not
  // depend on cadence - that's what removes the 1 Hz feedback loop that oscillated in v1.5
  double r0 = 50.0 / 14, r = 5.0;
  TrainerSimulation slow = Calculations::calculateTrainerSimulation(totalWeight, 2, 70, r, r0, 100, 0.004, 0.51, 2.55);
  TrainerSimulation fast = Calculations::calculateTrainerSimulation(totalWeight, 2, 100, r, r0, 100, 0.004, 0.51, 2.55);
  TEST_ASSERT_DOUBLE_WITHIN(0.005, slow.grade, fast.grade);  // only the 0.01 kg/m rounding of Cw remains
  TEST_ASSERT_DOUBLE_WITHIN(0.006, 0.51 * 1.4 * 1.4 * 1.4, slow.windCoefficient);
  TEST_ASSERT_TRUE(fabs(slow.residualForce) < 0.1);
  // physical 34x14: 0.51 x 2.06^3 = 4.45 doesn't fit, the rest goes into the grade
  TrainerSimulation capped = Calculations::calculateTrainerSimulation(totalWeight, 2, 90, r, 34.0 / 14, 100, 0.004, 0.51, 2.55);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.55, capped.windCoefficient);
  TEST_ASSERT_TRUE(capped.residualForce > 10);
}

void test_trainer_simulation_weight_only_scales_grade(void) {
  double r0 = 50.0 / 14, r = 50.0 / 17;
  TrainerSimulation own = Calculations::calculateTrainerSimulation(totalWeight, 5, 85, r, r0, 100, 0.004, 0.51, 2.55);
  TrainerSimulation light = Calculations::calculateTrainerSimulation(totalWeight, 5, 85, r, r0, 100, 0.004, 0.51, 2.55, 70);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, own.windCoefficient, light.windCoefficient);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, totalWeight * sin(atan(own.grade / 100.0)), 70 * sin(atan(light.grade / 100.0)));
}

void test_zwift_mapper_passthrough(void) {
  VirtualGears::GearMapper mapper;
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 3.03, mapper.map(3.03));
  TEST_ASSERT_EQUAL_INT(15, mapper.getGear());
  TEST_ASSERT_EQUAL_STRING("15/24", mapper.getLabel().c_str());
  TEST_ASSERT_EQUAL_INT(1, VirtualGears::zwiftGearIndex(0.70));
  TEST_ASSERT_EQUAL_INT(24, VirtualGears::zwiftGearIndex(6.0));
}

void test_fec_wind_page(void) {
  std::vector<uint8_t> data = fecWindResistance(0.0);
  TEST_ASSERT_EQUAL_UINT8(0x32, data[4]);
  TEST_ASSERT_EQUAL_UINT8(0, data[9]);
  TEST_ASSERT_EQUAL_UINT8(127, data[10]);
  TEST_ASSERT_EQUAL_UINT8(100, data[11]);
  TEST_ASSERT_EQUAL_UINT8(51, fecWindResistance(0.51)[9]);
}

void test_geared_grade_never_nan_or_out_of_range(void) {
  double grades[] = {-20, -5, 0, 5, 12, 20, 25};
  double cadences[] = {0, 30, 90, 130, 200};
  for (double grade : grades) {
    for (double cadence : cadences) {
      for (double gear : zwiftGears) {
        double result = Calculations::calculateGearedGrade(totalWeight, grade, cadence, gear, defaultRatio, 200);
        TEST_ASSERT_FALSE(std::isnan(result));
        TEST_ASSERT_TRUE(result <= 40.0 && result >= -40.0);
      }
    }
  }
}

void test_geared_power_and_resistance_sane(void) {
  double easy = Calculations::calculateGearedPower(totalWeight, 5.0, 85, 1.23, defaultRatio, 100);
  double hard = Calculations::calculateGearedPower(totalWeight, 5.0, 85, 4.24, defaultRatio, 100);
  TEST_ASSERT_TRUE(hard > easy);
  TEST_ASSERT_TRUE(easy > 0);
  TEST_ASSERT_EQUAL_DOUBLE(0.0, Calculations::calculateGearedPower(totalWeight, 5.0, 0, 4.24, defaultRatio, 100));
  double resistance = Calculations::calculateGearedResistancePercentage(totalWeight, 5.0, 85, 2.4, defaultRatio, 0, 100);
  TEST_ASSERT_TRUE(resistance > 0 && resistance <= 100);
}

void test_cadence_estimate_round_trip(void) {
  double speed = Calculations::calculateSpeed(90, Calculations::wheelDiameter, defaultRatio);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 90.0, Calculations::estimateCadenceFromSpeed(speed, defaultRatio, Calculations::wheelDiameter));
  TEST_ASSERT_EQUAL_DOUBLE(0.0, Calculations::estimateCadenceFromSpeed(0, defaultRatio, Calculations::wheelDiameter));
}


void test_cps_full_measurement(void) {
  // flags: balance (left reference), accumulated torque (crank based), crank data, offset indicator
  // power 250 W, balance 50.5 %, torque 1000/32 Nm, crank 1234 revs at 5000/1024 s
  uint16_t flags = 0x0001 | 0x0002 | 0x0004 | 0x0008 | 0x0020 | 0x1000;
  uint8_t data[] = {(uint8_t)(flags & 0xFF), (uint8_t)(flags >> 8), 250, 0, 101, 0xE8, 0x03, 0xD2, 0x04, 0x88, 0x13};
  TrainerProtocol::CyclingPowerData result;
  TEST_ASSERT_TRUE(TrainerProtocol::parseCyclingPowerMeasurement(data, sizeof(data), result));
  TEST_ASSERT_EQUAL_INT(250, result.powerWatts);
  TEST_ASSERT_TRUE(result.hasBalance);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 50.5, result.balancePercent);
  TEST_ASSERT_TRUE(result.balanceReferenceLeft);
  TEST_ASSERT_TRUE(result.hasAccumulatedTorque);
  TEST_ASSERT_EQUAL_UINT16(1000, result.accumulatedTorque);
  TEST_ASSERT_TRUE(result.torqueFromCrank);
  TEST_ASSERT_TRUE(result.hasCrank);
  TEST_ASSERT_EQUAL_UINT16(1234, result.crankRevolutions);
  TEST_ASSERT_EQUAL_UINT16(5000, result.crankEventTime);
  TEST_ASSERT_FALSE(result.hasWheel);
  TEST_ASSERT_TRUE(result.offsetCompensationIndicator);
}

void test_cps_truncated_keeps_power(void) {
  uint8_t data[] = {0x20, 0x00, 0x2C, 0x01, 0x10};  // says crank data follows, but it's cut off
  TrainerProtocol::CyclingPowerData result;
  TEST_ASSERT_TRUE(TrainerProtocol::parseCyclingPowerMeasurement(data, sizeof(data), result));
  TEST_ASSERT_EQUAL_INT(300, result.powerWatts);
  TEST_ASSERT_FALSE(result.hasCrank);
}

void test_cps_feature_names(void) {
  std::string text = TrainerProtocol::describeCyclingPowerFeatures((1u << 0) | (1u << 3) | (1u << 9));
  TEST_ASSERT_EQUAL_STRING("pedal_balance, crank_revolutions, offset_compensation", text.c_str());
}

// ---------- power calibration (v2 phase 4) ----------
static PowerCal::Input ride(double crank, double trainerShare, double meterScale, double speed = 30, bool meter = true, bool trainer = true) {
  PowerCal::Input input;
  input.trainerFresh = trainer;
  input.trainerPower = crank * trainerShare;
  input.trainerCadence = 85;
  input.trainerSpeedKph = speed;
  input.meterFresh = meter;
  input.meterPower = crank / meterScale;
  input.meterCadence = 85;
  return input;
}

void test_calibrator_learns_chain_loss(void) {
  PowerCal::Calibrator calibrator;
  PowerCal::Config config;  // scale 1.055, 3 % loss
  calibrator.configure(config);
  TEST_ASSERT_DOUBLE_WITHIN(1e-6, 1.0 / 0.97, calibrator.factor());
  PowerCal::Output output;
  for (int second = 0; second < 600; second++) output = calibrator.update(ride(220, 0.97, 1.055));
  TEST_ASSERT_EQUAL_INT((int)PowerCal::Source::POWER_METER, (int)output.source);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 220, output.power);  // corrected meter = crank power
  TEST_ASSERT_DOUBLE_WITHIN(0.002, 1.0 / 0.97, calibrator.factor());
  TEST_ASSERT_DOUBLE_WITHIN(0.2, 0.0, calibrator.driftPercent());
  TEST_ASSERT_FALSE(calibrator.driftWarning());
}

void test_calibrator_fallback_is_continuous(void) {
  // the trainer reads 2 % high against the chain-loss expectation; after learning, losing the
  // meter must not change the power Zwift sees
  PowerCal::Calibrator calibrator;
  calibrator.configure(PowerCal::Config());
  PowerCal::Output output;
  for (int second = 0; second < 900; second++) output = calibrator.update(ride(250, 0.97 * 1.02, 1.055));
  double withMeter = output.power;
  output = calibrator.update(ride(250, 0.97 * 1.02, 1.055, 30, false));
  TEST_ASSERT_EQUAL_INT((int)PowerCal::Source::TRAINER_CORRECTED, (int)output.source);
  TEST_ASSERT_DOUBLE_WITHIN(withMeter * 0.003, withMeter, output.power);
}

void test_calibrator_warns_on_drift(void) {
  PowerCal::Calibrator calibrator;
  calibrator.configure(PowerCal::Config());
  // trainer reads 6 % high: learned factor 6 % below baseline
  for (int second = 0; second < 100; second++) calibrator.update(ride(200, 0.97 * 1.06, 1.055));
  TEST_ASSERT_FALSE(calibrator.driftWarning());  // not enough evidence yet
  for (int second = 0; second < 900; second++) calibrator.update(ride(200, 0.97 * 1.06, 1.055));
  TEST_ASSERT_TRUE(calibrator.driftPercent() < -5.0);
  TEST_ASSERT_TRUE(calibrator.driftWarning());
  // after a spindown the trainer is right again: reset, relearn, warning clears
  calibrator.reset();
  for (int second = 0; second < 900; second++) calibrator.update(ride(200, 0.97, 1.055));
  TEST_ASSERT_FALSE(calibrator.driftWarning());
  TEST_ASSERT_DOUBLE_WITHIN(0.3, 0.0, calibrator.driftPercent());
}

void test_calibrator_ignores_unsteady_riding(void) {
  PowerCal::Calibrator calibrator;
  calibrator.configure(PowerCal::Config());
  // speed changes 1 km/h every second: flywheel inertia makes these samples misleading
  for (int second = 0; second < 300; second++) calibrator.update(ride(200, 0.80, 1.055, 25 + (second % 10)));
  TEST_ASSERT_EQUAL_UINT32(0, calibrator.learnedSeconds());
  // coasting / low power doesn't count either
  for (int second = 0; second < 300; second++) calibrator.update(ride(40, 0.80, 1.055));
  TEST_ASSERT_EQUAL_UINT32(0, calibrator.learnedSeconds());
}

void test_calibrator_warmup_start_transient(void) {
  // 11-27: the first two minutes read 10-17 % apart, which raised a false drift warning
  PowerCal::Calibrator calibrator;
  calibrator.configure(PowerCal::Config());
  calibrator.setPriorFactor(1.0123);
  for (int second = 0; second < 60; second++) calibrator.update(ride(200, 1.0 / 1.17, 1.055));
  for (int second = 0; second < 59; second++) calibrator.update(ride(200, 1.0 / 1.10, 1.055));
  TEST_ASSERT_TRUE(calibrator.warmingUp());  // warm-up ends with the 120th second
  TEST_ASSERT_EQUAL_UINT32(0, calibrator.learnedSeconds());
  for (int second = 0; second < 600; second++) calibrator.update(ride(200, 1.0 / 1.025, 1.055));
  TEST_ASSERT_FALSE(calibrator.warmingUp());
  TEST_ASSERT_FALSE(calibrator.driftWarning());
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 1.025, calibrator.factor());
}

void test_calibrator_prior_weight(void) {
  // a minute of odd readings after the warm-up moves a saved factor only part of the way
  PowerCal::Calibrator calibrator;
  calibrator.configure(PowerCal::Config());
  calibrator.setPriorFactor(1.0123);
  for (int second = 0; second < (int)PowerCal::Calibrator::WARMUP_SECONDS; second++) calibrator.update(ride(30, 1.0, 1.055));
  for (int second = 0; second < 60; second++) calibrator.update(ride(200, 1.0 / 1.10, 1.055));
  TEST_ASSERT_TRUE(calibrator.factor() < 1.045);
  TEST_ASSERT_TRUE(calibrator.factor() > 1.02);
}

void test_calibrator_stop_restarts_warmup(void) {
  PowerCal::Calibrator calibrator;
  calibrator.configure(PowerCal::Config());
  for (int second = 0; second < 200; second++) calibrator.update(ride(200, 0.97, 1.055));
  TEST_ASSERT_FALSE(calibrator.warmingUp());
  // a 30 s stop (traffic light in Zwift) keeps learning going
  for (int second = 0; second < 30; second++) calibrator.update(ride(0, 0.97, 1.055, 0));
  TEST_ASSERT_FALSE(calibrator.warmingUp());
  // a 60 s stop starts a new warm-up
  for (int second = 0; second < 60; second++) calibrator.update(ride(0, 0.97, 1.055, 0));
  TEST_ASSERT_TRUE(calibrator.warmingUp());
  uint32_t learned = calibrator.learnedSeconds();
  for (int second = 0; second < 100; second++) calibrator.update(ride(200, 0.97, 1.055));
  TEST_ASSERT_EQUAL_UINT32(learned, calibrator.learnedSeconds());
}

void test_calibrator_erg_target(void) {
  PowerCal::Calibrator calibrator;
  calibrator.configure(PowerCal::Config());
  // trainer = 0.97 × crank: to get 200 W at the crank the trainer must hold 194 W
  TEST_ASSERT_DOUBLE_WITHIN(0.01, 194.0, calibrator.trainerTargetFor(200));
  PowerCal::Config trainerOnly;
  trainerOnly.mode = PowerCal::Mode::TRAINER;
  calibrator.configure(trainerOnly);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 200.0, calibrator.trainerTargetFor(200));
  PowerCal::Output output = calibrator.update(ride(200, 0.97, 1.055));
  TEST_ASSERT_EQUAL_INT((int)PowerCal::Source::TRAINER, (int)output.source);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 194.0, output.power);
}

void test_rewrite_indoor_bike_data(void) {
  // flags 0x0044: speed (more data = 0), cadence, power. speed 3000, cadence 170 (85 rpm), power 200
  uint8_t data[] = {0x44, 0x00, 0xB8, 0x0B, 0xAA, 0x00, 0xC8, 0x00};
  TEST_ASSERT_TRUE(TrainerProtocol::rewriteIndoorBikeData(data, sizeof(data), 243, 91.5));
  TrainerProtocol::IndoorBikeData parsed;
  TEST_ASSERT_TRUE(TrainerProtocol::parseIndoorBikeData(data, sizeof(data), parsed));
  TEST_ASSERT_EQUAL_INT(243, parsed.powerWatts);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 91.5, parsed.cadenceRpm);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 30.0, parsed.speedKph);
  // with resistance level between cadence and power
  uint8_t withResistance[] = {0x64, 0x00, 0xB8, 0x0B, 0xAA, 0x00, 0x10, 0x00, 0xC8, 0x00};
  TEST_ASSERT_TRUE(TrainerProtocol::rewriteIndoorBikeData(withResistance, sizeof(withResistance), 301, 80));
  TEST_ASSERT_TRUE(TrainerProtocol::parseIndoorBikeData(withResistance, sizeof(withResistance), parsed));
  TEST_ASSERT_EQUAL_INT(301, parsed.powerWatts);
  TEST_ASSERT_EQUAL_UINT8(0x10, withResistance[6]);
}

void test_rewrite_cycling_power(void) {
  uint8_t data[] = {0x20, 0x00, 0xC8, 0x00, 0x10, 0x00, 0x00, 0x04};
  TEST_ASSERT_TRUE(TrainerProtocol::rewriteCyclingPowerMeasurement(data, sizeof(data), 243));
  TrainerProtocol::CyclingPowerData parsed;
  TEST_ASSERT_TRUE(TrainerProtocol::parseCyclingPowerMeasurement(data, sizeof(data), parsed));
  TEST_ASSERT_EQUAL_INT(243, parsed.powerWatts);
  TEST_ASSERT_EQUAL_UINT16(16, parsed.crankRevolutions);
}

// ---------- FTMS spin down ----------
void test_spindown_encode_and_parse(void) {
  std::vector<uint8_t> start = TrainerProtocol::ftmsSpinDownControl(TrainerProtocol::FTMS_SPIN_DOWN_START);
  TEST_ASSERT_EQUAL_UINT8(0x13, start[0]);
  TEST_ASSERT_EQUAL_UINT8(0x01, start[1]);
  // success, low 28.00 km/h (2800 = 0x0AF0), high 32.00 km/h (3200 = 0x0C80)
  uint8_t response[] = {0x80, 0x13, 0x01, 0xF0, 0x0A, 0x80, 0x0C};
  uint8_t result = 0;
  double low = 0, high = 0;
  TEST_ASSERT_TRUE(TrainerProtocol::parseSpinDownResponse(response, sizeof(response), result, low, high));
  TEST_ASSERT_EQUAL_UINT8(1, result);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 28.0, low);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 32.0, high);
  uint8_t other[] = {0x80, 0x11, 0x01};
  TEST_ASSERT_FALSE(TrainerProtocol::parseSpinDownResponse(other, sizeof(other), result, low, high));
}

void test_spindown_happy_path(void) {
  TrainerProtocol::SpinDown spinDown;
  spinDown.start(1000);
  TEST_ASSERT_TRUE(spinDown.active());
  spinDown.onResponse(0x01, 28, 32, 1200);
  TEST_ASSERT_EQUAL_STRING("speed_up", spinDown.getStateName());
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 32, spinDown.getTargetHighKph());
  spinDown.onStatus(TrainerProtocol::FTMS_SPIN_DOWN_STATUS_STOP_PEDALING, 15000);
  TEST_ASSERT_EQUAL_STRING("coast", spinDown.getStateName());
  spinDown.onStatus(TrainerProtocol::FTMS_SPIN_DOWN_STATUS_SUCCESS, 30000);
  TEST_ASSERT_EQUAL_STRING("success", spinDown.getStateName());
  TEST_ASSERT_FALSE(spinDown.active());
  spinDown.tick(30000 + TrainerProtocol::SpinDown::RESULT_SHOWN_MS + 1);
  TEST_ASSERT_EQUAL_STRING("idle", spinDown.getStateName());
}

void test_spindown_failures(void) {
  TrainerProtocol::SpinDown spinDown;
  spinDown.start(0);
  spinDown.tick(TrainerProtocol::SpinDown::ANSWER_TIMEOUT_MS + 1);
  TEST_ASSERT_EQUAL_STRING("failed", spinDown.getStateName());
  spinDown.start(0);
  spinDown.onResponse(0x05, 0, 0, 100);  // control not permitted
  TEST_ASSERT_EQUAL_STRING("failed", spinDown.getStateName());
  spinDown.start(0);
  spinDown.onResponse(0x01, 28, 32, 100);
  spinDown.onStatus(TrainerProtocol::FTMS_SPIN_DOWN_STATUS_ERROR, 5000);
  TEST_ASSERT_EQUAL_STRING("failed", spinDown.getStateName());
  spinDown.start(0);
  spinDown.onResponse(0x01, 28, 32, 100);
  spinDown.tick(TrainerProtocol::SpinDown::PROCEDURE_TIMEOUT_MS + 1);
  TEST_ASSERT_EQUAL_STRING("failed", spinDown.getStateName());
  // a "stop pedalling" without a speed range still moves on
  spinDown.start(0);
  spinDown.onStatus(TrainerProtocol::FTMS_SPIN_DOWN_STATUS_STOP_PEDALING, 100);
  TEST_ASSERT_EQUAL_STRING("coast", spinDown.getStateName());
  // the trainer asking for a spin down while idle is remembered
  TrainerProtocol::SpinDown idle;
  idle.onStatus(TrainerProtocol::FTMS_SPIN_DOWN_STATUS_REQUESTED, 0);
  TEST_ASSERT_TRUE(idle.wasRequestedByTrainer());
}

int main(int argc, char** argv) {
  Logger::defaultLogLevel = LOG_LEVEL_ERROR;
  UNITY_BEGIN();
  RUN_TEST(test_ftms_simulation_positive_grade);
  RUN_TEST(test_ftms_simulation_negative_grade);
  RUN_TEST(test_ftms_simulation_clamps_grade);
  RUN_TEST(test_ftms_target_power);
  RUN_TEST(test_ftms_resistance_level);
  RUN_TEST(test_ftms_request_control);
  RUN_TEST(test_parse_indoor_bike_data_full);
  RUN_TEST(test_parse_indoor_bike_data_more_data_and_extra_fields);
  RUN_TEST(test_parse_indoor_bike_data_truncated);
  RUN_TEST(test_parse_control_point_response);
  RUN_TEST(test_parse_cycling_power_with_crank);
  RUN_TEST(test_parse_cycling_power_wheel_and_crank);
  RUN_TEST(test_parse_cycling_power_only);
  RUN_TEST(test_crank_cadence);
  RUN_TEST(test_to_hex);
  RUN_TEST(test_fec_target_power_matches_known_capture);
  RUN_TEST(test_fec_track_resistance);
  RUN_TEST(test_fec_user_configuration);
  RUN_TEST(test_parse_fec_stationary_bike_page);
  RUN_TEST(test_geared_grade_increases_with_gear);
  RUN_TEST(test_geared_grade_default_gear_on_flat);
  RUN_TEST(test_pedal_torque_matches_real_bike);
  RUN_TEST(test_gear_effect_includes_air_drag);
  RUN_TEST(test_downhill_harder_gear_needs_less);
  RUN_TEST(test_trainer_weight_conversion);
  RUN_TEST(test_wind_coefficient_units);
  RUN_TEST(test_spindown_encode_and_parse);
  RUN_TEST(test_spindown_happy_path);
  RUN_TEST(test_spindown_failures);
  RUN_TEST(test_calibrator_learns_chain_loss);
  RUN_TEST(test_calibrator_fallback_is_continuous);
  RUN_TEST(test_calibrator_warns_on_drift);
  RUN_TEST(test_calibrator_ignores_unsteady_riding);
  RUN_TEST(test_calibrator_erg_target);
  RUN_TEST(test_calibrator_warmup_start_transient);
  RUN_TEST(test_calibrator_prior_weight);
  RUN_TEST(test_calibrator_stop_restarts_warmup);
  RUN_TEST(test_rewrite_indoor_bike_data);
  RUN_TEST(test_rewrite_cycling_power);
  RUN_TEST(test_cps_full_measurement);
  RUN_TEST(test_cps_truncated_keeps_power);
  RUN_TEST(test_cps_feature_names);
  RUN_TEST(test_axs_table_is_full_2x12_sorted);
  RUN_TEST(test_axs_mapper_follows_zwift_gear);
  RUN_TEST(test_trainer_simulation_matches_real_bike);
  RUN_TEST(test_trainer_simulation_grade_ignores_cadence_under_limit);
  RUN_TEST(test_trainer_simulation_weight_only_scales_grade);
  RUN_TEST(test_zwift_mapper_passthrough);
  RUN_TEST(test_fec_wind_page);
  RUN_TEST(test_geared_grade_never_nan_or_out_of_range);
  RUN_TEST(test_geared_power_and_resistance_sane);
  RUN_TEST(test_cadence_estimate_round_trip);
  return UNITY_END();
}
