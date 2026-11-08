// PC unit tests for the FTMS / FE-C encoders and the virtual shifting physics.
// Run with: pio test -e native

#include <Calculations.h>
#include <Logger.h>
#include <TrainerProtocol.h>
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
  // in the physical gear on a flat road the trainer's own Crr/Cw do the work (SHIFTR's -1 % offset)
  double grade = Calculations::calculateGearedGrade(totalWeight, 0.0, 85, defaultRatio, defaultRatio, 100);
  TEST_ASSERT_DOUBLE_WITHIN(0.01, -1.0, grade);
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
  RUN_TEST(test_fec_target_power_matches_known_capture);
  RUN_TEST(test_fec_track_resistance);
  RUN_TEST(test_fec_user_configuration);
  RUN_TEST(test_parse_fec_stationary_bike_page);
  RUN_TEST(test_geared_grade_increases_with_gear);
  RUN_TEST(test_geared_grade_default_gear_on_flat);
  RUN_TEST(test_geared_grade_never_nan_or_out_of_range);
  RUN_TEST(test_geared_power_and_resistance_sane);
  RUN_TEST(test_cadence_estimate_round_trip);
  return UNITY_END();
}
