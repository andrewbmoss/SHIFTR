#include <SettingsManager.h>
#include <Utils.h>
#include <Config.h>
#include <cmath>

char SettingsManager::iotWebConfChainringTeethParameterValue[16];
char SettingsManager::iotWebConfSprocketTeethParameterValue[16];
char SettingsManager::iotWebConfVirtualShiftingModeParameterValue[24];
char SettingsManager::iotWebConfVirtualShiftingParameterValue[16];
char SettingsManager::iotWebConfTrainerDeviceParameterValue[128];
char SettingsManager::iotWebConfGradeSmoothingParameterValue[16];
char SettingsManager::iotWebConfDifficultyParameterValue[16];
char SettingsManager::iotWebConfTrainerWeightParameterValue[16];
char SettingsManager::iotWebConfFTMSParameterValue[16];
char SettingsManager::iotWebConfTrainerProtocolParameterValue[16];
char SettingsManager::iotWebConfTrainerProtocolValues[][16] = { "auto", "ftms", "fec" };
char SettingsManager::iotWebConfTrainerProtocolNames[][16] = { "Auto (FTMS)", "FTMS", "FE-C" };
char SettingsManager::iotWebConfGearTableParameterValue[24];
char SettingsManager::iotWebConfGearTableValues[][24] = { "zwift", "axs_50_36_10_32" };
char SettingsManager::iotWebConfGearTableNames[][24] = { "Zwift (24 gears)", "SRAM AXS 50/36 x 10-32" };

char SettingsManager::iotWebConfVirtualShiftingModeValues[][24] = { "basic_resistance", "target_power", "track_resistance" };
char SettingsManager::iotWebConfVirtualShiftingModeNames[][24] = { "Basic Resistance", "Target Power", "Track Resistance" };

IotWebConfParameterGroup SettingsManager::iotWebConfSettingsGroup = IotWebConfParameterGroup("settings", "Device settings");
IotWebConfNumberParameter SettingsManager::iotWebConfChainringTeethParameter("Chainring teeth", "chainring_teeth", SettingsManager::iotWebConfChainringTeethParameterValue, sizeof(iotWebConfChainringTeethParameterValue), "34", "1..100", "min='1' max='100' step='1'");
IotWebConfNumberParameter SettingsManager::iotWebConfSprocketTeethParameter("Sprocket teeth", "sprocket_teeth", SettingsManager::iotWebConfSprocketTeethParameterValue, sizeof(iotWebConfSprocketTeethParameterValue), "14", "1..100", "min='1' max='100' step='1'");
IotWebConfSelectParameter SettingsManager::iotWebConfVirtualShiftingModeParameter = IotWebConfSelectParameter("Virtual Shifting Mode", "virtual_shifting_mode", SettingsManager::iotWebConfVirtualShiftingModeParameterValue, sizeof(iotWebConfVirtualShiftingModeParameterValue), (char*)iotWebConfVirtualShiftingModeValues, (char*)iotWebConfVirtualShiftingModeNames, sizeof(iotWebConfVirtualShiftingModeValues) / sizeof(iotWebConfVirtualShiftingModeParameterValue), sizeof(iotWebConfVirtualShiftingModeParameterValue), "track_resistance");
IotWebConfCheckboxParameter SettingsManager::iotWebConfVirtualShiftingParameter = IotWebConfCheckboxParameter("Virtual shifting", "virtual_shifting", SettingsManager::iotWebConfVirtualShiftingParameterValue, sizeof(iotWebConfVirtualShiftingParameterValue), true);
IotWebConfTextParameter SettingsManager::iotWebConfTrainerDeviceParameter = IotWebConfTextParameter("Trainer device", "trainer_device", iotWebConfTrainerDeviceParameterValue, sizeof(iotWebConfTrainerDeviceParameterValue), "");
// same 16 byte slot as the old on/off checkbox, so stored settings keep their layout
IotWebConfNumberParameter SettingsManager::iotWebConfGradeSmoothingParameter("Grade smoothing (s)", "grade_smoothing", SettingsManager::iotWebConfGradeSmoothingParameterValue, sizeof(iotWebConfGradeSmoothingParameterValue), "4", "0..10", "min='0' max='10' step='0.5'");
IotWebConfNumberParameter SettingsManager::iotWebConfTrainerWeightParameter("Trainer weight", "trainer_weight", SettingsManager::iotWebConfTrainerWeightParameterValue, sizeof(iotWebConfTrainerWeightParameterValue), "0", "0..250", "min='0' max='250' step='1'");
IotWebConfNumberParameter SettingsManager::iotWebConfDifficultyParameter("Difficulty", "difficulty", SettingsManager::iotWebConfDifficultyParameterValue, sizeof(iotWebConfDifficultyParameterValue), "100", "1..200", "min='1' max='200' step='1'");
IotWebConfCheckboxParameter SettingsManager::iotWebConfFTMSParameter = IotWebConfCheckboxParameter("FTMS Emulation", "ftms_emulation", SettingsManager::iotWebConfFTMSParameterValue, sizeof(iotWebConfFTMSParameterValue), false);

IotWebConfSelectParameter SettingsManager::iotWebConfTrainerProtocolParameter = IotWebConfSelectParameter("Trainer protocol", "trainer_protocol", SettingsManager::iotWebConfTrainerProtocolParameterValue, sizeof(iotWebConfTrainerProtocolParameterValue), (char*)iotWebConfTrainerProtocolValues, (char*)iotWebConfTrainerProtocolNames, sizeof(iotWebConfTrainerProtocolValues) / sizeof(iotWebConfTrainerProtocolParameterValue), sizeof(iotWebConfTrainerProtocolParameterValue), "auto");

IotWebConfSelectParameter SettingsManager::iotWebConfGearTableParameter = IotWebConfSelectParameter("Gear table", "gear_table", SettingsManager::iotWebConfGearTableParameterValue, sizeof(iotWebConfGearTableParameterValue), (char*)iotWebConfGearTableValues, (char*)iotWebConfGearTableNames, sizeof(iotWebConfGearTableValues) / sizeof(iotWebConfGearTableParameterValue), sizeof(iotWebConfGearTableParameterValue), "axs_50_36_10_32");

IotWebConf* SettingsManager::iotWebConf;

void SettingsManager::initialize(IotWebConf* iotWebConf) {
  SettingsManager::iotWebConf = iotWebConf;
  iotWebConfSettingsGroup.addItem(&iotWebConfTrainerDeviceParameter);
  iotWebConfSettingsGroup.addItem(&iotWebConfVirtualShiftingParameter);
  iotWebConfSettingsGroup.addItem(&iotWebConfChainringTeethParameter);
  iotWebConfSettingsGroup.addItem(&iotWebConfSprocketTeethParameter);
  iotWebConfSettingsGroup.addItem(&iotWebConfVirtualShiftingModeParameter);
  iotWebConfSettingsGroup.addItem(&iotWebConfGradeSmoothingParameter);
  iotWebConfSettingsGroup.addItem(&iotWebConfDifficultyParameter);
  iotWebConfSettingsGroup.addItem(&iotWebConfTrainerWeightParameter);
  iotWebConfSettingsGroup.addItem(&iotWebConfFTMSParameter);
  iotWebConfSettingsGroup.addItem(&iotWebConfTrainerProtocolParameter);
  iotWebConfSettingsGroup.addItem(&iotWebConfGearTableParameter);
}

VirtualShiftingMode SettingsManager::getVirtualShiftingMode() {
  if (strncmp(iotWebConfVirtualShiftingModeParameterValue, iotWebConfVirtualShiftingModeValues[1], sizeof(iotWebConfVirtualShiftingModeParameterValue)) == 0) {
    return VirtualShiftingMode::TARGET_POWER;
  }
  if (strncmp(iotWebConfVirtualShiftingModeParameterValue, iotWebConfVirtualShiftingModeValues[2], sizeof(iotWebConfVirtualShiftingModeParameterValue)) == 0) {
    return VirtualShiftingMode::TRACK_RESISTANCE;
  }
  if (strncmp(iotWebConfVirtualShiftingModeParameterValue, iotWebConfVirtualShiftingModeValues[0], sizeof(iotWebConfVirtualShiftingModeParameterValue)) == 0) {
    return VirtualShiftingMode::BASIC_RESISTANCE;
  }
  return VirtualShiftingMode::TRACK_RESISTANCE;
}

void SettingsManager::setVirtualShiftingMode(VirtualShiftingMode virtualShiftingMode) {
  switch (virtualShiftingMode) {
    case VirtualShiftingMode::TARGET_POWER:
      strncpy(iotWebConfVirtualShiftingModeParameterValue, iotWebConfVirtualShiftingModeValues[1], sizeof(iotWebConfVirtualShiftingModeParameterValue));
      break;
    case VirtualShiftingMode::TRACK_RESISTANCE:
      strncpy(iotWebConfVirtualShiftingModeParameterValue, iotWebConfVirtualShiftingModeValues[2], sizeof(iotWebConfVirtualShiftingModeParameterValue));
      break;
    default:
      strncpy(iotWebConfVirtualShiftingModeParameterValue, iotWebConfVirtualShiftingModeValues[0], sizeof(iotWebConfVirtualShiftingModeParameterValue));
      break;
  }
}

std::map<size_t, std::string> SettingsManager::getVirtualShiftingModes() {
  std::map<size_t, std::string> returnMap;
  for (size_t index = 0; index < (sizeof(iotWebConfVirtualShiftingModeValues) / sizeof(iotWebConfVirtualShiftingModeParameterValue)); index++)
  {
    returnMap.emplace(index, iotWebConfVirtualShiftingModeNames[index]);
  }
  return returnMap;
}

uint16_t SettingsManager::getChainringTeeth() {
  return atoi(iotWebConfChainringTeethParameterValue);
}

void SettingsManager::setChainringTeeth(uint16_t chainringTeeth) {
  std::string chainringTeethString = std::to_string(chainringTeeth);
  strncpy(iotWebConfChainringTeethParameterValue, chainringTeethString.c_str(), sizeof(iotWebConfChainringTeethParameterValue));
}

uint16_t SettingsManager::getSprocketTeeth() {
  return atoi(iotWebConfSprocketTeethParameterValue);
}

void SettingsManager::setSprocketTeeth(uint16_t sprocketTeeth) {
  std::string sprocketTeethString = std::to_string(sprocketTeeth);
  strncpy(iotWebConfSprocketTeethParameterValue, sprocketTeethString.c_str(), sizeof(iotWebConfSprocketTeethParameterValue));
}

bool SettingsManager::isVirtualShiftingEnabled() {
  return (strncmp(iotWebConfVirtualShiftingParameterValue, "selected", sizeof(iotWebConfVirtualShiftingParameterValue)) == 0);
}

void SettingsManager::setVirtualShiftingEnabled(bool enabled) {
  String virtualShiftingEnabled = "";
  if (enabled) {
    virtualShiftingEnabled = "selected";
  }
  strncpy(iotWebConfVirtualShiftingParameterValue, virtualShiftingEnabled.c_str(), sizeof(iotWebConfVirtualShiftingParameterValue));
}

std::string SettingsManager::getTrainerDeviceName() {
  return std::string(iotWebConfTrainerDeviceParameterValue);
}

void SettingsManager::setTrainerDeviceName(std::string trainerDevice) {
  strncpy(iotWebConfTrainerDeviceParameterValue, trainerDevice.c_str(), sizeof(iotWebConfTrainerDeviceParameterValue));
}

IotWebConfParameterGroup* SettingsManager::getIoTWebConfSettingsParameterGroup() {
  return &iotWebConfSettingsGroup;
}

std::string SettingsManager::getUsername() {
  return std::string(DEFAULT_USERNAME);
}

std::string SettingsManager::getAPPassword() {
  iotwebconf::Parameter* aPPasswordParameter = iotWebConf->getApPasswordParameter();
  std::string aPPassword = std::string(aPPasswordParameter->valueBuffer);
  if (aPPassword.length() == 0) {
    aPPassword = Utils::getHostName().c_str();
  }
  return aPPassword;
}

double SettingsManager::getGradeSmoothingSeconds() {
  // v1.5 and older stored a checkbox here ("selected" or ""); both become the new default.
  // Turning smoothing off now stores "0.0".
  if ((iotWebConfGradeSmoothingParameterValue[0] == 0) ||
      (strncmp(iotWebConfGradeSmoothingParameterValue, "selected", sizeof(iotWebConfGradeSmoothingParameterValue)) == 0)) {
    return DEFAULT_GRADE_SMOOTHING_SECONDS;
  }
  double seconds = atof(iotWebConfGradeSmoothingParameterValue);
  if (std::isnan(seconds) || seconds < 0) seconds = 0;
  if (seconds > MAX_GRADE_SMOOTHING_SECONDS) seconds = MAX_GRADE_SMOOTHING_SECONDS;
  return seconds;
}

void SettingsManager::setGradeSmoothingSeconds(double seconds) {
  if (std::isnan(seconds) || seconds < 0) seconds = 0;
  if (seconds > MAX_GRADE_SMOOTHING_SECONDS) seconds = MAX_GRADE_SMOOTHING_SECONDS;
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%.1f", seconds);
  strncpy(iotWebConfGradeSmoothingParameterValue, buffer, sizeof(iotWebConfGradeSmoothingParameterValue));
}

uint16_t SettingsManager::getDifficulty() {
  uint16_t difficulty = atoi(iotWebConfDifficultyParameterValue);
  if (difficulty < 1) {
    difficulty = 100;
  }
  return difficulty;
}

uint16_t SettingsManager::getTrainerWeight() {
  int weight = atoi(iotWebConfTrainerWeightParameterValue);
  if ((weight < 0) || (weight > 250)) {
    weight = 0;
  }
  return (uint16_t)weight;
}

void SettingsManager::setTrainerWeight(uint16_t weight) {
  std::string weightString = std::to_string(weight > 250 ? 0 : weight);
  strncpy(iotWebConfTrainerWeightParameterValue, weightString.c_str(), sizeof(iotWebConfTrainerWeightParameterValue));
}

void SettingsManager::setDifficulty(uint16_t difficulty) {
  std::string difficultyString = std::to_string(difficulty);
  strncpy(iotWebConfDifficultyParameterValue, difficultyString.c_str(), sizeof(iotWebConfDifficultyParameterValue));
}

bool SettingsManager::isFTMSEnabled() {
  return (strncmp(iotWebConfFTMSParameterValue, "selected", sizeof(iotWebConfFTMSParameterValue)) == 0);
}

void SettingsManager::setFTMSEnabled(bool enabled) {
  String fTMSEnabled = "";
  if (enabled) {
    fTMSEnabled = "selected";
  }
  strncpy(iotWebConfFTMSParameterValue, fTMSEnabled.c_str(), sizeof(iotWebConfFTMSParameterValue));
}

TrainerProtocolPreference SettingsManager::getTrainerProtocolPreference() {
  if (strncmp(iotWebConfTrainerProtocolParameterValue, iotWebConfTrainerProtocolValues[1], sizeof(iotWebConfTrainerProtocolParameterValue)) == 0) {
    return TrainerProtocolPreference::FTMS;
  }
  if (strncmp(iotWebConfTrainerProtocolParameterValue, iotWebConfTrainerProtocolValues[2], sizeof(iotWebConfTrainerProtocolParameterValue)) == 0) {
    return TrainerProtocolPreference::FEC;
  }
  return TrainerProtocolPreference::AUTO;
}

void SettingsManager::setTrainerProtocolPreference(TrainerProtocolPreference preference) {
  size_t index = (size_t)preference;
  if (index > 2) {
    index = 0;
  }
  strncpy(iotWebConfTrainerProtocolParameterValue, iotWebConfTrainerProtocolValues[index], sizeof(iotWebConfTrainerProtocolParameterValue));
}

std::map<size_t, std::string> SettingsManager::getTrainerProtocolPreferences() {
  std::map<size_t, std::string> returnMap;
  for (size_t index = 0; index < (sizeof(iotWebConfTrainerProtocolValues) / sizeof(iotWebConfTrainerProtocolParameterValue)); index++) {
    returnMap.emplace(index, iotWebConfTrainerProtocolNames[index]);
  }
  return returnMap;
}

VirtualGears::GearTable SettingsManager::getGearTable() {
  if (strncmp(iotWebConfGearTableParameterValue, iotWebConfGearTableValues[0], sizeof(iotWebConfGearTableParameterValue)) == 0) {
    return VirtualGears::GearTable::ZWIFT;
  }
  // anything else, including the v1.5 value "axs_50_10_32", is the AXS table
  return VirtualGears::GearTable::AXS_50_36_10_32;
}

void SettingsManager::setGearTable(VirtualGears::GearTable gearTable) {
  size_t index = (size_t)gearTable;
  if (index > 1) {
    index = 1;
  }
  strncpy(iotWebConfGearTableParameterValue, iotWebConfGearTableValues[index], sizeof(iotWebConfGearTableParameterValue));
}

std::map<size_t, std::string> SettingsManager::getGearTables() {
  std::map<size_t, std::string> returnMap;
  for (size_t index = 0; index < (sizeof(iotWebConfGearTableValues) / sizeof(iotWebConfGearTableParameterValue)); index++) {
    returnMap.emplace(index, iotWebConfGearTableNames[index]);
  }
  return returnMap;
}
