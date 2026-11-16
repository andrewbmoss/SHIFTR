#include <VirtualGears.h>

#include <cmath>
#include <cstdio>

namespace VirtualGears {

const double ZWIFT_RATIOS[ZWIFT_GEAR_COUNT] = {0.75, 0.87, 0.99, 1.11, 1.23, 1.38, 1.53, 1.68, 1.86, 2.04, 2.22, 2.40,
                                               2.61, 2.82, 3.03, 3.24, 3.49, 3.74, 3.99, 4.24, 4.54, 4.84, 5.14, 5.49};

// sorted by ratio: 1.125 1.286 1.500 1.563 1.714 1.786 1.895 2.083 2.118 2.381 2.400 2.571
//                  2.632 2.769 2.941 3.000 3.273 3.333 3.571 3.600 3.846 4.167 4.545 5.000
const ChainringCog AXS_GEARS[AXS_GEAR_COUNT] = {
    {36, 32}, {36, 28}, {36, 24}, {50, 32}, {36, 21}, {50, 28}, {36, 19}, {50, 24},
    {36, 17}, {50, 21}, {36, 15}, {36, 14}, {50, 19}, {36, 13}, {50, 17}, {36, 12},
    {36, 11}, {50, 15}, {50, 14}, {36, 10}, {50, 13}, {50, 12}, {50, 11}, {50, 10}};

int zwiftGearIndex(double ratio) {
  int best = 1;
  double bestDifference = 1e9;
  for (size_t index = 0; index < ZWIFT_GEAR_COUNT; index++) {
    double difference = std::fabs(ZWIFT_RATIOS[index] - ratio);
    if (difference < bestDifference) {
      bestDifference = difference;
      best = (int)index + 1;
    }
  }
  return best;
}

void GearMapper::setTable(GearTable newTable) {
  if (newTable != table) {
    table = newTable;
    reset();
  }
}

void GearMapper::reset() {
  gear = 0;
  ratio = 0;
}

double GearMapper::map(double zwiftRatio) {
  if (zwiftRatio <= 0) {
    return zwiftRatio;
  }
  // both tables have 24 gears: Zwift gear n is simulated as gear n of the active table
  gear = zwiftGearIndex(zwiftRatio);
  if (table == GearTable::AXS_50_36_10_32) {
    ratio = (double)AXS_GEARS[gear - 1].chainring / AXS_GEARS[gear - 1].cog;
  } else {
    ratio = zwiftRatio;
  }
  return ratio;
}

std::string GearMapper::getLabel() const {
  char buffer[32];
  if (gear == 0) {
    return "-";
  }
  if (table == GearTable::AXS_50_36_10_32) {
    snprintf(buffer, sizeof(buffer), "%dx%d (%d/24)", AXS_GEARS[gear - 1].chainring, AXS_GEARS[gear - 1].cog, gear);
  } else {
    snprintf(buffer, sizeof(buffer), "%d/24", gear);
  }
  return std::string(buffer);
}

}  // namespace VirtualGears
