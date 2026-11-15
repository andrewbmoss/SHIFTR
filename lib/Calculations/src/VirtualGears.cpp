#include <VirtualGears.h>

#include <cmath>
#include <cstdio>

namespace VirtualGears {

const double ZWIFT_RATIOS[ZWIFT_GEAR_COUNT] = {0.75, 0.87, 0.99, 1.11, 1.23, 1.38, 1.53, 1.68, 1.86, 2.04, 2.22, 2.40,
                                               2.61, 2.82, 3.03, 3.24, 3.49, 3.74, 3.99, 4.24, 4.54, 4.84, 5.14, 5.49};

const int AXS_COGS[AXS_GEAR_COUNT] = {32, 28, 24, 21, 19, 17, 15, 14, 13, 12, 11, 10};

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

static int clampGear(int value, int count) {
  if (value < 1) return 1;
  if (value > count) return count;
  return value;
}

void GearMapper::setTable(GearTable newTable) {
  if (newTable != table) {
    table = newTable;
    reset();
  }
}

void GearMapper::reset() {
  tracking = false;
  lastZwiftGear = 0;
  gear = 0;
  ratio = 0;
}

int GearMapper::getGearCount() const {
  return (table == GearTable::AXS_50_10_32) ? (int)AXS_GEAR_COUNT : (int)ZWIFT_GEAR_COUNT;
}

double GearMapper::map(double zwiftRatio) {
  if (zwiftRatio <= 0) {
    return zwiftRatio;
  }
  int zwiftGear = zwiftGearIndex(zwiftRatio);
  if (table == GearTable::ZWIFT) {
    gear = zwiftGear;
    ratio = zwiftRatio;
    return ratio;
  }
  // AXS: follow Zwift's up/down steps one cog at a time, stopping at the ends of the cassette.
  // Tracking the steps (instead of a fixed Zwift->AXS table) means a shift in the opposite
  // direction always acts immediately, even after over-shifting past an end.
  if (!tracking) {
    gear = clampGear(zwiftGear - AXS_ZWIFT_OFFSET, (int)AXS_GEAR_COUNT);
    tracking = true;
  } else {
    gear = clampGear(gear + (zwiftGear - lastZwiftGear), (int)AXS_GEAR_COUNT);
  }
  lastZwiftGear = zwiftGear;
  ratio = (double)AXS_CHAINRING / AXS_COGS[gear - 1];
  return ratio;
}

std::string GearMapper::getLabel() const {
  char buffer[32];
  if (gear == 0) {
    return "-";
  }
  if (table == GearTable::AXS_50_10_32) {
    snprintf(buffer, sizeof(buffer), "%dx%d (%d/%d)", AXS_CHAINRING, AXS_COGS[gear - 1], gear, (int)AXS_GEAR_COUNT);
  } else {
    snprintf(buffer, sizeof(buffer), "%d/%d", gear, (int)ZWIFT_GEAR_COUNT);
  }
  return std::string(buffer);
}

}  // namespace VirtualGears
