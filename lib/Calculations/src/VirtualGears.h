#ifndef VIRTUALGEARS_H
#define VIRTUALGEARS_H

// Maps the gear Zwift selects (it only ever sends one of its 24 ratios) onto the
// drivetrain the bridge should simulate. Pure C++ so it can be unit tested on a PC.

#include <cstddef>
#include <string>

namespace VirtualGears {

enum class GearTable : unsigned char {
  ZWIFT = 0,           // Zwift's own 24 gears, 0.75 .. 5.49
  AXS_50_36_10_32 = 1  // SRAM AXS 2x12: 50/36 chainrings, 10-32 cassette, all 24 combinations
};

constexpr size_t ZWIFT_GEAR_COUNT = 24;
extern const double ZWIFT_RATIOS[ZWIFT_GEAR_COUNT];

struct ChainringCog {
  int chainring;
  int cog;
};
constexpr size_t AXS_GEAR_COUNT = 24;
// every chainring/cog combination, easiest (36x32) to hardest (50x10), so Zwift gear n = AXS gear n
extern const ChainringCog AXS_GEARS[AXS_GEAR_COUNT];

// 1-based index of the Zwift gear closest to ratio
int zwiftGearIndex(double ratio);

class GearMapper {
 public:
  void setTable(GearTable table);
  GearTable getTable() const { return table; }
  void reset();
  // feed the ratio Zwift sent, returns the ratio to simulate
  double map(double zwiftRatio);

  int getGear() const { return gear; }          // 1-based, 0 = none yet
  int getGearCount() const { return 24; }
  double getRatio() const { return ratio; }
  // e.g. "36x15 (11/24)" or "12/24"
  std::string getLabel() const;

 private:
  GearTable table = GearTable::ZWIFT;
  int gear = 0;
  double ratio = 0;
};

}  // namespace VirtualGears

#endif
