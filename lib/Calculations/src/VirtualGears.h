#ifndef VIRTUALGEARS_H
#define VIRTUALGEARS_H

// Maps the gear Zwift selects (it only ever sends one of its 24 ratios) onto the
// drivetrain the bridge should simulate. Pure C++ so it can be unit tested on a PC.

#include <cstddef>
#include <string>

namespace VirtualGears {

enum class GearTable : unsigned char {
  ZWIFT = 0,        // Zwift's own 24 gears, 0.75 .. 5.49
  AXS_50_10_32 = 1  // SRAM AXS 1x: 50T chainring, 10-32 12-speed cassette
};

constexpr size_t ZWIFT_GEAR_COUNT = 24;
extern const double ZWIFT_RATIOS[ZWIFT_GEAR_COUNT];

constexpr size_t AXS_GEAR_COUNT = 12;
constexpr int AXS_CHAINRING = 50;
extern const int AXS_COGS[AXS_GEAR_COUNT];   // easiest (gear 1) to hardest (gear 12)
// Zwift gear that maps onto AXS gear 1 when tracking starts (Zwift 7..18 -> AXS 1..12)
constexpr int AXS_ZWIFT_OFFSET = 6;

// 1-based index of the Zwift gear closest to ratio
int zwiftGearIndex(double ratio);

class GearMapper {
 public:
  void setTable(GearTable table);
  GearTable getTable() const { return table; }
  // forget the tracked position (new Zwift session)
  void reset();
  // feed the ratio Zwift sent, returns the ratio to simulate
  double map(double zwiftRatio);

  int getGear() const { return gear; }          // 1-based gear in the active table, 0 = none yet
  int getGearCount() const;
  double getRatio() const { return ratio; }
  // e.g. "50x15 (6/12)" or "12/24"
  std::string getLabel() const;

 private:
  GearTable table = GearTable::ZWIFT;
  bool tracking = false;
  int lastZwiftGear = 0;
  int gear = 0;
  double ratio = 0;
};

}  // namespace VirtualGears

#endif
