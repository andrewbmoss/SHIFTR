#include <Telemetry.h>
#include <BTDeviceManager.h>
#include <DirConManager.h>
#include <PowerMeter.h>
#include <TrainerControl.h>
#include <Version.h>
#include <esp_heap_caps.h>
#include <esp_system.h>

#include <cstdarg>

namespace {

constexpr size_t SAMPLE_CAPACITY = 240;  // 4 minutes at 1 Hz
constexpr size_t EVENT_CAPACITY = 96;
constexpr size_t EVENT_DETAIL = 39;
constexpr size_t MAX_ROWS_PER_RESPONSE = 120;
constexpr uint32_t SAMPLE_INTERVAL_MS = 1000;

#pragma pack(push, 1)
struct Sample {
  uint32_t seq;
  uint32_t uptimeMs;
  uint8_t trainerFlags;  // bit0 connected, bit1 cadence measured, bit2 FTMS control
  int16_t trainerPower;
  uint16_t trainerCadence10;
  uint16_t trainerSpeed100;  // km/h x 100
  int8_t trainerRssi;
  uint16_t trainerAgeMs;
  uint8_t packetsFtms, packetsCps, packetsFec, packetsPm;  // received during this second
  uint8_t powerMeterConnected;
  int16_t powerMeterPower;
  uint16_t powerMeterCadence10;
  uint8_t powerMeterBalance2;  // 0xFF = none
  int8_t powerMeterRssi;
  uint16_t powerMeterAgeMs;
  int16_t outputPower;
  uint16_t outputCadence10;
  uint8_t outputSource;  // 0 trainer
  uint8_t mode;          // 0 SIM, 1 SIM + virtual shifting, 2 ERG
  uint8_t zwiftClients;
  int16_t grade100;
  int16_t smoothedGrade100;
  uint8_t gear;
  uint16_t gearRatio1000;
  uint16_t ergTarget;
  int16_t modelPower;
  int16_t dragPower;
  uint16_t trainerCw100;
  int16_t residualDrag10;
  uint8_t commandType;
  float commandValue;
  uint16_t commandCw100;
  uint16_t blockedWrites;
  uint32_t freeHeap;
  uint32_t minFreeHeap;
  uint32_t largestBlock;
};

struct Event {
  uint32_t seq;
  uint32_t uptimeMs;
  uint8_t type;
  char detail[EVENT_DETAIL + 1];
};
#pragma pack(pop)

const char* const EVENT_NAMES[] = {"boot", "marker", "trainer_connected", "trainer_disconnected", "control_granted", "control_refused",
                                   "control_lost", "command_failed", "zwift_connected", "zwift_disconnected", "mode", "shift",
                                   "erg_target", "settings_saved", "probe", "logger", "power_meter"};
static_assert(sizeof(EVENT_NAMES) / sizeof(EVENT_NAMES[0]) == (size_t)TelemetryEvent::COUNT, "event names");

const char* const SAMPLE_COLUMNS =
    "seq,uptime_ms,tr_connected,tr_power,tr_cadence,tr_cadence_measured,tr_speed_kph,tr_rssi,tr_age_ms,tr_control,"
    "pkt_ftms,pkt_cps,pkt_fec,pm_connected,pm_power,pm_cadence,pm_balance,pm_rssi,pm_age_ms,pkt_pm,"
    "out_power,out_cadence,out_source,mode,zwift_clients,grade,smoothed_grade,gear,gear_ratio,erg_target,"
    "model_power,drag_power,trainer_cw,residual_drag,cmd_type,cmd_value,cmd_cw,blocked_writes,free_heap,min_free_heap,largest_block";
const char* const EVENT_COLUMNS = "seq,uptime_ms,event,detail";
const char* const MODE_NAMES[] = {"SIM", "SIM+VS", "ERG"};
const char* const COMMAND_NAMES[] = {"none", "grade", "target_power", "resistance"};
const char* const SOURCE_NAMES[] = {"trainer", "power_meter", "trainer_corrected"};

Sample* samples = nullptr;
Event* events = nullptr;
uint32_t nextSampleSeq = 1;
uint32_t nextEventSeq = 1;
uint32_t bootId = 0;
uint32_t lastSampleAt = 0;
uint32_t lastFetchAt = 0;
uint32_t lastPackets[3] = {0, 0, 0};
uint32_t lastPowerMeterPackets = 0;
uint8_t lastMode = 0xFF;
portMUX_TYPE eventLock = portMUX_INITIALIZER_UNLOCKED;
// events that arrive before begin() (or if allocation failed) are dropped

template <typename T>
T clampTo(double value) {
  double low = (double)std::numeric_limits<T>::min();
  double high = (double)std::numeric_limits<T>::max();
  if (std::isnan(value)) return 0;
  if (value < low) return std::numeric_limits<T>::min();
  if (value > high) return std::numeric_limits<T>::max();
  return (T)lround(value);
}

uint8_t modeIndex() {
  switch (DirConManager::getZwiftTrainerMode()) {
    case TrainerMode::SIM_MODE_VIRTUAL_SHIFTING:
      return 1;
    case TrainerMode::ERG_MODE:
      return 2;
    default:
      return 0;
  }
}

void takeSample(uint32_t now) {
  Sample& sample = samples[nextSampleSeq % SAMPLE_CAPACITY];
  memset(&sample, 0, sizeof(sample));
  sample.seq = nextSampleSeq;
  sample.uptimeMs = now;

  double defaultGearRatio = DirConManager::getDefaultGearRatio();
  bool trainerConnected = BTDeviceManager::isConnected();
  sample.trainerFlags = (trainerConnected ? 1 : 0) | (TrainerControl::isCadenceMeasured() ? 2 : 0) | (TrainerControl::hasControl() ? 4 : 0);
  sample.trainerPower = clampTo<int16_t>(TrainerControl::getPower());
  uint8_t cadence = TrainerControl::getEffectiveCadence(defaultGearRatio);
  sample.trainerCadence10 = cadence * 10;
  sample.trainerSpeed100 = clampTo<uint16_t>(TrainerControl::getSpeedMps() * 360.0);
  sample.trainerRssi = clampTo<int8_t>(trainerConnected ? BTDeviceManager::getRssi() : 0);
  uint32_t age = TrainerControl::getMillisSinceLastData();
  sample.trainerAgeMs = age > 65535 ? 65535 : age;

  uint32_t packets[3];
  TrainerControl::getPacketCounts(packets[0], packets[1], packets[2]);
  uint8_t* deltas[3] = {&sample.packetsFtms, &sample.packetsCps, &sample.packetsFec};
  for (int index = 0; index < 3; index++) {
    // counters restart at 0 on every trainer connection
    uint32_t delta = (packets[index] >= lastPackets[index]) ? packets[index] - lastPackets[index] : packets[index];
    *deltas[index] = delta > 255 ? 255 : delta;
    lastPackets[index] = packets[index];
  }

  PowerMeterReading meter = PowerMeter::getReading();
  sample.powerMeterConnected = meter.connected ? 1 : 0;
  sample.powerMeterPower = meter.power;
  sample.powerMeterCadence10 = clampTo<uint16_t>(meter.cadence * 10.0);
  sample.powerMeterBalance2 = (meter.connected && meter.hasBalance) ? clampTo<uint8_t>(meter.balanceLeft * 2.0) : 0xFF;
  sample.powerMeterRssi = clampTo<int8_t>(meter.connected ? meter.rssi : 0);
  sample.powerMeterAgeMs = meter.ageMs > 65535 ? 65535 : meter.ageMs;
  uint32_t meterDelta = meter.packets - lastPowerMeterPackets;
  sample.packetsPm = meterDelta > 255 ? 255 : meterDelta;
  lastPowerMeterPackets = meter.packets;

  // what Zwift receives: the trainer's values in v1 behaviour
  sample.outputPower = sample.trainerPower;
  sample.outputCadence10 = sample.trainerCadence10;
  sample.outputSource = 0;

  sample.mode = modeIndex();
  sample.zwiftClients = DirConManager::getConnectedClientCount();
  sample.grade100 = clampTo<int16_t>(DirConManager::getZwiftGradePercent() * 100.0);
  sample.smoothedGrade100 = clampTo<int16_t>(DirConManager::getSmoothedGradePercent() * 100.0);
  sample.gear = DirConManager::getVirtualGearIndex();
  sample.gearRatio1000 = clampTo<uint16_t>(DirConManager::getSimulatedGearRatio() * 1000.0);
  sample.ergTarget = DirConManager::getErgTargetPower();
  sample.modelPower = clampTo<int16_t>(DirConManager::getModelPower());
  sample.dragPower = clampTo<int16_t>(DirConManager::getTrainerDragPower());
  sample.trainerCw100 = clampTo<uint16_t>(DirConManager::getTrainerWindCoefficient() * 100.0);
  sample.residualDrag10 = clampTo<int16_t>(DirConManager::getResidualDragForce() * 10.0);
  TrainerControl::CommandSnapshot command = TrainerControl::getLastSentCommand();
  sample.commandType = command.type;
  sample.commandValue = (float)command.value1;
  sample.commandCw100 = clampTo<uint16_t>(command.value3 * 100.0);
  sample.blockedWrites = clampTo<uint16_t>(DirConManager::getBlockedPassthroughWrites());
  sample.freeHeap = ESP.getFreeHeap();
  sample.minFreeHeap = ESP.getMinFreeHeap();
  sample.largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

  nextSampleSeq++;

  if (sample.mode != lastMode) {
    if (lastMode != 0xFF) {
      Telemetry::event(TelemetryEvent::MODE, MODE_NAMES[sample.mode]);
    }
    lastMode = sample.mode;
  }
}

int formatSample(char* buffer, size_t size, const Sample& s) {
  char balance[8] = "";
  if (s.powerMeterBalance2 != 0xFF) snprintf(balance, sizeof(balance), "%.1f", s.powerMeterBalance2 / 2.0);
  char commandValue[16];
  snprintf(commandValue, sizeof(commandValue), s.commandType == 1 ? "%.2f" : "%.0f", s.commandValue);
  return snprintf(buffer, size,
                  "S,%lu,%lu,%u,%d,%.1f,%u,%.2f,%d,%u,%u,"
                  "%u,%u,%u,%u,%s,%s,%s,%s,%s,%u,"
                  "%d,%.1f,%s,%s,%u,%.2f,%.2f,%u,%.3f,%u,"
                  "%d,%d,%.2f,%.1f,%s,%s,%.2f,%u,%lu,%lu,%lu\n",
                  (unsigned long)s.seq, (unsigned long)s.uptimeMs, s.trainerFlags & 1, s.trainerPower, s.trainerCadence10 / 10.0,
                  (s.trainerFlags >> 1) & 1, s.trainerSpeed100 / 100.0, s.trainerRssi, s.trainerAgeMs, (s.trainerFlags >> 2) & 1,
                  s.packetsFtms, s.packetsCps, s.packetsFec, s.powerMeterConnected,
                  s.powerMeterConnected ? String(s.powerMeterPower).c_str() : "", s.powerMeterConnected ? String(s.powerMeterCadence10 / 10.0, 1).c_str() : "",
                  balance, s.powerMeterConnected ? String(s.powerMeterRssi).c_str() : "", s.powerMeterConnected ? String(s.powerMeterAgeMs).c_str() : "", s.packetsPm,
                  s.outputPower, s.outputCadence10 / 10.0, SOURCE_NAMES[s.outputSource < 3 ? s.outputSource : 0], MODE_NAMES[s.mode < 3 ? s.mode : 0],
                  s.zwiftClients, s.grade100 / 100.0, s.smoothedGrade100 / 100.0, s.gear, s.gearRatio1000 / 1000.0, s.ergTarget,
                  s.modelPower, s.dragPower, s.trainerCw100 / 100.0, s.residualDrag10 / 10.0, COMMAND_NAMES[s.commandType < 4 ? s.commandType : 0],
                  commandValue, s.commandCw100 / 100.0, s.blockedWrites, (unsigned long)s.freeHeap, (unsigned long)s.minFreeHeap,
                  (unsigned long)s.largestBlock);
}

uint32_t parseSeq(WebServer& server, const char* name) {
  if (!server.hasArg(name)) return 0;
  return (uint32_t)strtoul(server.arg(name).c_str(), nullptr, 10);
}

// buffered chunked writer: keeps each chunk ~1.4 KB so we never build the whole response in RAM
class ChunkWriter {
 public:
  explicit ChunkWriter(WebServer& server) : server(server) {}
  void write(const char* text, size_t length) {
    if (used + length >= sizeof(buffer)) flush();
    if (length >= sizeof(buffer)) {
      server.sendContent(text, length);
      return;
    }
    memcpy(buffer + used, text, length);
    used += length;
  }
  void flush() {
    if (used > 0) {
      server.sendContent(buffer, used);
      used = 0;
    }
  }

 private:
  WebServer& server;
  char buffer[1400];
  size_t used = 0;
};

}  // namespace

bool Telemetry::begin() {
  bootId = esp_random();
  samples = (Sample*)heap_caps_calloc(SAMPLE_CAPACITY, sizeof(Sample), MALLOC_CAP_8BIT);
  events = (Event*)heap_caps_calloc(EVENT_CAPACITY, sizeof(Event), MALLOC_CAP_8BIT);
  if ((samples == nullptr) || (events == nullptr)) {
    log_e("Telemetry: couldn't allocate %u + %u bytes", (unsigned)(SAMPLE_CAPACITY * sizeof(Sample)), (unsigned)(EVENT_CAPACITY * sizeof(Event)));
    free(samples);
    free(events);
    samples = nullptr;
    events = nullptr;
    return false;
  }
  log_i("Telemetry: %u samples x %u bytes, %u events x %u bytes", (unsigned)SAMPLE_CAPACITY, (unsigned)sizeof(Sample), (unsigned)EVENT_CAPACITY, (unsigned)sizeof(Event));
  static const char* const resetReasons[] = {"unknown", "power-on", "external", "software", "panic", "interrupt watchdog", "task watchdog",
                                             "other watchdog", "deep sleep", "brownout", "SDIO"};
  esp_reset_reason_t reason = esp_reset_reason();
  eventf(TelemetryEvent::BOOT, "%s reset %s", VERSION, (reason < 11) ? resetReasons[reason] : "?");
  return true;
}

void Telemetry::update() {
  if (samples == nullptr) return;
  uint32_t now = millis();
  if ((lastSampleAt == 0) || (now - lastSampleAt >= SAMPLE_INTERVAL_MS)) {
    // keep a steady 1 s grid even if the loop was late
    lastSampleAt = (lastSampleAt == 0) ? now : lastSampleAt + SAMPLE_INTERVAL_MS;
    if (now - lastSampleAt > SAMPLE_INTERVAL_MS) lastSampleAt = now;
    takeSample(now);
  }
}

void Telemetry::event(TelemetryEvent type, const char* detail) {
  if (events == nullptr) return;
  uint32_t now = millis();
  portENTER_CRITICAL(&eventLock);
  Event& entry = events[nextEventSeq % EVENT_CAPACITY];
  entry.seq = nextEventSeq++;
  entry.uptimeMs = now;
  entry.type = (uint8_t)type;
  size_t length = 0;
  if (detail != nullptr) {
    for (; (detail[length] != 0) && (length < EVENT_DETAIL); length++) {
      char c = detail[length];
      // keep the CSV simple: no commas, quotes or line breaks inside a field
      entry.detail[length] = (c == ',') ? ';' : ((c == '"') || (c == '\n') || (c == '\r')) ? ' ' : c;
    }
  }
  entry.detail[length] = 0;
  portEXIT_CRITICAL(&eventLock);
}

void Telemetry::eventf(TelemetryEvent type, const char* format, ...) {
  char buffer[EVENT_DETAIL + 1];
  va_list arguments;
  va_start(arguments, format);
  vsnprintf(buffer, sizeof(buffer), format, arguments);
  va_end(arguments);
  event(type, buffer);
}

void Telemetry::handleInfo(WebServer& server) {
  char buffer[1024];
  uint32_t oldestSample = (nextSampleSeq > SAMPLE_CAPACITY) ? nextSampleSeq - SAMPLE_CAPACITY : 1;
  uint32_t oldestEvent = (nextEventSeq > EVENT_CAPACITY) ? nextEventSeq - EVENT_CAPACITY : 1;
  snprintf(buffer, sizeof(buffer),
           "{\"version\": \"%s\", \"boot_id\": \"%08lx\", \"uptime_ms\": %lu, \"available\": %s, \"sample_interval_ms\": %lu, "
           "\"sample_capacity\": %u, \"event_capacity\": %u, \"sample_next\": %lu, \"sample_oldest\": %lu, \"event_next\": %lu, \"event_oldest\": %lu, "
           "\"sample_columns\": \"%s\", \"event_columns\": \"%s\"}",
           VERSION, (unsigned long)bootId, (unsigned long)millis(), samples != nullptr ? "true" : "false", (unsigned long)SAMPLE_INTERVAL_MS,
           (unsigned)SAMPLE_CAPACITY, (unsigned)EVENT_CAPACITY, (unsigned long)nextSampleSeq, (unsigned long)oldestSample,
           (unsigned long)nextEventSeq, (unsigned long)oldestEvent, SAMPLE_COLUMNS, EVENT_COLUMNS);
  server.send(200, "application/json", buffer);
}

// GET /log?s=<next sample seq wanted>&e=<next event seq wanted>
// Text lines:
//   U,<uptime_ms>,<boot_id>,<sample_next>,<event_next>
//   G,S|E,<first wanted>,<first available>   rows were overwritten before they were fetched
//   S,<sample columns>
//   E,<event columns>
//   M,<1 if more rows are waiting>
void Telemetry::handleLog(WebServer& server) {
  if ((samples == nullptr) || (events == nullptr)) {
    server.send(503, "text/plain", "telemetry not available\n");
    return;
  }
  lastFetchAt = millis();
  uint32_t wantSample = parseSeq(server, "s");
  uint32_t wantEvent = parseSeq(server, "e");
  uint32_t sampleNext = nextSampleSeq;
  portENTER_CRITICAL(&eventLock);
  uint32_t eventNext = nextEventSeq;
  portEXIT_CRITICAL(&eventLock);
  uint32_t oldestSample = (sampleNext > SAMPLE_CAPACITY) ? sampleNext - SAMPLE_CAPACITY : 1;
  // the oldest event slot may be overwritten while we read, so leave one slot of margin
  uint32_t oldestEvent = (eventNext > EVENT_CAPACITY - 1) ? eventNext - (EVENT_CAPACITY - 1) : 1;

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "");
  ChunkWriter writer(server);
  char line[400];
  int length = snprintf(line, sizeof(line), "U,%lu,%08lx,%lu,%lu\n", (unsigned long)millis(), (unsigned long)bootId, (unsigned long)sampleNext, (unsigned long)eventNext);
  writer.write(line, length);

  // a request from before a reboot (or a fresh start with 0) starts at the oldest row we have
  if ((wantSample == 0) || (wantSample > sampleNext)) wantSample = oldestSample;
  if ((wantEvent == 0) || (wantEvent > eventNext)) wantEvent = oldestEvent;
  if (wantSample < oldestSample) {
    length = snprintf(line, sizeof(line), "G,S,%lu,%lu\n", (unsigned long)wantSample, (unsigned long)oldestSample);
    writer.write(line, length);
    wantSample = oldestSample;
  }
  if (wantEvent < oldestEvent) {
    length = snprintf(line, sizeof(line), "G,E,%lu,%lu\n", (unsigned long)wantEvent, (unsigned long)oldestEvent);
    writer.write(line, length);
    wantEvent = oldestEvent;
  }

  bool more = false;
  size_t rows = 0;
  for (uint32_t seq = wantSample; seq < sampleNext; seq++) {
    if (rows >= MAX_ROWS_PER_RESPONSE) {
      more = true;
      break;
    }
    const Sample& sample = samples[seq % SAMPLE_CAPACITY];
    if (sample.seq != seq) continue;
    length = formatSample(line, sizeof(line), sample);
    if (length > 0) writer.write(line, (size_t)length < sizeof(line) ? length : sizeof(line) - 1);
    rows++;
  }
  rows = 0;
  for (uint32_t seq = wantEvent; seq < eventNext; seq++) {
    if (rows >= MAX_ROWS_PER_RESPONSE) {
      more = true;
      break;
    }
    Event entry;
    portENTER_CRITICAL(&eventLock);
    entry = events[seq % EVENT_CAPACITY];
    portEXIT_CRITICAL(&eventLock);
    if (entry.seq != seq) continue;
    const char* name = (entry.type < (uint8_t)TelemetryEvent::COUNT) ? EVENT_NAMES[entry.type] : "?";
    length = snprintf(line, sizeof(line), "E,%lu,%lu,%s,%s\n", (unsigned long)entry.seq, (unsigned long)entry.uptimeMs, name, entry.detail);
    writer.write(line, length);
    rows++;
  }
  length = snprintf(line, sizeof(line), "M,%d\n", more ? 1 : 0);
  writer.write(line, length);
  writer.flush();
  server.sendContent("");
}

void Telemetry::handleMarker(WebServer& server) {
  String text = server.arg("text");
  if (text.length() == 0) text = "marker";
  event(TelemetryEvent::MARKER, text.c_str());
  server.send(200, "application/json", "{\"ok\": true}");
}

uint32_t Telemetry::getMillisSinceLastFetch() {
  return (lastFetchAt == 0) ? UINT32_MAX : millis() - lastFetchAt;
}

size_t Telemetry::getSampleCapacity() { return SAMPLE_CAPACITY; }
