#include <CrashReport.h>
#include <Telemetry.h>
#include <esp_core_dump.h>
#include <esp_flash.h>
#include <esp_system.h>

namespace {

const char* exceptionName(uint32_t cause) {
  switch (cause) {
    case 0: return "IllegalInstruction";
    case 2: return "InstructionFetchError";
    case 3: return "LoadStoreError";
    case 4: return "Level1Interrupt";
    case 6: return "IntegerDivideByZero";
    case 9: return "LoadStoreAlignment";
    case 20: return "InstFetchProhibited";
    case 28: return "LoadProhibited";
    case 29: return "StoreProhibited";
    default: return "other";
  }
}

const char* resetName(esp_reset_reason_t reason) {
  static const char* const names[] = {"unknown", "power-on", "external", "software", "panic", "interrupt watchdog", "task watchdog",
                                      "other watchdog", "deep sleep", "brownout", "SDIO"};
  return (reason < 11) ? names[reason] : "?";
}

}  // namespace

bool CrashReport::hasDump() {
  size_t address = 0, size = 0;
  return esp_core_dump_image_get(&address, &size) == ESP_OK;
}

void CrashReport::begin() {
  esp_reset_reason_t reason = esp_reset_reason();
  if (!hasDump()) {
    if ((reason == ESP_RST_PANIC) || (reason == ESP_RST_INT_WDT) || (reason == ESP_RST_TASK_WDT) || (reason == ESP_RST_WDT)) {
      Telemetry::eventf(TelemetryEvent::BOOT, "crash (%s), no core dump", resetName(reason));
    }
    return;
  }
  esp_core_dump_summary_t* summary = (esp_core_dump_summary_t*)malloc(sizeof(esp_core_dump_summary_t));
  if (summary == nullptr) return;
  if (esp_core_dump_get_summary(summary) == ESP_OK) {
    char task[17];
    memcpy(task, summary->exc_task, 16);
    task[16] = 0;
    log_e("Core dump in flash: task %s, PC 0x%08lx, cause %lu", task, (unsigned long)summary->exc_pc, (unsigned long)summary->ex_info.exc_cause);
    Telemetry::eventf(TelemetryEvent::BOOT, "core dump: %s pc %08lx", task, (unsigned long)summary->exc_pc);
  }
  free(summary);
}

void CrashReport::handleSummary(WebServer& server) {
  String json = "{\"reset_reason\": \"" + String(resetName(esp_reset_reason())) + "\"";
  size_t address = 0, size = 0;
  if (esp_core_dump_image_get(&address, &size) != ESP_OK) {
    json += ", \"present\": false}";
    server.send(200, "application/json", json);
    return;
  }
  json += ", \"present\": true, \"size\": " + String((unsigned long)size);
  esp_core_dump_summary_t* summary = (esp_core_dump_summary_t*)malloc(sizeof(esp_core_dump_summary_t));
  if ((summary != nullptr) && (esp_core_dump_get_summary(summary) == ESP_OK)) {
    char task[17];
    memcpy(task, summary->exc_task, 16);
    task[16] = 0;
    char elf[APP_ELF_SHA256_SZ + 1];
    memcpy(elf, summary->app_elf_sha256, APP_ELF_SHA256_SZ);
    elf[APP_ELF_SHA256_SZ] = 0;
    char buffer[48];
    json += ", \"task\": \"" + String(task) + "\"";
    snprintf(buffer, sizeof(buffer), "0x%08lx", (unsigned long)summary->exc_pc);
    json += ", \"pc\": \"" + String(buffer) + "\"";
    json += ", \"exception\": \"" + String(exceptionName(summary->ex_info.exc_cause)) + "\", \"exception_cause\": " + String((unsigned long)summary->ex_info.exc_cause);
    snprintf(buffer, sizeof(buffer), "0x%08lx", (unsigned long)summary->ex_info.exc_vaddr);
    json += ", \"address\": \"" + String(buffer) + "\"";
    json += ", \"app_elf_sha256\": \"" + String(elf) + "\"";
    json += ", \"backtrace_corrupted\": " + String(summary->exc_bt_info.corrupted ? "true" : "false") + ", \"backtrace\": \"";
    for (uint32_t index = 0; (index < summary->exc_bt_info.depth) && (index < 16); index++) {
      snprintf(buffer, sizeof(buffer), "%s0x%08lx", index ? " " : "", (unsigned long)summary->exc_bt_info.bt[index]);
      json += buffer;
    }
    json += "\"";
  } else {
    json += ", \"summary\": \"unreadable\"";
  }
  free(summary);
  json += "}";
  server.send(200, "application/json", json);
}

void CrashReport::handleDump(WebServer& server) {
  size_t address = 0, size = 0;
  if (esp_core_dump_image_get(&address, &size) != ESP_OK) {
    server.send(404, "text/plain", "no core dump\n");
    return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=\"shiftr-coredump.elf\"");
  server.setContentLength(size);
  server.send(200, "application/octet-stream", "");
  uint8_t buffer[1024];
  for (size_t offset = 0; offset < size; offset += sizeof(buffer)) {
    size_t length = (size - offset < sizeof(buffer)) ? size - offset : sizeof(buffer);
    if (esp_flash_read(nullptr, buffer, address + offset, length) != ESP_OK) break;
    server.sendContent((const char*)buffer, length);
  }
}

void CrashReport::handleClear(WebServer& server) {
  esp_core_dump_image_erase();
  server.send(200, "application/json", "{\"ok\": true}");
}
