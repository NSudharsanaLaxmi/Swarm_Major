#include "DisplayManager.h"

DisplayManager::DisplayManager() : display(128, 64, &Wire, -1), isOnline(false) {}

bool DisplayManager::init() {
  Wire.beginTransmission(ADDR_OLED);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("[DISPLAY] WARNING: SSD1306 OLED not detected at 0x3C."));
    isOnline = false;
    return false;
  }

  if (display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED)) {
    isOnline = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 15);
    display.println(F("====================="));
    display.printf("  SWARM ROBOT R0%d\n", ROBOT_ID + 1);
    display.println(F("  5-DOF Arm & Nav Ready"));
    display.println(F("====================="));
    display.display();
    Serial.println(F("[DISPLAY] SSD1306 OLED Dashboard Initialized (0x3C)."));
    return true;
  }
  return false;
}

void DisplayManager::render(const char* stateStr, float x, float y, float ang, const char* rfidTag, bool wifiOk, int armBase, int armShoulder) {
  if (!isOnline) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("ROBOT R0%d | %s\n", ROBOT_ID + 1, wifiOk ? "WIFI:OK" : "WIFI:--");
  display.println(F("---------------------"));
  display.printf("STATE : %s\n", stateStr);
  display.printf("POS   : X:%.1f Y:%.1f\n", x, y);
  display.printf("HEAD  : %.1f deg\n", ang);
  display.printf("ARM   : B:%d S:%d\n", armBase, armShoulder);
  display.printf("RFID  : %s\n", (rfidTag && strlen(rfidTag) > 0) ? rfidTag : "NONE");

  display.display();
}
