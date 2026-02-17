#include "BatteryMonitor.hpp"

// Single global instance (definition)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);