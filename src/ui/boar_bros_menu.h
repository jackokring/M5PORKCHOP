// BOAR BROS Menu - Manage excluded networks
#pragma once

#include <Arduino.h>
#include <M5Unified.h>
#include <vector>

struct BroInfo {
    uint64_t bssid;      // BSSID as uint64
    String bssidStr;     // Formatted BSSID (AA:BB:CC:DD:EE:FF)
    String ssid;         // SSID if known (from file comment)
};

class BoarBrosMenu {
public:
    static void init();
    static void show();
    static void hide();
    static void update();
    static void draw(M5Canvas& canvas);
    static bool isActive() { return active; }
    static size_t getCount();
    static String getSelectedInfo();
    
private:
    static std::vector<BroInfo> bros;
    static uint8_t selectedIndex;
    static uint8_t scrollOffset;
    static bool active;
    static bool keyWasPressed;
    static bool deleteConfirmActive;
    
    static const uint8_t VISIBLE_ITEMS = 5;
    
    static void handleInput();
    static void loadBros();
    static void deleteSelected();
    static void drawDeleteConfirm(M5Canvas& canvas);
    static String formatBSSID(uint64_t bssid);
};
