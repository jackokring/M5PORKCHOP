// Warhog Mode implementation

#include "warhog.h"
#include "../core/config.h"
#include "../ui/display.h"
#include "../piglet/mood.h"
#include "../ml/features.h"
#include "../ml/inference.h"
#include <WiFi.h>
#include <SPI.h>
#include <SD.h>

// Maximum entries to prevent memory exhaustion (~120 bytes each, 2000 = ~240KB)
static const size_t MAX_ENTRIES = 2000;

// Static members
bool WarhogMode::running = false;
uint32_t WarhogMode::lastScanTime = 0;
uint32_t WarhogMode::scanInterval = 5000;
std::vector<WardrivingEntry> WarhogMode::entries;
size_t WarhogMode::newCount = 0;
uint32_t WarhogMode::totalNetworks = 0;
uint32_t WarhogMode::openNetworks = 0;
uint32_t WarhogMode::wepNetworks = 0;
uint32_t WarhogMode::wpaNetworks = 0;
uint32_t WarhogMode::savedCount = 0;
String WarhogMode::currentFilename = "";

// Enhanced mode statics
bool WarhogMode::enhancedMode = false;
std::map<uint64_t, WiFiFeatures> WarhogMode::beaconFeatures;
uint32_t WarhogMode::beaconCount = 0;

// Simple flag to avoid concurrent access between promiscuous callback and main thread
static volatile bool beaconMapBusy = false;

void WarhogMode::init() {
    entries.clear();
    newCount = 0;
    totalNetworks = 0;
    openNetworks = 0;
    wepNetworks = 0;
    wpaNetworks = 0;
    savedCount = 0;
    currentFilename = "";
    
    // Check if Enhanced ML mode is enabled
    enhancedMode = (Config::ml().collectionMode == MLCollectionMode::ENHANCED);
    beaconFeatures.clear();
    beaconCount = 0;
    
    scanInterval = Config::gps().updateInterval * 1000;
    
    Serial.printf("[WARHOG] Initialized (ML Mode: %s)\n", 
                  enhancedMode ? "Enhanced" : "Basic");
}

void WarhogMode::start() {
    if (running) return;
    
    Serial.println("[WARHOG] Starting...");
    
    // Clear previous session data
    entries.clear();
    totalNetworks = 0;
    openNetworks = 0;
    wepNetworks = 0;
    wpaNetworks = 0;
    savedCount = 0;
    currentFilename = "";
    beaconFeatures.clear();
    beaconCount = 0;
    
    // Reload scan interval from config
    scanInterval = Config::gps().updateInterval * 1000;
    Serial.printf("[WARHOG] Scan interval: %lu ms\n", scanInterval);
    
    // Check if Enhanced ML mode is enabled (might have changed in settings)
    enhancedMode = (Config::ml().collectionMode == MLCollectionMode::ENHANCED);
    
    // Full WiFi reinitialization for clean slate
    WiFi.disconnect(true);  // Disconnect and clear settings
    WiFi.mode(WIFI_OFF);    // Turn off WiFi completely
    delay(100);             // Let it settle
    WiFi.mode(WIFI_STA);    // Station mode for scanning
    delay(100);             // Let it initialize
    
    Serial.println("[WARHOG] WiFi initialized in STA mode");
    
    // If Enhanced mode, start promiscuous capture for beacons
    if (enhancedMode) {
        startEnhancedCapture();
    }
    
    // Wake up GPS
    GPS::wake();
    
    running = true;
    lastScanTime = 0;  // Trigger immediate scan
    newCount = 0;
    
    Display::setWiFiStatus(true);
    Mood::onWarhogUpdate();  // Show WARHOG phrase on start
    Serial.printf("[WARHOG] Running (ML Mode: %s)\n", 
                  enhancedMode ? "Enhanced" : "Basic");
}

void WarhogMode::stop() {
    if (!running) return;
    
    Serial.println("[WARHOG] Stopping...");
    
    // Stop Enhanced mode capture
    if (enhancedMode) {
        stopEnhancedCapture();
    }
    
    running = false;
    
    // Put GPS to sleep if power management enabled
    if (Config::gps().powerSave) {
        GPS::sleep();
    }
    
    Display::setWiFiStatus(false);
    
    Serial.println("[WARHOG] Stopped");
}

void WarhogMode::update() {
    if (!running) return;
    
    uint32_t now = millis();
    static uint32_t lastPhraseTime = 0;
    
    // Rotate phrases every 5 seconds when idle
    if (now - lastPhraseTime >= 5000) {
        Mood::onWarhogUpdate();
        lastPhraseTime = now;
    }
    
    // Periodic scanning with synchronous scan
    if (now - lastScanTime >= scanInterval) {
        performScan();
        
        // Process results immediately (synchronous scan)
        int n = WiFi.scanComplete();
        if (n >= 0) {
            processScanResults();
        }
        
        // Clean up
        WiFi.scanDelete();
        
        lastScanTime = now;
    }
}

void WarhogMode::triggerScan() {
    performScan();
}

bool WarhogMode::isScanComplete() {
    return WiFi.scanComplete() >= 0;
}

void WarhogMode::performScan() {
    Serial.println("[WARHOG] Starting WiFi scan...");
    
    // Temporarily disable promiscuous mode for scanning (conflicts with scan API)
    bool wasEnhanced = enhancedMode;
    if (wasEnhanced) {
        esp_wifi_set_promiscuous(false);
    }
    
    // Use SYNCHRONOUS scan for reliability - async has issues on ESP32
    // This blocks but is more reliable for wardriving
    int result = WiFi.scanNetworks(false, true);  // async=false (blocking), hidden=true
    
    Serial.printf("[WARHOG] scanNetworks returned: %d\n", result);
    
    if (result >= 0) {
        Serial.printf("[WARHOG] Scan complete, found %d networks\n", result);
    } else if (result == WIFI_SCAN_FAILED) {
        Serial.println("[WARHOG] Scan failed");
    } else {
        Serial.printf("[WARHOG] Unexpected scan result: %d\n", result);
    }
    
    // Re-enable promiscuous if it was on
    if (wasEnhanced) {
        esp_wifi_set_promiscuous(true);
    }
}

void WarhogMode::processScanResults() {
    int n = WiFi.scanComplete();
    
    if (n == WIFI_SCAN_RUNNING) {
        return;  // Still scanning
    }
    
    if (n == WIFI_SCAN_FAILED || n < 0) {
        Serial.println("[WARHOG] Scan failed");
        WiFi.scanDelete();
        return;
    }
    
    // Reset count for this scan cycle
    newCount = 0;
    
    // Get current GPS data
    GPSData gps = GPS::getData();
    bool hasGPS = GPS::hasFix();
    
    Serial.printf("[WARHOG] Found %d networks (GPS: %s)\n", 
                 n, hasGPS ? "yes" : "no");
    
    // Get actual wifi_ap_record_t array from ESP-IDF for accurate features
    uint16_t apCount = n;
    wifi_ap_record_t* apRecords = new wifi_ap_record_t[apCount];
    if (!apRecords) {
        Serial.println("[WARHOG] Failed to allocate AP records");
        WiFi.scanDelete();
        return;
    }
    
    esp_err_t err = esp_wifi_scan_get_ap_records(&apCount, apRecords);
    if (err != ESP_OK) {
        Serial.printf("[WARHOG] Failed to get AP records: %d\n", err);
        delete[] apRecords;
        WiFi.scanDelete();
        return;
    }
    
    // Guard beacon map access from promiscuous callback
    beaconMapBusy = true;
    
    size_t previousCount = entries.size();
    
    for (uint16_t i = 0; i < apCount; i++) {
        wifi_ap_record_t& ap = apRecords[i];
        
        int idx = findEntry(ap.bssid);
        
        if (idx < 0) {
            // Check memory limit before adding
            if (entries.size() >= MAX_ENTRIES) {
                Serial.println("[WARHOG] Max entries reached, saving and clearing");
                saveNewEntries();  // Save current entries
                entries.clear();   // Clear to make room
                previousCount = 0; // Reset so newCount calculation is correct
                // Reset stats to match cleared entries
                totalNetworks = 0;
                openNetworks = 0;
                wepNetworks = 0;
                wpaNetworks = 0;
                savedCount = 0;
            }
            
            // New network - use actual ESP-IDF data
            WardrivingEntry entry = {0};
            memcpy(entry.bssid, ap.bssid, 6);
            strncpy(entry.ssid, (const char*)ap.ssid, 32);
            entry.ssid[32] = '\0';
            entry.rssi = ap.rssi;
            entry.channel = ap.primary;
            entry.authmode = ap.authmode;
            entry.timestamp = millis();
            entry.saved = false;
            entry.label = 0;  // Unknown - user can label later
            
            // Extract ML features - Enhanced mode uses beacon-captured features
            if (enhancedMode) {
                uint64_t key = bssidToKey(ap.bssid);
                auto it = beaconFeatures.find(key);
                if (it != beaconFeatures.end()) {
                    // Use beacon-extracted features (full IE parsing, WPS, vendor IEs, etc.)
                    entry.features = it->second;
                    // Update RSSI from scan (may be fresher)
                    entry.features.rssi = ap.rssi;
                    entry.features.snr = (float)(ap.rssi - entry.features.noise);
                } else {
                    // No beacon captured yet, fall back to scan API
                    entry.features = FeatureExtractor::extractFromScan(&ap);
                }
            } else {
                // Basic mode: use scan API features only
                entry.features = FeatureExtractor::extractFromScan(&ap);
            }
            
            if (hasGPS) {
                entry.latitude = gps.latitude;
                entry.longitude = gps.longitude;
                entry.altitude = gps.altitude;
            }
            
            entries.push_back(entry);
            totalNetworks++;
            
            // Track auth types
            switch (entry.authmode) {
                case WIFI_AUTH_OPEN:
                    openNetworks++;
                    break;
                case WIFI_AUTH_WEP:
                    wepNetworks++;
                    break;
                default:
                    wpaNetworks++;
                    break;
            }
            
            Serial.printf("[WARHOG] New: %s (ch%d, %s)\n",
                         entry.ssid, entry.channel, 
                         authModeToString(entry.authmode).c_str());
        } else {
            // Update existing - maybe update GPS if we have better fix
            if (hasGPS && (entries[idx].latitude == 0 || 
                          ap.rssi > entries[idx].rssi)) {
                // Track if this was missing coords (will need save)
                bool wasMissingCoords = (entries[idx].latitude == 0);
                
                entries[idx].latitude = gps.latitude;
                entries[idx].longitude = gps.longitude;
                entries[idx].altitude = gps.altitude;
                entries[idx].rssi = ap.rssi;
                
                // Update features - Enhanced mode may have better data now
                if (enhancedMode) {
                    uint64_t key = bssidToKey(ap.bssid);
                    auto it = beaconFeatures.find(key);
                    if (it != beaconFeatures.end()) {
                        entries[idx].features = it->second;
                        entries[idx].features.rssi = ap.rssi;
                        entries[idx].features.snr = (float)(ap.rssi - entries[idx].features.noise);
                    }
                } else {
                    entries[idx].features = FeatureExtractor::extractFromScan(&ap);
                }
                
                // If entry now has coords but wasn't saved yet, count as needing save
                if (wasMissingCoords && !entries[idx].saved) {
                    newCount++;
                }
            }
        }
    }
    
    // Release beacon map guard
    beaconMapBusy = false;
    
    delete[] apRecords;
    
    // Add newly discovered networks to count (entries that got coords are already counted above)
    newCount += entries.size() - previousCount;
    
    if (newCount > 0) {
        // Use WARHOG-specific phrases for found networks
        Mood::onWarhogFound(nullptr, 0);
        
        // Auto-save new entries with GPS fix to CSV
        if (hasGPS && Config::isSDAvailable()) {
            saveNewEntries();
        }
    }
    
    WiFi.scanDelete();
    
    // Re-enable promiscuous mode for beacon capture if Enhanced mode
    if (enhancedMode) {
        esp_wifi_set_promiscuous(true);
    }
}

// Helper to escape XML special characters
static String escapeXML(const char* str) {
    String result;
    for (int i = 0; str[i] && i < 64; i++) {
        switch (str[i]) {
            case '&':  result += "&amp;"; break;
            case '<':  result += "&lt;"; break;
            case '>':  result += "&gt;"; break;
            case '"':  result += "&quot;"; break;
            case '\'': result += "&apos;"; break;
            default:   result += str[i]; break;
        }
    }
    return result;
}

// Helper to write CSV-escaped SSID field (quoted, doubles internal quotes, strips control chars)
static void writeCSVField(File& f, const char* ssid) {
    f.print("\"");
    for (int i = 0; i < 32 && ssid[i]; i++) {
        if (ssid[i] == '"') {
            f.print("\"\"");
        } else if (ssid[i] >= 32) {  // Skip control characters (newlines, etc)
            f.print(ssid[i]);
        }
    }
    f.print("\"");
}

void WarhogMode::saveNewEntries() {
    // Create file with unique name on first save
    if (currentFilename.length() == 0) {
        currentFilename = generateFilename("csv");
        
        // Write CSV header
        File f = SD.open(currentFilename.c_str(), FILE_WRITE);
        if (f) {
            f.println("BSSID,SSID,RSSI,Channel,AuthMode,Latitude,Longitude,Altitude,Timestamp");
            f.close();
            Serial.printf("[WARHOG] Created file: %s\n", currentFilename.c_str());
        } else {
            Serial.printf("[WARHOG] Failed to create: %s\n", currentFilename.c_str());
            return;
        }
    }
    
    // Append unsaved entries that have GPS coordinates
    File f = SD.open(currentFilename.c_str(), FILE_APPEND);
    if (!f) {
        Serial.printf("[WARHOG] Failed to append to %s\n", currentFilename.c_str());
        return;
    }
    
    uint32_t newSaved = 0;
    for (auto& e : entries) {
        // Only save if not already saved AND has GPS coordinates
        if (!e.saved && e.latitude != 0 && e.longitude != 0) {
            f.printf("%02X:%02X:%02X:%02X:%02X:%02X,",
                    e.bssid[0], e.bssid[1], e.bssid[2],
                    e.bssid[3], e.bssid[4], e.bssid[5]);
            
            // Escape SSID for CSV
            writeCSVField(f, e.ssid);
            f.print(",");
            
            f.printf("%d,%d,%s,%.6f,%.6f,%.1f,%lu\n",
                    e.rssi, e.channel, authModeToString(e.authmode).c_str(),
                    e.latitude, e.longitude, e.altitude, e.timestamp);
            
            e.saved = true;
            newSaved++;
            savedCount++;
        }
    }
    
    f.flush();  // Ensure data hits SD immediately (crash protection)
    f.close();
    
    if (newSaved > 0) {
        Serial.printf("[WARHOG] Saved %lu entries (total: %lu)\n", newSaved, savedCount);
    }
}

bool WarhogMode::hasGPSFix() {
    return GPS::hasFix();
}

GPSData WarhogMode::getGPSData() {
    return GPS::getData();
}

bool WarhogMode::exportCSV(const char* path) {
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        Serial.printf("[WARHOG] Failed to open %s\n", path);
        return false;
    }
    
    // CSV header
    f.println("BSSID,SSID,RSSI,Channel,AuthMode,Latitude,Longitude,Altitude,Timestamp");
    
    for (const auto& e : entries) {
        f.printf("%02X:%02X:%02X:%02X:%02X:%02X,",
                e.bssid[0], e.bssid[1], e.bssid[2],
                e.bssid[3], e.bssid[4], e.bssid[5]);
        
        // Escape SSID for CSV
        writeCSVField(f, e.ssid);
        f.print(",");
        
        f.printf("%d,%d,%s,%.6f,%.6f,%.1f,%lu\n",
                e.rssi, e.channel, authModeToString(e.authmode).c_str(),
                e.latitude, e.longitude, e.altitude, e.timestamp);
    }
    
    f.close();
    Serial.printf("[WARHOG] Exported %d entries to %s\n", entries.size(), path);
    return true;
}

bool WarhogMode::exportWigle(const char* path) {
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        return false;
    }
    
    // Wigle CSV format
    f.println("WigleWifi-1.4,appRelease=porkchop,model=M5Cardputer,release=1.0.0,device=ESP32-S3,display=,board=,brand=M5Stack");
    f.println("MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type");
    
    for (const auto& e : entries) {
        f.printf("%02X:%02X:%02X:%02X:%02X:%02X,",
                e.bssid[0], e.bssid[1], e.bssid[2],
                e.bssid[3], e.bssid[4], e.bssid[5]);
        
        writeCSVField(f, e.ssid);
        f.print(",");
        
        f.printf("%s,", authModeToString(e.authmode).c_str());
        
        // Timestamp from GPS or use current date
        GPSData gps = GPS::getData();
        if (gps.date > 0 && gps.time > 0) {
            // Parse DDMMYY and HHMMSSCC format
            uint8_t day = gps.date / 10000;
            uint8_t month = (gps.date / 100) % 100;
            uint8_t year = gps.date % 100;
            uint8_t hour = gps.time / 1000000;
            uint8_t minute = (gps.time / 10000) % 100;
            uint8_t second = (gps.time / 100) % 100;
            
            // Validate date/time ranges
            if (day > 0 && day <= 31 && month > 0 && month <= 12 && year <= 99 &&
                hour < 24 && minute < 60 && second < 60) {
                f.printf("20%02d-%02d-%02d %02d:%02d:%02d,", year, month, day, hour, minute, second);
            } else {
                f.printf("2025-01-01 00:00:00,");
            }
        } else {
            f.printf("2025-01-01 00:00:00,");
        }
        
        f.printf("%d,%d,%.6f,%.6f,%.1f,10.0,WIFI\n",
                e.channel, e.rssi,
                e.latitude, e.longitude, e.altitude);
    }
    
    f.close();
    Serial.printf("[WARHOG] Wigle export: %d entries to %s\n", entries.size(), path);
    return true;
}

bool WarhogMode::exportKismet(const char* path) {
    // Kismet NetXML format - simplified
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        return false;
    }
    
    f.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
    f.println("<detection-run kismet-version=\"porkchop\">");
    
    for (const auto& e : entries) {
        f.println("<wireless-network>");
        f.printf("<BSSID>%02X:%02X:%02X:%02X:%02X:%02X</BSSID>\n",
                e.bssid[0], e.bssid[1], e.bssid[2],
                e.bssid[3], e.bssid[4], e.bssid[5]);
        f.printf("<SSID>%s</SSID>\n", escapeXML(e.ssid).c_str());
        f.printf("<channel>%d</channel>\n", e.channel);
        f.printf("<encryption>%s</encryption>\n", authModeToString(e.authmode).c_str());
        f.println("<gps-info>");
        f.printf("<lat>%.6f</lat>\n", e.latitude);
        f.printf("<lon>%.6f</lon>\n", e.longitude);
        f.printf("<alt>%.1f</alt>\n", e.altitude);
        f.println("</gps-info>");
        f.println("</wireless-network>");
    }
    
    f.println("</detection-run>");
    f.close();
    Serial.printf("[WARHOG] Kismet export: %d entries to %s\n", entries.size(), path);
    return true;
}

int WarhogMode::findEntry(const uint8_t* bssid) {
    for (size_t i = 0; i < entries.size(); i++) {
        if (memcmp(entries[i].bssid, bssid, 6) == 0) {
            return i;
        }
    }
    return -1;
}

String WarhogMode::authModeToString(wifi_auth_mode_t mode) {
    switch (mode) {
        case WIFI_AUTH_OPEN: return "OPEN";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA";
        case WIFI_AUTH_WPA2_PSK: return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
        case WIFI_AUTH_WPA3_PSK: return "WPA3";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
        case WIFI_AUTH_WAPI_PSK: return "WAPI";
        default: return "UNKNOWN";
    }
}

String WarhogMode::generateFilename(const char* ext) {
    // Generate filename with date/time from GPS: YYYYMMDD_HHMMSS
    char buf[64];
    GPSData gps = GPS::getData();
    
    if (gps.date > 0 && gps.time > 0) {
        // date format: DDMMYY, time format: HHMMSSCC (centiseconds)
        uint8_t day = gps.date / 10000;
        uint8_t month = (gps.date / 100) % 100;
        uint8_t year = gps.date % 100;
        uint8_t hour = gps.time / 1000000;
        uint8_t minute = (gps.time / 10000) % 100;
        uint8_t second = (gps.time / 100) % 100;
        
        snprintf(buf, sizeof(buf), "/wardriving/warhog_20%02d%02d%02d_%02d%02d%02d.%s",
                year, month, day, hour, minute, second, ext);
    } else {
        snprintf(buf, sizeof(buf), "/wardriving/warhog_%lu.%s", millis(), ext);
    }
    
    return String(buf);
}

bool WarhogMode::exportMLTraining(const char* path) {
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        Serial.printf("[WARHOG] Failed to open ML export: %s\n", path);
        return false;
    }
    
    // CSV header - all 32 feature vector values + label + metadata
    f.print("bssid,ssid,");
    f.print("rssi,noise,snr,channel,secondary_ch,beacon_interval,");
    f.print("capability_lo,capability_hi,has_wps,has_wpa,has_wpa2,has_wpa3,");
    f.print("is_hidden,response_time,beacon_count,beacon_jitter,");
    f.print("responds_probe,probe_response_time,vendor_ie_count,");
    f.print("supported_rates,ht_cap,vht_cap,anomaly_score,");
    f.print("f23,f24,f25,f26,f27,f28,f29,f30,f31,");  // Reserved features
    f.println("label,latitude,longitude");
    
    float featureVec[FEATURE_VECTOR_SIZE];
    
    for (const auto& e : entries) {
        // BSSID
        f.printf("%02X:%02X:%02X:%02X:%02X:%02X,",
                e.bssid[0], e.bssid[1], e.bssid[2],
                e.bssid[3], e.bssid[4], e.bssid[5]);
        
        // SSID (escaped)
        writeCSVField(f, e.ssid);
        f.print(",");
        
        // Convert features to vector
        FeatureExtractor::toFeatureVector(e.features, featureVec);
        
        // Write all 32 feature values
        for (int i = 0; i < FEATURE_VECTOR_SIZE; i++) {
            f.printf("%.4f,", featureVec[i]);
        }
        
        // Label and GPS
        f.printf("%d,%.6f,%.6f\n", e.label, e.latitude, e.longitude);
    }
    
    f.close();
    Serial.printf("[WARHOG] ML training export: %d entries to %s\n", entries.size(), path);
    return true;
}

// Enhanced ML Mode - Promiscuous beacon capture

void WarhogMode::promiscuousCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
    // Only process management frames (beacons, probe responses)
    if (type != WIFI_PKT_MGMT) return;
    
    // Skip if main thread is accessing the map
    if (beaconMapBusy) return;
    
    wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
    const uint8_t* frame = pkt->payload;
    uint16_t len = pkt->rx_ctrl.sig_len;
    int8_t rssi = pkt->rx_ctrl.rssi;
    
    if (len < 24) return;
    
    // Frame control field - check if it's a beacon (type=0, subtype=8)
    uint16_t frameControl = frame[0] | (frame[1] << 8);
    uint8_t frameType = (frameControl >> 2) & 0x03;      // Type (bits 2-3)
    uint8_t frameSubtype = (frameControl >> 4) & 0x0F;   // Subtype (bits 4-7)
    
    // Only process beacons (subtype 8) and probe responses (subtype 5)
    if (frameType != 0) return;  // Not management
    if (frameSubtype != 8 && frameSubtype != 5) return;
    
    // BSSID is at offset 16 for beacons/probe responses
    const uint8_t* bssid = frame + 16;
    uint64_t key = bssidToKey(bssid);
    
    // Extract full features from beacon frame
    WiFiFeatures features = FeatureExtractor::extractFromBeacon(frame, len, rssi);
    
    // Store or update features for this BSSID
    // Note: map operations are quick, but we limit to prevent memory issues
    if (beaconFeatures.size() < 500) {  // Max 500 BSSIDs in beacon cache
        auto it = beaconFeatures.find(key);
        if (it != beaconFeatures.end()) {
            // Update beacon count and jitter calculation
            it->second.beaconCount++;
            // Simple jitter: difference from expected interval
            // Real interval would need timestamps, simplified here
        } else {
            features.beaconCount = 1;
            beaconFeatures[key] = features;
        }
        beaconCount++;
    }
}

void WarhogMode::startEnhancedCapture() {
    Serial.println("[WARHOG] Starting Enhanced ML capture (promiscuous mode)");
    
    beaconFeatures.clear();
    beaconCount = 0;
    
    // Configure promiscuous mode filter for management frames only
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
    };
    
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(promiscuousCallback);
    esp_wifi_set_promiscuous(true);
    
    Serial.println("[WARHOG] Promiscuous mode enabled for beacon capture");
}

void WarhogMode::stopEnhancedCapture() {
    Serial.println("[WARHOG] Stopping Enhanced ML capture");
    
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(nullptr);
    
    Serial.printf("[WARHOG] Captured %d beacons from %d BSSIDs\n", 
                  beaconCount, beaconFeatures.size());
}
