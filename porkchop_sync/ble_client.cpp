/**
 * BLE Client for Porkchop - Pulls captures from Sirloin
 * 
 * Sirloin = SERVER (advertises, serves data)
 * Porkchop = CLIENT (scans, connects, pulls)
 */

#include "ble_client.h"
#include <NimBLEDevice.h>

// Must match Sirloin's UUIDs exactly
#define SERVICE_UUID        "50524b43-4841-5033-4c49-4e4b53594e4b"  // "PRKCHAP3LINKSYNK"
#define CTRL_CHAR_UUID      "50524b43-0001-4841-5033-4c494e4b5359"
#define DATA_CHAR_UUID      "50524b43-0002-4841-5033-4c494e4b5359"
#define STATUS_CHAR_UUID    "50524b43-0003-4841-5033-4c494e4b5359"

// Commands (Porkchop -> Sirloin)
#define CMD_HELLO           0x01
#define CMD_GET_COUNT       0x02
#define CMD_START_SYNC      0x03
#define CMD_ACK_CHUNK       0x04
#define CMD_ABORT           0x05
#define CMD_MARK_SYNCED     0x06
#define CMD_PURGE_SYNCED    0x07

// Responses (Sirloin -> Porkchop)
#define RSP_HELLO           0x81
#define RSP_COUNT           0x82
#define RSP_SYNC_START      0x83
#define RSP_OK              0x84
#define RSP_ERROR           0x85
#define RSP_ABORTED         0x86
#define RSP_PURGED          0x87

// Chunk size (must match Sirloin)
#define CHUNK_SIZE          17

namespace BLEClient {

// State
enum class State {
    IDLE,
    SCANNING,
    CONNECTING,
    CONNECTED,
    SYNCING,
    WAITING_CHUNKS,
    SYNC_COMPLETE,
    ERROR
};

static State state = State::IDLE;
static bool bleInitialized = false;

// NimBLE objects
static NimBLEClient* pClient = nullptr;
static NimBLERemoteCharacteristic* pCtrlChar = nullptr;
static NimBLERemoteCharacteristic* pDataChar = nullptr;
static NimBLERemoteCharacteristic* pStatusChar = nullptr;
static NimBLEAdvertisedDevice* targetDevice = nullptr;

// Sync state
static uint16_t remotePMKIDs = 0;
static uint16_t remoteHandshakes = 0;
static uint16_t currentType = 0;        // 0x01=PMKID, 0x02=Handshake
static uint16_t currentIndex = 0;
static uint16_t totalChunks = 0;
static uint16_t receivedChunks = 0;
static uint16_t syncedPMKIDs = 0;
static uint16_t syncedHandshakes = 0;

// Receive buffer
static uint8_t rxBuffer[2048];
static uint16_t rxBufferLen = 0;

// Error
static char lastError[64] = "";
static char foundDeviceName[32] = "";

// Callbacks
static CaptureCallback onCaptureCb = nullptr;
static SyncCompleteCallback onSyncCompleteCb = nullptr;

// CRC32 verification
static uint32_t calculateCRC32(const uint8_t* data, uint16_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

// Forward declarations
static void sendCommand(uint8_t cmd);
static void sendCommand(uint8_t cmd, uint8_t arg1, uint8_t arg2);
static void requestNextCapture();

// ============================================================================
// NOTIFICATION CALLBACKS
// ============================================================================

// Called when Control characteristic notifies (responses from Sirloin)
static void ctrlNotifyCallback(NimBLERemoteCharacteristic* pChar,
                                uint8_t* pData, size_t length, bool isNotify) {
    if (length < 1) return;
    
    uint8_t rsp = pData[0];
    
    switch (rsp) {
        case RSP_HELLO:
            if (length >= 6) {
                remotePMKIDs = pData[2] | (pData[3] << 8);
                remoteHandshakes = pData[4] | (pData[5] << 8);
                Serial.printf("[BLE-CLIENT] HELLO: %d PMKIDs, %d Handshakes\n", 
                              remotePMKIDs, remoteHandshakes);
            }
            break;
            
        case RSP_COUNT:
            if (length >= 5) {
                remotePMKIDs = pData[1] | (pData[2] << 8);
                remoteHandshakes = pData[3] | (pData[4] << 8);
                Serial.printf("[BLE-CLIENT] COUNT: %d PMKIDs, %d Handshakes\n",
                              remotePMKIDs, remoteHandshakes);
            }
            break;
            
        case RSP_SYNC_START:
            if (length >= 5) {
                totalChunks = pData[1] | (pData[2] << 8) | (pData[3] << 16) | (pData[4] << 24);
                receivedChunks = 0;
                rxBufferLen = 0;
                state = State::WAITING_CHUNKS;
                Serial.printf("[BLE-CLIENT] SYNC_START: %d chunks expected\n", totalChunks);
            }
            break;
            
        case RSP_OK:
            Serial.println("[BLE-CLIENT] OK");
            break;
            
        case RSP_ERROR:
            if (length >= 2) {
                snprintf(lastError, sizeof(lastError), "Error code: 0x%02X", pData[1]);
                Serial.printf("[BLE-CLIENT] ERROR: %s\n", lastError);
            }
            break;
            
        case RSP_ABORTED:
            Serial.println("[BLE-CLIENT] Transfer aborted");
            state = State::CONNECTED;
            break;
            
        case RSP_PURGED:
            if (length >= 2) {
                Serial.printf("[BLE-CLIENT] Purged %d captures\n", pData[1]);
            }
            break;
    }
}

// Called when Data characteristic notifies (capture chunks from Sirloin)
static void dataNotifyCallback(NimBLERemoteCharacteristic* pChar,
                                uint8_t* pData, size_t length, bool isNotify) {
    if (length < 2) return;
    
    uint16_t seq = pData[0] | (pData[1] << 8);
    
    // Check for end marker (0xFFFF)
    if (seq == 0xFFFF && length >= 6) {
        // End of transfer - verify CRC
        uint32_t receivedCRC = pData[2] | (pData[3] << 8) | (pData[4] << 16) | (pData[5] << 24);
        uint32_t calculatedCRC = calculateCRC32(rxBuffer, rxBufferLen);
        
        if (receivedCRC == calculatedCRC) {
            Serial.printf("[BLE-CLIENT] Transfer complete! CRC OK, %d bytes\n", rxBufferLen);
            
            // Call capture callback
            if (onCaptureCb) {
                onCaptureCb(currentType, rxBuffer, rxBufferLen);
            }
            
            // Mark synced on Sirloin
            sendCommand(CMD_MARK_SYNCED, currentType, currentIndex);
            
            // Update counts
            if (currentType == 0x01) {
                syncedPMKIDs++;
            } else {
                syncedHandshakes++;
            }
            
            // Request next capture
            currentIndex++;
            requestNextCapture();
        } else {
            Serial.printf("[BLE-CLIENT] CRC MISMATCH! Got 0x%08X, expected 0x%08X\n",
                          receivedCRC, calculatedCRC);
            snprintf(lastError, sizeof(lastError), "CRC mismatch");
            // Retry same capture
            rxBufferLen = 0;
            receivedChunks = 0;
            sendCommand(CMD_START_SYNC, currentType, currentIndex);
        }
        return;
    }
    
    // Regular data chunk
    uint8_t dataLen = length - 2;
    uint16_t offset = seq * CHUNK_SIZE;
    
    if (offset + dataLen <= sizeof(rxBuffer)) {
        memcpy(rxBuffer + offset, pData + 2, dataLen);
        if (offset + dataLen > rxBufferLen) {
            rxBufferLen = offset + dataLen;
        }
        receivedChunks++;
        
        // Send ACK (3 bytes: cmd + 16-bit chunk number)
        uint8_t ack[3] = {CMD_ACK_CHUNK, (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8)};
        pCtrlChar->writeValue(ack, 3, false);
        
        Serial.printf("[BLE-CLIENT] Chunk %d/%d received\n", seq + 1, totalChunks);
    }
}

// ============================================================================
// SCAN CALLBACK
// ============================================================================

class ScanCallbacks : public NimBLEScanCallbacks {
    void onResult(NimBLEAdvertisedDevice* device) override {
        // Look for SIRLOIN by name or service UUID
        if (device->haveName() && device->getName() == "SIRLOIN") {
            Serial.printf("[BLE-CLIENT] Found SIRLOIN: %s\n", 
                          device->getAddress().toString().c_str());
            
            strncpy(foundDeviceName, device->getName().c_str(), sizeof(foundDeviceName) - 1);
            targetDevice = device;
            
            // Stop scanning
            NimBLEDevice::getScan()->stop();
            state = State::IDLE;
        }
    }
    
    void onScanEnd(NimBLEScanResults results) override {
        Serial.println("[BLE-CLIENT] Scan complete");
        if (state == State::SCANNING) {
            state = State::IDLE;
        }
    }
};

static ScanCallbacks scanCallbacks;

// ============================================================================
// CLIENT CALLBACKS
// ============================================================================

class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) override {
        Serial.println("[BLE-CLIENT] Connected to Sirloin!");
        state = State::CONNECTED;
    }
    
    void onDisconnect(NimBLEClient* pClient) override {
        Serial.println("[BLE-CLIENT] Disconnected from Sirloin");
        state = State::IDLE;
        pCtrlChar = nullptr;
        pDataChar = nullptr;
        pStatusChar = nullptr;
    }
};

static ClientCallbacks clientCallbacks;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static void sendCommand(uint8_t cmd) {
    if (pCtrlChar) {
        pCtrlChar->writeValue(&cmd, 1, false);
    }
}

static void sendCommand(uint8_t cmd, uint8_t type, uint16_t index) {
    if (pCtrlChar) {
        uint8_t data[4] = {cmd, type, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8)};
        pCtrlChar->writeValue(data, 4, false);
    }
}

static void requestNextCapture() {
    // First sync all PMKIDs, then all Handshakes
    if (currentType == 0x01) {
        // Still doing PMKIDs
        if (currentIndex < remotePMKIDs) {
            rxBufferLen = 0;
            receivedChunks = 0;
            sendCommand(CMD_START_SYNC, 0x01, currentIndex);
            state = State::SYNCING;
            Serial.printf("[BLE-CLIENT] Requesting PMKID %d/%d\n", currentIndex + 1, remotePMKIDs);
        } else {
            // Done with PMKIDs, start Handshakes
            currentType = 0x02;
            currentIndex = 0;
            requestNextCapture();
        }
    } else {
        // Doing Handshakes
        if (currentIndex < remoteHandshakes) {
            rxBufferLen = 0;
            receivedChunks = 0;
            sendCommand(CMD_START_SYNC, 0x02, currentIndex);
            state = State::SYNCING;
            Serial.printf("[BLE-CLIENT] Requesting Handshake %d/%d\n", currentIndex + 1, remoteHandshakes);
        } else {
            // All done!
            Serial.printf("[BLE-CLIENT] SYNC COMPLETE! %d PMKIDs, %d Handshakes\n",
                          syncedPMKIDs, syncedHandshakes);
            state = State::SYNC_COMPLETE;
            
            if (onSyncCompleteCb) {
                onSyncCompleteCb(syncedPMKIDs, syncedHandshakes);
            }
            
            // Purge synced captures from Sirloin to free memory
            sendCommand(CMD_PURGE_SYNCED);
        }
    }
}

// ============================================================================
// PUBLIC API
// ============================================================================

void init() {
    if (bleInitialized) return;
    
    Serial.println("[BLE-CLIENT] Initializing...");
    
    NimBLEDevice::init("PORKCHOP");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(&clientCallbacks);
    
    // Configure connection parameters
    pClient->setConnectionParams(12, 12, 0, 51);  // min, max, latency, timeout
    pClient->setConnectTimeout(10);  // 10 seconds
    
    bleInitialized = true;
    state = State::IDLE;
    
    Serial.println("[BLE-CLIENT] Ready");
}

void deinit() {
    if (!bleInitialized) return;
    
    disconnect();
    NimBLEDevice::deinit(true);
    
    bleInitialized = false;
    pClient = nullptr;
    pCtrlChar = nullptr;
    pDataChar = nullptr;
    pStatusChar = nullptr;
    targetDevice = nullptr;
    
    Serial.println("[BLE-CLIENT] Deinitialized");
}

void startScan() {
    if (!bleInitialized) return;
    if (state == State::SCANNING) return;
    
    Serial.println("[BLE-CLIENT] Scanning for Sirloin...");
    
    targetDevice = nullptr;
    foundDeviceName[0] = 0;
    
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(&scanCallbacks);
    pScan->setActiveScan(true);
    pScan->setInterval(100);
    pScan->setWindow(99);
    pScan->start(10, false);  // 10 second scan
    
    state = State::SCANNING;
}

void stopScan() {
    NimBLEDevice::getScan()->stop();
    if (state == State::SCANNING) {
        state = State::IDLE;
    }
}

bool connect() {
    if (!bleInitialized || !pClient) return false;
    if (!targetDevice) {
        snprintf(lastError, sizeof(lastError), "No device found - scan first");
        return false;
    }
    
    Serial.printf("[BLE-CLIENT] Connecting to %s...\n", 
                  targetDevice->getAddress().toString().c_str());
    
    state = State::CONNECTING;
    
    if (!pClient->connect(targetDevice)) {
        snprintf(lastError, sizeof(lastError), "Connection failed");
        state = State::ERROR;
        return false;
    }
    
    // Get service
    NimBLERemoteService* pService = pClient->getService(SERVICE_UUID);
    if (!pService) {
        snprintf(lastError, sizeof(lastError), "Service not found");
        pClient->disconnect();
        state = State::ERROR;
        return false;
    }
    
    // Get characteristics
    pCtrlChar = pService->getCharacteristic(CTRL_CHAR_UUID);
    pDataChar = pService->getCharacteristic(DATA_CHAR_UUID);
    pStatusChar = pService->getCharacteristic(STATUS_CHAR_UUID);
    
    if (!pCtrlChar || !pDataChar) {
        snprintf(lastError, sizeof(lastError), "Characteristics not found");
        pClient->disconnect();
        state = State::ERROR;
        return false;
    }
    
    // Subscribe to notifications (MUST subscribe to data/status before starting sync)
    if (pCtrlChar->canNotify()) {
        if (!pCtrlChar->subscribe(true, ctrlNotifyCallback)) {
            snprintf(lastError, sizeof(lastError), "Failed to subscribe to control");
            pClient->disconnect();
            state = State::ERROR;
            return false;
        }
    }
    if (pDataChar->canNotify()) {
        if (!pDataChar->subscribe(true, dataNotifyCallback)) {
            snprintf(lastError, sizeof(lastError), "Failed to subscribe to data");
            pClient->disconnect();
            state = State::ERROR;
            return false;
        }
    }
    if (pStatusChar && pStatusChar->canNotify()) {
        pStatusChar->subscribe(true, nullptr);  // Subscribe but no callback needed
    }
    
    state = State::CONNECTED;
    
    // Send HELLO to get counts
    delay(100);
    sendCommand(CMD_HELLO);
    
    return true;
}

void disconnect() {
    if (pClient && pClient->isConnected()) {
        pClient->disconnect();
    }
    state = State::IDLE;
}

void update() {
    // State machine runs via callbacks
    // Nothing to do here for now
}

void startSync() {
    if (state != State::CONNECTED) {
        snprintf(lastError, sizeof(lastError), "Not connected");
        return;
    }
    
    if (remotePMKIDs == 0 && remoteHandshakes == 0) {
        Serial.println("[BLE-CLIENT] Nothing to sync");
        state = State::SYNC_COMPLETE;
        return;
    }
    
    // Reset counters
    syncedPMKIDs = 0;
    syncedHandshakes = 0;
    currentType = 0x01;  // Start with PMKIDs
    currentIndex = 0;
    
    Serial.printf("[BLE-CLIENT] Starting sync: %d PMKIDs, %d Handshakes\n",
                  remotePMKIDs, remoteHandshakes);
    
    requestNextCapture();
}

void abortSync() {
    if (state == State::SYNCING || state == State::WAITING_CHUNKS) {
        sendCommand(CMD_ABORT);
        state = State::CONNECTED;
    }
}

bool isScanning() { return state == State::SCANNING; }
bool isConnected() { return state == State::CONNECTED || state == State::SYNCING || 
                           state == State::WAITING_CHUNKS || state == State::SYNC_COMPLETE; }
bool isSyncing() { return state == State::SYNCING || state == State::WAITING_CHUNKS; }
bool isSyncComplete() { return state == State::SYNC_COMPLETE; }

uint16_t getRemotePMKIDCount() { return remotePMKIDs; }
uint16_t getRemoteHandshakeCount() { return remoteHandshakes; }
uint16_t getSyncedCount() { return syncedPMKIDs + syncedHandshakes; }
uint16_t getTotalToSync() { return remotePMKIDs + remoteHandshakes; }

const char* getLastError() { return lastError; }
const char* getFoundDeviceName() { return foundDeviceName; }

void setOnCapture(CaptureCallback cb) { onCaptureCb = cb; }
void setOnSyncComplete(SyncCompleteCallback cb) { onSyncCompleteCb = cb; }

} // namespace BLEClient
