/**
 * SON OF A PIG - BLE Sync Client
 * 
 * Porkchop receives captures from Sirloin devices over BLE.
 * Acts as BLE central/client, connects to Sirloin peripherals.
 * 
 * Protocol: PRKCHAP3LINKSYNK
 * Data flow: Sirloin (child) -> Papa (parent)
 * 
 * READY TO PCAP YOUR PHONE. LOL.
 */
#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <vector>

// Connection state for discovered Sirloin devices
struct SirloinDevice {
    NimBLEAddress address;
    int8_t rssi;
    uint16_t pendingCaptures;  // From advertising data
    uint8_t flags;             // 0x01=hunting, 0x02=buffer full, 0x04=battery low
    uint32_t lastSeen;
    bool syncing;              // Currently syncing with this device
    char name[16];             // Device name (e.g., "SIRLOIN")
};

// Transfer progress
struct SyncProgress {
    uint16_t currentChunk;
    uint16_t totalChunks;
    uint32_t bytesReceived;
    uint32_t startTime;
    uint8_t captureType;       // 0=PMKID, 1=Handshake
    uint8_t captureIndex;
    bool inProgress;
};

class CallPapaMode {
public:
    static void init();
    static void start();
    static void stop();
    static void update();
    static bool isRunning() { return running; }
    
    // Scanning for Sirloin devices
    static void startScan();
    static void stopScan();
    static bool isScanning();
    static void setScanningState(bool state);
    static const std::vector<SirloinDevice>& getDevices() { return devices; }
    static uint8_t getDeviceCount() { return devices.size(); }
    static std::vector<SirloinDevice>& getDevicesMutable() { return devices; }
    
    // Check if SIRLOIN is available (for conditional message display)
    static bool isSirloinAvailable() { return devices.size() > 0; }
    
    // Connection management
    static bool connectTo(uint8_t deviceIndex);
    static void disconnect();
    static bool isConnected();
    static const SirloinDevice* getConnectedDevice();
    
    // Sync operations
    static bool startSync();
    static void abortSync();
    static bool isSyncing();
    static bool isSyncComplete();
    static const SyncProgress& getProgress() { return progress; }
    
    // Remote device info (after HELLO)
    static uint16_t getRemotePMKIDCount() { return remotePMKIDCount; }
    static uint16_t getRemoteHandshakeCount() { return remoteHSCount; }
    static uint16_t getTotalSynced() { return totalSynced; }
    static uint16_t getSyncedCount() { return syncedPMKIDs + syncedHandshakes; }
    static uint16_t getTotalToSync() { return remotePMKIDCount + remoteHSCount; }
    
    // UI selection
    static void selectDevice(uint8_t index);
    static uint8_t getSelectedIndex() { return selectedIndex; }
    
    // Last error
    static const char* getLastError() { return lastError; }
    
    // Callback types
    typedef void (*CaptureCallback)(uint8_t type, const uint8_t* data, uint16_t len);
    typedef void (*SyncCompleteCallback)(uint16_t pmkids, uint16_t handshakes);
    
    // Set callbacks
    static void setOnCapture(CaptureCallback cb) { onCaptureCb = cb; }
    static void setOnSyncComplete(SyncCompleteCallback cb) { onSyncCompleteCb = cb; }
    
    // Friend declarations for BLE callbacks
    friend void ctrlNotifyCallback(NimBLERemoteCharacteristic*, uint8_t*, size_t, bool);
    friend void dataNotifyCallback(NimBLERemoteCharacteristic*, uint8_t*, size_t, bool);
    friend class ScanCallbacks;
    friend class ClientCallbacks;
    
private:
    static bool running;
    static bool bleInitialized;
    static uint8_t selectedIndex;
    
    static std::vector<SirloinDevice> devices;
    static NimBLEClient* pClient;
    static NimBLERemoteCharacteristic* pCtrlChar;
    static NimBLERemoteCharacteristic* pDataChar;
    static NimBLERemoteCharacteristic* pStatusChar;
    static NimBLEAdvertisedDevice* targetDevice;
    
    // Remote device counts
    static uint16_t remotePMKIDCount;
    static uint16_t remoteHSCount;
    static uint16_t totalSynced;
    static uint16_t syncedPMKIDs;
    static uint16_t syncedHandshakes;
    
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
    static State state;
    
    // Sync state
    static uint16_t currentType;        // 0x01=PMKID, 0x02=Handshake
    static uint16_t currentIndex;
    static uint16_t totalChunks;
    static uint16_t receivedChunks;
    
    // Transfer state
    static SyncProgress progress;
    static uint8_t rxBuffer[];
    static uint16_t rxBufferLen;
    static char lastError[];
    
    // Callbacks
    static CaptureCallback onCaptureCb;
    static SyncCompleteCallback onSyncCompleteCb;
    
    // Protocol helpers
    static void sendCommand(uint8_t cmd);
    static void sendCommand(uint8_t cmd, uint8_t param1, uint8_t param2);
    static void sendCommand(uint8_t cmd, uint8_t type, uint16_t index);
    static void requestNextCapture();
    static uint32_t calculateCRC32(const uint8_t* data, uint16_t len);
    
    // Saving
    static bool savePMKID(const uint8_t* data, uint16_t len);
    static bool saveHandshake(const uint8_t* data, uint16_t len);
    
    // Constants
    static const uint16_t RX_BUFFER_SIZE = 2048;
    static const uint16_t SCAN_DURATION = 10;  // seconds
};
