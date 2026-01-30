// SD card formatting helpers
#pragma once

#include <Arduino.h>

namespace SDFormat {
    struct Result {
        bool success;
        bool usedFallback;
        char message[64];
    };

    // Attempts FAT32 format if possible; may fall back to wipe if allowFallback is true.
    Result formatCard(bool allowFallback);
}
