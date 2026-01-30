// SD card formatting implementation

#include "sd_format.h"
#include "config.h"
#include "sd_layout.h"
#include "sdlog.h"
#include "../web/fileserver.h"
#include <SD.h>

// FATFS (ESP-IDF) headers may not exist in all Arduino builds.
// Guard format APIs to avoid compile errors.
#if __has_include(<ff.h>)
#include <ff.h>
#define SD_FORMAT_HAS_FF 1
#else
#define SD_FORMAT_HAS_FF 0
#endif

namespace {
SDFormat::Result makeResult(bool success, bool usedFallback, const char* msg) {
    SDFormat::Result res{};
    res.success = success;
    res.usedFallback = usedFallback;
    if (msg && msg[0]) {
        strncpy(res.message, msg, sizeof(res.message) - 1);
        res.message[sizeof(res.message) - 1] = '\0';
    } else {
        res.message[0] = '\0';
    }
    return res;
}

bool wipePorkchopLayout() {
    const char* root = SDLayout::newRoot();
    if (SD.exists(root)) {
        if (!FileServer::deletePathRecursive(String(root))) {
            return false;
        }
    }
    SDLayout::setUseNewLayout(true);
    SDLayout::ensureDirs();
    return true;
}

#if SD_FORMAT_HAS_FF
bool fatfsFormat() {
    // FATFS format uses logical drive strings like "0:"
    const char* drive = "0:";
#if defined(MKFS_PARM)
    MKFS_PARM opt{};
    opt.fmt = FM_FAT32;
    opt.n_fat = 1;
    opt.align = 0;
    opt.n_root = 0;
    opt.au_size = 0;
#else
    // Older FatFs uses a BYTE for format flags (FDISK not supported here).
    BYTE opt = FM_FAT32;
#endif

    static uint8_t workbuf[4096];
#if defined(MKFS_PARM)
    FRESULT fr = f_mkfs(drive, &opt, 0, workbuf, sizeof(workbuf));
#else
    FRESULT fr = f_mkfs(drive, opt, 0, workbuf, sizeof(workbuf));
#endif
    return fr == FR_OK;
}
#endif
} // namespace

namespace SDFormat {

Result formatCard(bool allowFallback) {
    if (!Config::isSDAvailable()) {
        return makeResult(false, false, "NO SD CARD");
    }

    bool logWasEnabled = SDLog::isEnabled();
    SDLog::close();
    SDLog::setEnabled(false);

#if SD_FORMAT_HAS_FF
    if (fatfsFormat()) {
        SD.end();
        delay(80);
        if (!Config::reinitSD()) {
            SDLog::setEnabled(logWasEnabled);
            return makeResult(false, false, "REMOUNT FAIL");
        }
        SDLayout::setUseNewLayout(true);
        SDLayout::ensureDirs();
        SDLog::setEnabled(logWasEnabled);
        return makeResult(true, false, "FORMAT OK");
    }
#endif

    if (allowFallback && wipePorkchopLayout()) {
        SDLog::setEnabled(logWasEnabled);
        return makeResult(true, true, "WIPE OK");
    }

    SDLog::setEnabled(logWasEnabled);
    return makeResult(false, allowFallback, "FORMAT FAIL");
}

} // namespace SDFormat
