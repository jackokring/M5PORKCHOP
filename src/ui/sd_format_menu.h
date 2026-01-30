#pragma once

#include <Arduino.h>
#include <M5Unified.h>
#include "../core/sd_format.h"

class SdFormatMenu {
public:
    static void show();
    static void hide();
    static void update();
    static bool isActive() { return active; }
    static void draw(M5Canvas& canvas);

private:
    enum class State : uint8_t {
        IDLE,
        CONFIRM,
        WORKING,
        RESULT
    };

    static bool active;
    static bool keyWasPressed;
    static State state;
    static SDFormat::Result lastResult;
    static bool hasResult;

    static void handleInput();
    static void startFormat();
    static void drawIdle(M5Canvas& canvas);
    static void drawConfirm(M5Canvas& canvas);
    static void drawWorking(M5Canvas& canvas);
    static void drawResult(M5Canvas& canvas);
};
