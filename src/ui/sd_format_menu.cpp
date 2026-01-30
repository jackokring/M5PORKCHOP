// SD Format Menu - destructive SD card formatting UI

#include "sd_format_menu.h"
#include "display.h"
#include "../core/config.h"
#include <M5Cardputer.h>

bool SdFormatMenu::active = false;
bool SdFormatMenu::keyWasPressed = false;
SdFormatMenu::State SdFormatMenu::state = SdFormatMenu::State::IDLE;
SDFormat::Result SdFormatMenu::lastResult = {};
bool SdFormatMenu::hasResult = false;

void SdFormatMenu::show() {
    active = true;
    keyWasPressed = true;  // Ignore the Enter that brought us here
    state = State::IDLE;
    hasResult = false;
    Display::clearBottomOverlay();
}

void SdFormatMenu::hide() {
    active = false;
    Display::clearBottomOverlay();
}

void SdFormatMenu::update() {
    if (!active) return;
    if (state == State::WORKING) {
        startFormat();
        return;
    }
    handleInput();
}

void SdFormatMenu::handleInput() {
    bool anyPressed = M5Cardputer.Keyboard.isPressed();
    if (!anyPressed) {
        keyWasPressed = false;
        return;
    }
    if (keyWasPressed) return;
    keyWasPressed = true;

    auto keys = M5Cardputer.Keyboard.keysState();

    if (state == State::CONFIRM) {
        if (M5Cardputer.Keyboard.isKeyPressed('y') || M5Cardputer.Keyboard.isKeyPressed('Y')) {
            Display::clearBottomOverlay();
            state = State::WORKING;
        } else if (M5Cardputer.Keyboard.isKeyPressed('n') || M5Cardputer.Keyboard.isKeyPressed('N') ||
                   M5Cardputer.Keyboard.isKeyPressed(KEY_BACKSPACE) || keys.enter) {
            Display::clearBottomOverlay();
            state = State::IDLE;
        }
        return;
    }

    if (state == State::RESULT) {
        if (keys.enter || M5Cardputer.Keyboard.isKeyPressed(KEY_BACKSPACE)) {
            hide();
        }
        return;
    }

    if (keys.enter) {
        if (!Config::isSDAvailable()) {
            Display::notify(NoticeKind::WARNING, "NO SD CARD");
            return;
        }
        state = State::CONFIRM;
        Display::setBottomOverlay("PERMANENT | NO UNDO");
        return;
    }

    if (M5Cardputer.Keyboard.isKeyPressed(KEY_BACKSPACE)) {
        hide();
    }
}

void SdFormatMenu::startFormat() {
    Display::showProgress("FORMATTING", 0);
    lastResult = SDFormat::formatCard(true);
    hasResult = true;
    Display::showProgress("FORMATTING", 100);
    state = State::RESULT;
}

void SdFormatMenu::draw(M5Canvas& canvas) {
    if (!active) return;

    canvas.fillSprite(COLOR_BG);
    canvas.setTextColor(COLOR_FG);

    canvas.setTextDatum(top_center);
    canvas.setTextSize(2);
    canvas.drawString("SD FORMAT", DISPLAY_W / 2, 2);
    canvas.drawLine(10, 20, DISPLAY_W - 10, 20, COLOR_FG);

    if (state == State::WORKING) {
        drawWorking(canvas);
        return;
    }

    if (state == State::RESULT && hasResult) {
        drawResult(canvas);
    } else {
        drawIdle(canvas);
    }

    if (state == State::CONFIRM) {
        drawConfirm(canvas);
    }
}

void SdFormatMenu::drawIdle(M5Canvas& canvas) {
    canvas.setTextDatum(top_left);
    canvas.setTextSize(1);

    int y = 26;
    const int lineH = 14;

    canvas.drawString("ERASES ALL DATA", 4, y);
    y += lineH;
    canvas.drawString("FAT32 OR WIPE", 4, y);
    y += lineH;

    canvas.drawString("SD:", 4, y);
    canvas.drawString(Config::isSDAvailable() ? "OK" : "MISSING", 40, y);
    y += lineH;

    canvas.drawString("ENTER=FORMAT", 4, y);
    y += lineH;
    canvas.drawString("BKSPC=BACK", 4, y);
}

void SdFormatMenu::drawWorking(M5Canvas& canvas) {
    canvas.setTextDatum(top_center);
    canvas.setTextSize(2);
    canvas.drawString("FORMATTING", DISPLAY_W / 2, 44);
    canvas.setTextSize(1);
    canvas.drawString("DO NOT POWER OFF", DISPLAY_W / 2, 66);
}

void SdFormatMenu::drawResult(M5Canvas& canvas) {
    canvas.setTextDatum(top_center);
    canvas.setTextSize(2);
    canvas.drawString(lastResult.success ? "SUCCESS" : "FAILED", DISPLAY_W / 2, 28);

    canvas.setTextSize(1);
    if (lastResult.message[0] != '\0') {
        canvas.drawString(lastResult.message, DISPLAY_W / 2, 50);
    }
    if (lastResult.usedFallback) {
        canvas.drawString("FALLBACK USED", DISPLAY_W / 2, 64);
    }
    canvas.drawString("ENTER/BKSPC TO EXIT", DISPLAY_W / 2, 86);
}

void SdFormatMenu::drawConfirm(M5Canvas& canvas) {
    const int boxW = 200;
    const int boxH = 70;
    const int boxX = (DISPLAY_W - boxW) / 2;
    const int boxY = (MAIN_H - boxH) / 2 - 5;

    canvas.fillRoundRect(boxX - 2, boxY - 2, boxW + 4, boxH + 4, 8, COLOR_BG);
    canvas.fillRoundRect(boxX, boxY, boxW, boxH, 8, COLOR_FG);

    canvas.setTextColor(COLOR_BG, COLOR_FG);
    canvas.setTextDatum(top_center);
    canvas.setTextSize(1);

    int centerX = DISPLAY_W / 2;
    canvas.drawString("!! FORMAT SD !!", centerX, boxY + 8);
    canvas.drawString("ERASES ALL DATA", centerX, boxY + 22);
    canvas.drawString("NO UNDO.", centerX, boxY + 36);
    canvas.drawString("[Y] DO IT  [N] ABORT", centerX, boxY + 54);
}
