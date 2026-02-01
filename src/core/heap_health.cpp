#include "heap_health.h"
#include "heap_policy.h"
#include <Arduino.h>
#include <esp_heap_caps.h>

namespace HeapHealth {

static uint8_t heapHealthPct = 100;
static uint32_t lastSampleMs = 0;
static uint32_t toastStartMs = 0;
static uint32_t lastToastMs = 0;
static uint8_t toastDelta = 0;
static bool toastImproved = false;
static bool toastActive = false;
static size_t peakFree = 0;
static size_t peakLargest = 0;
static size_t minFree = 0;
static size_t minLargest = 0;
static bool conditionPending = false;
static uint32_t lastConditionMs = 0;

static uint8_t computePercent(size_t freeHeap, size_t largestBlock, bool updatePeaks) {
    if (updatePeaks) {
        if (freeHeap > peakFree) peakFree = freeHeap;
        if (largestBlock > peakLargest) peakLargest = largestBlock;
    }

    float freeNorm = peakFree > 0 ? (float)freeHeap / (float)peakFree : 0.0f;
    float contigNorm = peakLargest > 0 ? (float)largestBlock / (float)peakLargest : 0.0f;
    float thresholdNorm = 1.0f;
    if (HeapPolicy::kMinHeapForTls > 0 && HeapPolicy::kMinContigForTls > 0) {
        float freeGate = (float)freeHeap / (float)HeapPolicy::kMinHeapForTls;
        float contigGate = (float)largestBlock / (float)HeapPolicy::kMinContigForTls;
        thresholdNorm = (freeGate < contigGate) ? freeGate : contigGate;
    }

    float health = freeNorm < contigNorm ? freeNorm : contigNorm;
    if (thresholdNorm < health) health = thresholdNorm;

    float fragRatio = freeHeap > 0 ? (float)largestBlock / (float)freeHeap : 0.0f;
    float fragPenalty = fragRatio / HeapPolicy::kHealthFragPenaltyScale;  // Penalize fragmentation when largest << total free
    if (fragPenalty < 0.0f) fragPenalty = 0.0f;
    if (fragPenalty > 1.0f) fragPenalty = 1.0f;
    health *= fragPenalty;

    if (health < 0.0f) health = 0.0f;
    if (health > 1.0f) health = 1.0f;

    int pct = (int)(health * 100.0f + 0.5f);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (uint8_t)pct;
}

void update() {
    uint32_t now = millis();
    if (now - lastSampleMs < HeapPolicy::kHealthSampleIntervalMs) {
        return;
    }
    lastSampleMs = now;

    size_t freeHeap = ESP.getFreeHeap();
    size_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    if (peakFree == 0 || peakLargest == 0) {
        peakFree = freeHeap;
        peakLargest = largestBlock;
    }
    if (minFree == 0 || freeHeap < minFree) minFree = freeHeap;
    if (minLargest == 0 || largestBlock < minLargest) minLargest = largestBlock;
    uint8_t newPct = computePercent(freeHeap, largestBlock, true);

    int delta = (int)newPct - (int)heapHealthPct;
    uint8_t deltaAbs = (delta < 0) ? (uint8_t)(-delta) : (uint8_t)delta;
    heapHealthPct = newPct;

    bool contigLow = largestBlock < HeapPolicy::kProactiveTlsConditioning;
    bool pctLow = newPct <= HeapPolicy::kHealthConditionTriggerPct;
    if (!conditionPending) {
        if (pctLow && contigLow &&
            (lastConditionMs == 0 || (now - lastConditionMs) >= HeapPolicy::kHealthConditionCooldownMs)) {
            conditionPending = true;
        }
    } else {
        bool pctRecovered = newPct >= HeapPolicy::kHealthConditionClearPct;
        bool contigRecovered = largestBlock >= HeapPolicy::kProactiveTlsConditioning;
        if (pctRecovered && contigRecovered) {
            conditionPending = false;
        }
    }

    if (delta != 0 && deltaAbs >= HeapPolicy::kHealthToastMinDelta) {
        if (now - lastToastMs >= HeapPolicy::kHealthToastDurationMs) {
            toastDelta = deltaAbs;
            toastImproved = delta > 0;
            toastActive = true;
            toastStartMs = now;
            lastToastMs = now;
        }
    }
}

uint8_t getPercent() {
    return heapHealthPct;
}

void resetPeaks(bool suppressToast) {
    peakFree = ESP.getFreeHeap();
    peakLargest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    minFree = peakFree;
    minLargest = peakLargest;
    heapHealthPct = computePercent(peakFree, peakLargest, false);
    conditionPending = false;
    lastConditionMs = millis();

    if (suppressToast) {
        toastActive = false;
        toastDelta = 0;
        toastImproved = false;
        lastToastMs = millis();
        lastSampleMs = millis();
    }
}

bool shouldShowToast() {
    if (!toastActive) return false;
    if (millis() - toastStartMs >= HeapPolicy::kHealthToastDurationMs) {
        toastActive = false;
        return false;
    }
    return true;
}

bool isToastImproved() {
    return toastImproved;
}

uint8_t getToastDelta() {
    return toastDelta;
}

uint32_t getMinFree() {
    return (uint32_t)minFree;
}

uint32_t getMinLargest() {
    return (uint32_t)minLargest;
}

bool consumeConditionRequest() {
    if (!conditionPending) return false;
    conditionPending = false;
    return true;
}

}  // namespace HeapHealth
