#include "heap_gates.h"
#include "heap_policy.h"
#include <Arduino.h>
#include <esp_heap_caps.h>

namespace HeapGates {

TlsGateStatus checkTlsGates() {
    size_t freeHeap = ESP.getFreeHeap();
    size_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    TlsGateFailure failure = TlsGateFailure::None;
    if (largestBlock < HeapPolicy::kMinContigForTls) {
        failure = TlsGateFailure::Fragmented;
    } else if (freeHeap < HeapPolicy::kMinHeapForTls) {
        failure = TlsGateFailure::LowHeap;
    }
    return {freeHeap, largestBlock, failure};
}

bool canTls(const TlsGateStatus& status, char* outError, size_t outErrorLen) {
    if (status.failure == TlsGateFailure::None) {
        return true;
    }
    if (!outError || outErrorLen == 0) {
        return false;
    }
    if (status.failure == TlsGateFailure::Fragmented) {
        snprintf(outError, outErrorLen, "FRAGMENTED: %uKB",
                 (unsigned int)(status.largestBlock / 1024));
    } else {
        snprintf(outError, outErrorLen, "LOW HEAP: %uKB",
                 (unsigned int)(status.freeHeap / 1024));
    }
    return false;
}

bool shouldProactivelyCondition(const TlsGateStatus& status) {
    return (status.largestBlock < HeapPolicy::kProactiveTlsConditioning &&
            status.largestBlock >= HeapPolicy::kMinContigForTls);
}

}  // namespace HeapGates
