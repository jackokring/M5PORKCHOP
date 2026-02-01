#pragma once

#include <cstddef>
#include <cstdint>

namespace HeapGates {
    enum class TlsGateFailure : uint8_t {
        None = 0,
        Fragmented,
        LowHeap
    };

    struct TlsGateStatus {
        size_t freeHeap;
        size_t largestBlock;
        TlsGateFailure failure;
    };

    // Snapshot current heap and evaluate TLS gating status.
    TlsGateStatus checkTlsGates();

    // Return true if TLS can proceed, and optionally format an error string.
    bool canTls(const TlsGateStatus& status, char* outError, size_t outErrorLen);

    // True when we are above the TLS gate but below the proactive threshold.
    bool shouldProactivelyCondition(const TlsGateStatus& status);
}
