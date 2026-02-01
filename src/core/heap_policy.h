#pragma once

#include <cstddef>
#include <cstdint>

namespace HeapPolicy {
    // TLS gating thresholds
    static constexpr size_t kMinHeapForTls = 35000;
    static constexpr size_t kMinContigForTls = 35000;
    static constexpr size_t kProactiveTlsConditioning = 45000;

    // General allocation safety thresholds
    static constexpr size_t kMinHeapForOinkNetworkAdd = 30000;
    static constexpr size_t kMinHeapForHandshakeAdd = 60000;
    static constexpr size_t kMinHeapForReconGrowth = 20000;
    static constexpr size_t kMinHeapForDnhGrowth = 40000;
    static constexpr size_t kMinHeapForSpectrumGrowth = 20000;

    // Heap stabilization / recovery thresholds
    static constexpr size_t kHeapStableThreshold = 50000;
    static constexpr size_t kFileServerRecoveryThreshold = 50000;
    static constexpr size_t kFileServerMinHeap = 40000;
    static constexpr size_t kFileServerMinLargest = 30000;

    // Allocation slack (allocator overhead / fragmentation cushion)
    static constexpr size_t kReserveSlackSmall = 256;
    static constexpr size_t kReserveSlackLarge = 1024;
    static constexpr size_t kPmkidAllocSlack = 256;
    static constexpr size_t kHandshakeAllocSlack = 1024;

    // Mode-specific thresholds
    static constexpr size_t kWarhogHeapWarning = 40000;
    static constexpr size_t kWarhogHeapCritical = 25000;
    static constexpr size_t kDnhInjectMinHeap = 80000;
    static constexpr size_t kPigSyncMinContig = 26000;

    // Heap health sampling/tuning
    static constexpr uint32_t kHealthSampleIntervalMs = 1000;
    static constexpr uint32_t kHealthToastDurationMs = 5000;
    static constexpr uint8_t kHealthToastMinDelta = 5;
    static constexpr uint8_t kHealthConditionTriggerPct = 65;
    static constexpr uint8_t kHealthConditionClearPct = 75;
    static constexpr uint32_t kHealthConditionCooldownMs = 30000;
    static constexpr float kHealthFragPenaltyScale = 0.60f;

    // Stress test guardrail
    static constexpr size_t kStressMinHeap = 70000;
}
