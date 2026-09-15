#include "sync_test_logger.h"

#include <limits.h>

// =====================================================
// TEST SETTINGS
// =====================================================

// One synchronization measurement per node per second.
static constexpr uint32_t LOG_INTERVAL_MS = 1000UL;

// 30-minute synchronization-drift test.
static constexpr uint32_t TEST_DURATION_MS =
    30UL * 60UL * 1000UL;

static constexpr uint32_t DAY_MS = 86400000UL;
static constexpr uint8_t NODE_COUNT = 4;
static constexpr uint8_t SUMMARY_SAMPLE_COUNT = 10;

// =====================================================
// NODE STATISTICS
// =====================================================

struct NodeSyncStatistics {
    uint32_t totalSamples = 0;
    uint64_t totalAbsoluteErrorMs = 0;

    uint32_t minimumAbsoluteErrorMs = UINT32_MAX;
    uint32_t maximumAbsoluteErrorMs = 0;

    uint32_t initialErrors[SUMMARY_SAMPLE_COUNT] = {};
    uint8_t initialCount = 0;

    uint32_t finalErrors[SUMMARY_SAMPLE_COUNT] = {};
    uint8_t finalCount = 0;
    uint8_t finalWriteIndex = 0;
};

static NodeSyncStatistics nodeStatistics[NODE_COUNT];

static uint32_t lastLogTimeMs[NODE_COUNT] = {
    0, 0, 0, 0
};

static uint8_t detectedNodeMask = 0;

static bool testStarted = false;
static bool testCompleted = false;

static uint32_t testStartMillis = 0;

// =====================================================
// TIME-DIFFERENCE CALCULATION
// =====================================================

static int32_t calculateSignedErrorMs(
    uint32_t nodeTimeMs,
    uint32_t centerReceiveTimeMs
) {
    int64_t difference =
        static_cast<int64_t>(nodeTimeMs) -
        static_cast<int64_t>(centerReceiveTimeMs);

    const int64_t halfDay =
        static_cast<int64_t>(DAY_MS) / 2;

    // Handle a test that crosses midnight.
    if (difference > halfDay) {
        difference -= DAY_MS;
    } else if (difference < -halfDay) {
        difference += DAY_MS;
    }

    return static_cast<int32_t>(difference);
}

static uint32_t getAbsoluteErrorMs(int32_t errorMs) {
    if (errorMs < 0) {
        return static_cast<uint32_t>(
            -static_cast<int64_t>(errorMs)
        );
    }

    return static_cast<uint32_t>(errorMs);
}

// =====================================================
// STATISTICS
// =====================================================

static void addErrorSample(
    uint8_t nodeIndex,
    uint32_t absoluteErrorMs
) {
    NodeSyncStatistics &stats =
        nodeStatistics[nodeIndex];

    stats.totalSamples++;
    stats.totalAbsoluteErrorMs += absoluteErrorMs;

    if (
        stats.minimumAbsoluteErrorMs == UINT32_MAX ||
        absoluteErrorMs < stats.minimumAbsoluteErrorMs
    ) {
        stats.minimumAbsoluteErrorMs =
            absoluteErrorMs;
    }

    if (
        absoluteErrorMs >
        stats.maximumAbsoluteErrorMs
    ) {
        stats.maximumAbsoluteErrorMs =
            absoluteErrorMs;
    }

    // Store the first 10 values.
    if (
        stats.initialCount <
        SUMMARY_SAMPLE_COUNT
    ) {
        stats.initialErrors[
            stats.initialCount
        ] = absoluteErrorMs;

        stats.initialCount++;
    }

    // Continuously retain the latest 10 values.
    stats.finalErrors[
        stats.finalWriteIndex
    ] = absoluteErrorMs;

    stats.finalWriteIndex =
        (stats.finalWriteIndex + 1) %
        SUMMARY_SAMPLE_COUNT;

    if (
        stats.finalCount <
        SUMMARY_SAMPLE_COUNT
    ) {
        stats.finalCount++;
    }
}

static float calculateAverage(
    const uint32_t *values,
    uint8_t count
) {
    if (count == 0) {
        return 0.0f;
    }

    uint64_t sum = 0;

    for (uint8_t i = 0; i < count; i++) {
        sum += values[i];
    }

    return static_cast<float>(sum) /
           static_cast<float>(count);
}

// =====================================================
// FINAL SUMMARY
// =====================================================

static void printFinalSummary() {
    Serial.println();
    Serial.println(
        "=========== TIME-SYNCHRONIZATION RESULT ==========="
    );

    Serial.println("Test duration: 30 minutes");

    Serial.println(
        "node,initial_error_ms,final_error_ms,"
        "mean_error_ms,min_error_ms,max_error_ms,samples"
    );

    for (
        uint8_t nodeIndex = 0;
        nodeIndex < NODE_COUNT;
        nodeIndex++
    ) {
        const NodeSyncStatistics &stats =
            nodeStatistics[nodeIndex];

        const float initialAverage =
            calculateAverage(
                stats.initialErrors,
                stats.initialCount
            );

        const float finalAverage =
            calculateAverage(
                stats.finalErrors,
                stats.finalCount
            );

        const float overallAverage =
            stats.totalSamples == 0
                ? 0.0f
                : static_cast<float>(
                      stats.totalAbsoluteErrorMs
                  ) /
                  static_cast<float>(
                      stats.totalSamples
                  );

        const uint32_t minimumError =
            stats.minimumAbsoluteErrorMs ==
                    UINT32_MAX
                ? 0
                : stats.minimumAbsoluteErrorMs;

        Serial.printf(
            "%u,%.3f,%.3f,%.3f,%lu,%lu,%lu\n",
            static_cast<unsigned>(
                nodeIndex + 1
            ),
            initialAverage,
            finalAverage,
            overallAverage,
            static_cast<unsigned long>(
                minimumError
            ),
            static_cast<unsigned long>(
                stats.maximumAbsoluteErrorMs
            ),
            static_cast<unsigned long>(
                stats.totalSamples
            )
        );
    }

    Serial.println(
        "===================================================="
    );

    Serial.println(
        "Reset the centre to begin another test."
    );
}

// =====================================================
// PUBLIC FUNCTIONS
// =====================================================

void initTimeSyncTestLogger() {
    for (
        uint8_t i = 0;
        i < NODE_COUNT;
        i++
    ) {
        nodeStatistics[i] =
            NodeSyncStatistics();

        lastLogTimeMs[i] = 0;
    }

    detectedNodeMask = 0;
    testStarted = false;
    testCompleted = false;
    testStartMillis = 0;

    Serial.println();
    Serial.println(
        "#INFO,Time-synchronization logger initialized"
    );

    Serial.println(
        "#INFO,Waiting for Nodes 1-4"
    );

    Serial.println(
        "record,elapsed_s,node_id,sequence,"
        "node_time_ms,center_time_ms,"
        "error_ms,absolute_error_ms"
    );
}

void processTimeSyncTestRecord(
    const IMU_Node_Frame &frame,
    uint32_t centerReceiveTimeMs
) {
    if (
        frame.nodeID < 1 ||
        frame.nodeID > 4 ||
        frame.timestamp_ms == 0
    ) {
        return;
    }

    const uint8_t nodeIndex =
        frame.nodeID - 1;

    detectedNodeMask |=
        static_cast<uint8_t>(
            1U << nodeIndex
        );

    // Start only after all four nodes have sent
    // at least one valid timestamped packet.
    if (
        !testStarted &&
        detectedNodeMask == 0x0F
    ) {
        testStarted = true;
        testStartMillis = millis();

        Serial.println();
        Serial.println(
            "#EVENT,TIME_SYNC_TEST_STARTED"
        );

        Serial.println(
            "#INFO,All four nodes detected"
        );

        Serial.println(
            "#INFO,Test duration=1800 seconds"
        );
    }

    if (
        !testStarted ||
        testCompleted
    ) {
        return;
    }

    const uint32_t currentMillis = millis();

    // Print only one measurement per second
    // for each node.
    if (
        lastLogTimeMs[nodeIndex] != 0 &&
        currentMillis -
            lastLogTimeMs[nodeIndex] <
            LOG_INTERVAL_MS
    ) {
        return;
    }

    lastLogTimeMs[nodeIndex] =
        currentMillis;

    const int32_t signedErrorMs =
        calculateSignedErrorMs(
            frame.timestamp_ms,
            centerReceiveTimeMs
        );

    const uint32_t absoluteErrorMs =
        getAbsoluteErrorMs(
            signedErrorMs
        );

    addErrorSample(
        nodeIndex,
        absoluteErrorMs
    );

    const float elapsedSeconds =
        static_cast<float>(
            currentMillis -
            testStartMillis
        ) /
        1000.0f;

    Serial.printf(
        "SYNC,%.3f,%u,%u,%lu,%lu,%ld,%lu\n",
        elapsedSeconds,
        static_cast<unsigned>(
            frame.nodeID
        ),
        static_cast<unsigned>(
            frame.sequenceNo
        ),
        static_cast<unsigned long>(
            frame.timestamp_ms
        ),
        static_cast<unsigned long>(
            centerReceiveTimeMs
        ),
        static_cast<long>(
            signedErrorMs
        ),
        static_cast<unsigned long>(
            absoluteErrorMs
        )
    );
}

void handleTimeSyncTestLogger() {
    if (
        !testStarted ||
        testCompleted
    ) {
        return;
    }

    if (
        millis() - testStartMillis >=
        TEST_DURATION_MS
    ) {
        testCompleted = true;
        printFinalSummary();
    }
}