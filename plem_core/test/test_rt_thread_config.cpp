// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_rt_thread_config.cpp
 * @brief Unit tests for RT thread configuration utilities
 */

#include <gtest/gtest.h>

#include <cerrno>
#include <fstream>
#include <string>
#include <thread>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

#include <plem/core/threading/rt_thread_config.hpp>

using namespace plem::core::threading;

class RtThreadConfigTest : public ::testing::Test {};

/**
 * @brief Test detect_isolated_cpu on a system with isolation configured.
 *
 * This test verifies that detect_isolated_cpu correctly reads and parses
 * the kernel's isolated CPU configuration from /sys/devices/system/cpu/isolated.
 *
 * Prerequisites:
 *   - Kernel boot parameter: isolcpus=managed_irq,domain,<cpu_id>
 *   - /sys/devices/system/cpu/isolated file exists and is readable
 */
TEST_F(RtThreadConfigTest, DetectIsolatedCpuReadsKernelConfig) {
    // Read expected value directly from system file
    std::ifstream file("/sys/devices/system/cpu/isolated");
    if (!file.is_open()) {
        GTEST_SKIP() << "No isolated CPU configured (file not readable)";
    }

    std::string line;
    if (!std::getline(file, line) || line.empty()) {
        GTEST_SKIP() << "No isolated CPU configured (empty file)";
    }

    // The parsing itself is pinned by the hermetic ParseLastIsolatedCpu table
    // below; this integration check only covers the file-read plumbing.
    auto expected_cpu = parse_last_isolated_cpu(line);
    ASSERT_TRUE(expected_cpu.has_value()) << "Kernel isolated-CPU content should parse: '" << line << "'";

    auto result = detect_isolated_cpu();

    ASSERT_TRUE(result.has_value()) << "detect_isolated_cpu should return a value when isolation is configured";
    EXPECT_EQ(*result, *expected_cpu) << "Detected CPU should match parsed value from system file";
}

/**
 * @brief Hermetic table test for the isolated-CPU list parser: documented
 *        kernel-list formats and malformed inputs. Only the last token must
 *        parse completely; earlier tokens are not inspected; the parser does
 *        not throw.
 */
TEST(ParseLastIsolatedCpuTest, TableDrivenSyntaxAndMalformedInputs) {
    struct Case {
        const char* input;
        std::optional<int> expected;
    };
    const Case cases[] = {
        {"7", 7},
        {"7,11", 11},
        {"5-7", 7},
        {"5-7,9-11", 11},
        {"11\n", 11},
        {"11 \t", 11},
        {"", std::nullopt},
        {"\n", std::nullopt},
        {"abc", std::nullopt},
        {"7,", std::nullopt},
        {"7-", std::nullopt},
        {"3,junk", std::nullopt},
        {"1.5", std::nullopt},
        {"7 11", std::nullopt},
        {"99999999999", std::nullopt},  // int overflow
        {"-1", std::nullopt},           // leading '-' is a sign, not a range
        // Only the last token is validated; earlier tokens are not inspected
        // (the input is kernel-generated sysfs, see the header). These rows pin
        // that boundary.
        {"junk,5", 5},
        {"0-3,,5", 5},
        {"7-5", 5},
        {"--1", 1},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(std::string("input='") + c.input + "'");
        EXPECT_EQ(parse_last_isolated_cpu(c.input), c.expected);
    }
}

/**
 * @brief A range token followed by a single-CPU token returns the single CPU
 *        ("0-3,5" is 5, not the range end 3).
 */
TEST(ParseLastIsolatedCpuTest, RangeThenSingleTokenReturnsLastCpuNotRangeBound) {
    EXPECT_EQ(parse_last_isolated_cpu("0-3,5"), std::optional<int>(5));
}

TEST_F(RtThreadConfigTest, SchedulingPolicyValues) {
    // Passed to pthread_setschedparam as is: they must be <sched.h>'s values
    EXPECT_EQ(static_cast<int>(SchedulingPolicy::OTHER), SCHED_OTHER);
    EXPECT_EQ(static_cast<int>(SchedulingPolicy::FIFO), SCHED_FIFO);
    EXPECT_EQ(static_cast<int>(SchedulingPolicy::RR), SCHED_RR);
}

// apply_rt_thread_config applies policy/priority/affinity to the calling
// thread, reporting each aspect's outcome without ever throwing.

TEST_F(RtThreadConfigTest, ApplyOtherPolicyIsNoOp) {
    RtThreadConfig cfg;
    cfg.policy = SchedulingPolicy::OTHER;
    cfg.priority = 0;

    const auto result = apply_rt_thread_config(cfg);

    EXPECT_TRUE(result.scheduling_ok);
    EXPECT_TRUE(result.ok());
    EXPECT_EQ(result.sched_errno, 0);
}

TEST_F(RtThreadConfigTest, ApplyFifoRejectsOutOfRangePriority) {
    // 9999 is far above sched_get_priority_max(SCHED_FIFO) (== 99 on Linux);
    // the range check must reject it before issuing the syscall (no privilege
    // needed) so the failure is deterministic on every host.
    RtThreadConfig cfg;
    cfg.policy = SchedulingPolicy::FIFO;
    cfg.priority = 9999;

    const auto result = apply_rt_thread_config(cfg);

    EXPECT_FALSE(result.scheduling_ok);
    EXPECT_FALSE(result.ok());
#ifdef __linux__
    EXPECT_EQ(result.sched_errno, EINVAL);
#endif
}

TEST_F(RtThreadConfigTest, ApplyFifoValidPriorityAppliesOrIsGracefullyDenied) {
    // Run in a throwaway thread so a successful promotion does not leave the
    // gtest main thread at SCHED_FIFO and perturb later tests.
    RtApplyResult result;
    int observed_policy = -1;
    std::thread worker([&] {
        RtThreadConfig cfg;
        cfg.policy = SchedulingPolicy::FIFO;
        cfg.priority = 10;
        result = apply_rt_thread_config(cfg);
#ifdef __linux__
        observed_policy = sched_getscheduler(0);
#endif
    });
    worker.join();

#ifdef __linux__
    if (result.scheduling_ok) {
        EXPECT_EQ(observed_policy, SCHED_FIFO) << "FIFO applied → thread must report SCHED_FIFO";
    } else {
        // Unprivileged / RT-bandwidth-exhausted host: must degrade gracefully, not
        // crash. EPERM (missing CAP_SYS_NICE) is the typical errno, but EAGAIN and
        // others are valid too; assert only that a failure was recorded (the intent).
        EXPECT_NE(result.sched_errno, 0);
    }
#endif
}

TEST_F(RtThreadConfigTest, ApplyAffinityAppliesOrIsGracefullyDenied) {
    RtApplyResult result;
    bool cpu0_in_mask = false;
    std::thread worker([&] {
        RtThreadConfig cfg;
        cfg.policy = SchedulingPolicy::OTHER;  // affinity-only; no RT privilege required
        cfg.affinity = CpuAffinity{{0}};
        result = apply_rt_thread_config(cfg);
#ifdef __linux__
        cpu_set_t got;
        CPU_ZERO(&got);
        if (pthread_getaffinity_np(pthread_self(), sizeof(got), &got) == 0) {
            cpu0_in_mask = CPU_ISSET(0, &got);
        }
#endif
    });
    worker.join();

#ifdef __linux__
    if (result.affinity_ok) {
        EXPECT_TRUE(cpu0_in_mask) << "affinity {0} applied → CPU 0 must be present in the mask";
    } else {
        // Pinning denied (rare for an OTHER thread): must record a non-zero errno,
        // symmetric with the scheduling graceful-deny path.
        EXPECT_NE(result.affinity_errno, 0);
    }
#endif
}

// A CPU index the mask cannot hold refuses the pinning (EINVAL) instead of
// pinning to the remaining CPUs; the thread's mask is left as it was.
TEST_F(RtThreadConfigTest, ACpuOutsideTheMaskRefusesThePinning) {
    for (const int bad : {-1, CPU_SETSIZE}) {
        RtApplyResult result;
        cpu_set_t before;
        cpu_set_t after;
        std::thread worker([&] {
            pthread_getaffinity_np(pthread_self(), sizeof(before), &before);
            result = apply_rt_thread_config({.policy = SchedulingPolicy::OTHER, .affinity = CpuAffinity{{0, bad}}});
            pthread_getaffinity_np(pthread_self(), sizeof(after), &after);
        });
        worker.join();
        EXPECT_FALSE(result.affinity_ok) << "cpu " << bad;
        EXPECT_EQ(result.affinity_errno, EINVAL) << "cpu " << bad;
        EXPECT_TRUE(CPU_EQUAL(&before, &after)) << "cpu " << bad << ": not pinned to CPU 0 alone";
    }
}
