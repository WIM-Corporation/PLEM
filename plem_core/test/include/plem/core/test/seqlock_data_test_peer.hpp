// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file seqlock_data_test_peer.hpp
/// @brief The test-only peer SeqlockData friends. Test support: installed
/// beside plem_core's headers only in a BUILD_TESTING build, so another
/// package's tests use this one definition.

#pragma once

#include <atomic>
#include <cstdint>

#include <plem/core/threading/seqlock.hpp>

namespace plem::core::test {

/// Freezes a slot in the states a preempted or descheduled writer leaves
/// behind, which write() never exposes: the deterministic stand-in for "the
/// writer stopped inside its critical section".
struct SeqlockDataTestPeer {
    template <typename T>
    static uint64_t sequence(const threading::SeqlockData<T>& lock) {
        return lock.sequence_.load(std::memory_order_acquire);
    }

    /// Writer stopped inside its critical section: sequence odd.
    template <typename T>
    static void freeze_writer_mid_write(threading::SeqlockData<T>& lock) {
        lock.sequence_.fetch_add(1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
    }

    /// The frozen writer resumes and completes: data landed, sequence even.
    template <typename T>
    static void resume_writer_and_complete(threading::SeqlockData<T>& lock, const T& value) {
        threading::seqlock::store_relaxed(lock.data_, &value, sizeof(T));
        std::atomic_thread_fence(std::memory_order_release);
        lock.sequence_.fetch_add(1, std::memory_order_relaxed);
    }
};

}  // namespace plem::core::test
