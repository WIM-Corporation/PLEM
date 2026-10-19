// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

/**
 * @file
 * @brief A byte-level seqlock (seqlock::) and SeqlockData<T>, its in-process slot.
 *        The same protocol serves a byte-addressed region shared between processes.
 *
 * One writer per sequence. The sequence is odd while a write is in flight and
 * even otherwise; a reader accepts a copy only when the sequence was even and
 * unchanged across it.
 *
 * The payload is copied with relaxed atomic accesses (see Partition), so a
 * reader overlapping the writer is not a data race; the fences order those
 * accesses against the sequence (Boehm, "Can seqlocks get along with
 * programming language memory models?", MSPC 2012). The private side is
 * plain memory.
 *
 * @realtime_safe seqlock::write() and try_read(): no allocation, no syscall,
 *                never wait on the writer. SeqlockData::read() may spin.
 */

namespace plem::core::test {
struct SeqlockDataTestPeer;  // test/include/plem/core/test/seqlock_data_test_peer.hpp
}  // namespace plem::core::test

namespace plem::core::threading::seqlock {

/// How the shared bytes are split into atomic objects. Every access to one
/// region must use the same split with the same bounds: a word and a byte
/// covering the same address are different atomic objects, and the C++ model
/// gives no guarantee when both are used.
enum class Partition : std::uint8_t {
    WORDS,  ///< 8-byte words where aligned, bytes for the head and tail; fixed slots (SeqlockData)
    BYTES,  ///< single bytes; any prefix size of one region (a byte-addressed region shared between processes)
};

/// Copy @p n bytes from private @p src into shared @p dst with relaxed atomic stores.
inline void store_relaxed(void* dst, const void* src, std::size_t n, Partition p = Partition::WORDS) noexcept {
    auto* d = static_cast<unsigned char*>(dst);
    const auto* s = static_cast<const unsigned char*>(src);
    std::size_t i = 0;
    for (; p == Partition::WORDS && i < n && (reinterpret_cast<std::uintptr_t>(d + i) % 8 != 0); ++i) {
        std::atomic_ref<unsigned char>(d[i]).store(s[i], std::memory_order_relaxed);
    }
    for (; p == Partition::WORDS && i + 8 <= n; i += 8) {
        uint64_t w;
        std::memcpy(&w, s + i, 8);
        std::atomic_ref<uint64_t>(*reinterpret_cast<uint64_t*>(d + i)).store(w, std::memory_order_relaxed);
    }
    for (; i < n; ++i) {
        std::atomic_ref<unsigned char>(d[i]).store(s[i], std::memory_order_relaxed);
    }
}

/// Copy @p n bytes from shared @p src into private @p dst with relaxed atomic loads.
inline void load_relaxed(void* dst, const void* src, std::size_t n, Partition p = Partition::WORDS) noexcept {
    auto* d = static_cast<unsigned char*>(dst);
    auto* s = const_cast<unsigned char*>(static_cast<const unsigned char*>(src));  // atomic_ref needs non-const
    std::size_t i = 0;
    for (; p == Partition::WORDS && i < n && (reinterpret_cast<std::uintptr_t>(s + i) % 8 != 0); ++i) {
        d[i] = std::atomic_ref<unsigned char>(s[i]).load(std::memory_order_relaxed);
    }
    for (; p == Partition::WORDS && i + 8 <= n; i += 8) {
        const uint64_t w =
            std::atomic_ref<uint64_t>(*reinterpret_cast<uint64_t*>(s + i)).load(std::memory_order_relaxed);
        std::memcpy(d + i, &w, 8);
    }
    for (; i < n; ++i) {
        d[i] = std::atomic_ref<unsigned char>(s[i]).load(std::memory_order_relaxed);
    }
}

static_assert(std::atomic_ref<uint64_t>::is_always_lock_free && std::atomic_ref<unsigned char>::is_always_lock_free);
static_assert(std::atomic_ref<uint64_t>::required_alignment == 8);

/// Publish @p n bytes from @p src into @p dst under @p seq.
///
/// The writer derives parity from the stored value rather than blindly
/// incrementing: a writer that died inside its write left the sequence odd,
/// and its restarted successor must not come out with the parity inverted
/// (readers would then accept copies taken mid-write and reject complete ones).
inline void write(std::atomic<uint64_t>& seq, void* dst, const void* src, std::size_t n,
                  Partition p = Partition::WORDS) noexcept {
    uint64_t s = seq.load(std::memory_order_relaxed);
    s += s & 1;                                   // even base, even after a dead writer
    seq.store(s + 1, std::memory_order_relaxed);  // odd: write in progress
    std::atomic_thread_fence(std::memory_order_release);
    store_relaxed(dst, src, n, p);
    std::atomic_thread_fence(std::memory_order_release);
    seq.store(s + 2, std::memory_order_relaxed);  // even: write complete
}

/// One consistency attempt: copy @p n bytes from @p src into @p dst.
///
/// @return false when a write was in flight or landed mid-copy. @p dst is left
///         untouched when the writer was already inside its write, but may hold
///         a torn copy when a write landed mid-copy; on false it must not be used.
[[nodiscard]] inline bool try_read(const std::atomic<uint64_t>& seq, void* dst, const void* src, std::size_t n,
                                   Partition p = Partition::WORDS) noexcept {
    const uint64_t seq1 = seq.load(std::memory_order_acquire);
    if (seq1 & 1) {
        return false;
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    load_relaxed(dst, src, n, p);
    std::atomic_thread_fence(std::memory_order_acquire);
    return seq.load(std::memory_order_acquire) == seq1;
}

}  // namespace plem::core::threading::seqlock

namespace plem::core::threading {

/**
 * @brief In-process seqlock: consistent snapshots of data larger than one atomic
 *
 * A reader never uses a copy the writer overlapped; it discards it and retries.
 * The protocol is seqlock:: above. read() retries a bounded number of times;
 * try_read() makes one attempt and never waits.
 *
 * @tparam T Data type (must be trivially copyable)
 *
 * @thread_safety Single writer, multiple readers
 * @realtime_safe Writer never blocks; read() may spin briefly, try_read() never waits
 */
template <typename T>
class SeqlockData {
    static_assert(std::is_trivially_copyable_v<T>, "SeqlockData requires trivially copyable type");
    static_assert(alignof(T) <= 8, "payload storage is 8-byte aligned");

  public:
    static constexpr uint32_t kMaxRetries = 10000;

    SeqlockData() : sequence_(0) {
        const T initial{};  // T's own member defaults, not zeros
        std::memcpy(data_, &initial, sizeof(T));
    }

    /**
     * @brief Publish a new value
     *
     * O(1) operation that never blocks. Safe for RT context.
     *
     * @param value Data to write
     * @realtime_safe Yes - no allocations, no syscalls, no blocking
     */
    void write(const T& value) noexcept {
        seqlock::write(sequence_, &data_, &value, sizeof(T));
    }

    /**
     * @brief Read a consistent snapshot
     *
     * May retry internally if a write is in progress.
     * Worst case retry count is bounded by kMaxRetries.
     *
     * Use when the writer always completes promptly relative to this reader
     * (it outranks the reader on the same core, or keeps running on another
     * core); the short spin then buys the freshest value.
     *
     * @param[out] value Buffer to read into
     * @return true if read successful, false if max retries exceeded
     *
     * @realtime_safe Mostly - may spin briefly during concurrent write
     */
    [[nodiscard]] bool read(T& value) const noexcept {
        for (uint32_t retry = 0; retry < kMaxRetries; ++retry) {
            if (try_read(value)) {
                return true;  // Consistent read
            }
            // Write in progress or landed mid-copy, retry
        }
        return false;  // Max retries exceeded
    }

    /**
     * @brief Single-attempt read for readers that must not wait on the writer
     *
     * Exactly one consistency attempt: returns false immediately when a
     * write is in progress or landed mid-copy. Use when spinning cannot help:
     * a reader that outranks the writer on the same core (the writer cannot
     * resume while the reader spins: priority inversion), or a
     * non-RT writer that may sit descheduled mid-write for milliseconds. On
     * false the caller keeps its previous copy (zero-order hold).
     *
     * @param[out] value Buffer to read into (may hold a torn copy on false;
     *                   must not be used)
     * @return true on a consistent read, false when a write was in flight
     *
     * @realtime_safe Yes - single bounded attempt, never spins
     */
    [[nodiscard]] bool try_read(T& value) const noexcept {
        return seqlock::try_read(sequence_, &value, &data_, sizeof(T));
    }

  private:
    /// Test-only peer (test_seqlock_data.cpp): simulates a writer preempted
    /// inside its critical section (odd sequence), a state write() can never
    /// leave behind, so the reader contracts are testable deterministically.
    friend struct ::plem::core::test::SeqlockDataTestPeer;

    alignas(64) std::atomic<uint64_t> sequence_;  // Cache-line aligned
    /// Raw storage (implicitly creates the 8-byte words seqlock:: accesses atomically).
    alignas(8) unsigned char data_[sizeof(T)];
};

}  // namespace plem::core::threading
