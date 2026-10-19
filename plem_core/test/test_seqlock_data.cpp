// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include <plem/core/test/seqlock_data_test_peer.hpp>
#include <plem/core/types/shared_joint_data.hpp>

namespace plem::core::threading {

using test::SeqlockDataTestPeer;

struct TestData {
    double values[4];
    int count;
};

class SeqlockDataTest : public ::testing::Test {
  protected:
    SeqlockData<TestData> seqlock_;
};

TEST_F(SeqlockDataTest, WriteIncrementsSequence) {
    TestData data{};
    data.values[0] = 1.0;
    data.count = 42;

    seqlock_.write(data);

    // After write, sequence should be 2 (incremented twice)
    EXPECT_EQ(SeqlockDataTestPeer::sequence(seqlock_), 2);
}

TEST_F(SeqlockDataTest, ReadReturnsWrittenData) {
    TestData write_data{};
    write_data.values[0] = 3.14;
    write_data.values[1] = 2.71;
    write_data.count = 100;

    seqlock_.write(write_data);

    TestData read_data{};
    EXPECT_TRUE(seqlock_.read(read_data));
    EXPECT_DOUBLE_EQ(read_data.values[0], 3.14);
    EXPECT_DOUBLE_EQ(read_data.values[1], 2.71);
    EXPECT_EQ(read_data.count, 100);
}

TEST_F(SeqlockDataTest, MultipleWritesAndReads) {
    for (int i = 0; i < 100; ++i) {
        TestData write_data{};
        write_data.count = i;
        seqlock_.write(write_data);

        TestData read_data{};
        EXPECT_TRUE(seqlock_.read(read_data));
        EXPECT_EQ(read_data.count, i);
    }

    // Sequence should be 200 (2 * 100 writes)
    EXPECT_EQ(SeqlockDataTestPeer::sequence(seqlock_), 200);
}

TEST_F(SeqlockDataTest, ConcurrentWriterReader) {
    std::atomic<bool> running{true};
    std::atomic<int> read_count{0};
    std::atomic<int> inconsistent_count{0};

    std::thread writer([this, &running]() {
        int counter = 0;
        while (running.load(std::memory_order_relaxed)) {
            TestData data{};
            // Set all values to same number for consistency check
            data.values[0] = static_cast<double>(counter);
            data.values[1] = static_cast<double>(counter);
            data.values[2] = static_cast<double>(counter);
            data.values[3] = static_cast<double>(counter);
            data.count = counter;
            seqlock_.write(data);
            ++counter;
        }
    });

    std::thread reader([this, &running, &read_count, &inconsistent_count]() {
        while (running.load(std::memory_order_relaxed)) {
            TestData data{};
            if (seqlock_.read(data)) {
                ++read_count;
                // Check consistency: all values should be same
                if (data.values[0] != data.values[1] || data.values[1] != data.values[2] ||
                    data.values[2] != data.values[3] || static_cast<int>(data.values[0]) != data.count) {
                    ++inconsistent_count;
                }
            }
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    running.store(false, std::memory_order_relaxed);

    writer.join();
    reader.join();

    // Should have many successful reads
    EXPECT_GT(read_count.load(), 0);
    // No inconsistent reads (seqlock guarantees atomicity)
    EXPECT_EQ(inconsistent_count.load(), 0) << "Detected " << inconsistent_count.load() << " inconsistent reads!";
}

class SharedJointDataTest : public ::testing::Test {
  protected:
    SharedJointData<6> shared_data_;
};

TEST_F(SharedJointDataTest, StartsNeverStamped) {
    EXPECT_EQ(shared_data_.last_cycle_ns.load(), 0) << "0 means the RT loop has not written a cycle yet";
}

TEST_F(SharedJointDataTest, JointActualValuesTorqueRoundTrip) {
    JointActualValues<6> write_data;
    for (size_t i = 0; i < 6; ++i) {
        write_data.states[i].position_actual_value = static_cast<double>(i) * 0.1;
        write_data.states[i].velocity_actual_value = static_cast<double>(i) * 0.01;
        write_data.states[i].torque_actual_value = static_cast<double>(i) * 1.5;
    }

    shared_data_.joint_actual.write(write_data);

    JointActualValues<6> read_data;
    EXPECT_TRUE(shared_data_.joint_actual.read(read_data));

    for (size_t i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(read_data.states[i].torque_actual_value, static_cast<double>(i) * 1.5);
    }
}

TEST_F(SharedJointDataTest, ConcurrentJointDataAccess) {
    // Simulated EtherCAT-to-controller joint data.
    std::atomic<bool> running{true};
    std::atomic<int> read_count{0};
    std::atomic<int> inconsistent_count{0};

    // EtherCAT-side writer.
    std::thread writer([this, &running]() {
        int cycle = 0;
        while (running.load(std::memory_order_relaxed)) {
            JointActualValues<6> data;
            // Set all joints to same value for consistency check
            for (size_t i = 0; i < 6; ++i) {
                data.states[i].position_actual_value = static_cast<double>(cycle);
                data.states[i].velocity_actual_value = static_cast<double>(cycle);
            }
            shared_data_.joint_actual.write(data);
            ++cycle;
        }
    });

    // Controller-side reader.
    std::thread reader([this, &running, &read_count, &inconsistent_count]() {
        while (running.load(std::memory_order_relaxed)) {
            JointActualValues<6> data;
            if (shared_data_.joint_actual.read(data)) {
                ++read_count;
                // Check consistency: all joints should have same position
                double first_pos = data.states[0].position_actual_value;
                for (size_t i = 1; i < 6; ++i) {
                    if (data.states[i].position_actual_value != first_pos) {
                        ++inconsistent_count;
                        break;
                    }
                }
            }
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    running.store(false, std::memory_order_relaxed);

    writer.join();
    reader.join();

    EXPECT_GT(read_count.load(), 0);
    EXPECT_EQ(inconsistent_count.load(), 0) << "Detected " << inconsistent_count.load() << " inconsistent reads!";
}

TEST_F(SeqlockDataTest, TryReadReturnsWrittenDataWhenIdle) {
    TestData write_data{};
    write_data.values[0] = 3.14;
    write_data.count = 7;
    seqlock_.write(write_data);

    TestData read_data{};
    EXPECT_TRUE(seqlock_.try_read(read_data));
    EXPECT_DOUBLE_EQ(read_data.values[0], 3.14);
    EXPECT_EQ(read_data.count, 7);
}

// Invariant: an RT reader that outranks (same core) or outlives
// (descheduled non-RT writer) the writer must fall back to its previous copy
// immediately: the writer cannot complete while the reader spins, so every
// retry is wasted RT budget (measured ~4.6 µs at kMaxRetries).
TEST_F(SeqlockDataTest, TryReadFailsImmediatelyWhileWriterFrozenMidWrite) {
    TestData initial{};
    initial.count = 1;
    seqlock_.write(initial);

    SeqlockDataTestPeer::freeze_writer_mid_write(seqlock_);

    TestData read_data{};
    read_data.count = 99;  // sentinel; must stay untouched on the odd-sequence path
    EXPECT_FALSE(seqlock_.try_read(read_data));
    EXPECT_EQ(read_data.count, 99);
}

TEST_F(SeqlockDataTest, TryReadRecoversAfterFrozenWriterCompletes) {
    TestData initial{};
    initial.count = 1;
    seqlock_.write(initial);
    SeqlockDataTestPeer::freeze_writer_mid_write(seqlock_);

    TestData completed{};
    completed.count = 2;
    SeqlockDataTestPeer::resume_writer_and_complete(seqlock_, completed);

    TestData read_data{};
    EXPECT_TRUE(seqlock_.try_read(read_data));
    EXPECT_EQ(read_data.count, 2);
}

TEST_F(SeqlockDataTest, ReadStillFailsAfterMaxRetriesWhileWriterFrozen) {
    // read() still gives up after kMaxRetries while the writer is frozen.
    SeqlockDataTestPeer::freeze_writer_mid_write(seqlock_);
    TestData read_data{};
    EXPECT_FALSE(seqlock_.read(read_data));
}

TEST_F(SeqlockDataTest, ConcurrentWriterTryReaderNeverAcceptsTornData) {
    std::atomic<bool> running{true};
    std::atomic<int> success_count{0};
    std::atomic<int> inconsistent_count{0};

    std::thread writer([this, &running]() {
        int counter = 0;
        while (running.load(std::memory_order_relaxed)) {
            TestData data{};
            data.values[0] = static_cast<double>(counter);
            data.values[1] = static_cast<double>(counter);
            data.values[2] = static_cast<double>(counter);
            data.values[3] = static_cast<double>(counter);
            data.count = counter;
            seqlock_.write(data);
            ++counter;
        }
    });

    std::thread reader([this, &running, &success_count, &inconsistent_count]() {
        while (running.load(std::memory_order_relaxed)) {
            TestData data{};
            if (seqlock_.try_read(data)) {
                ++success_count;
                if (data.values[0] != data.values[1] || data.values[1] != data.values[2] ||
                    data.values[2] != data.values[3] || static_cast<int>(data.values[0]) != data.count) {
                    ++inconsistent_count;
                }
            }
            // false = a write was in flight; the RT caller would keep its
            // previous copy (ZOH); nothing to assert per iteration.
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    running.store(false, std::memory_order_relaxed);

    writer.join();
    reader.join();

    EXPECT_GT(success_count.load(), 0);
    EXPECT_EQ(inconsistent_count.load(), 0) << "try_read accepted " << inconsistent_count.load() << " torn reads!";
}

// The bus loop holds the last command (ZOH) on the caller side: a persistent
// output outside the loop, updated only on a successful read, as
// JointDataPlane's command read does. A frozen writer leaves the previous
// torque in effect (not zeros or garbage), including the freshness stamp the
// staleness watchdog ages out.
TEST_F(SharedJointDataTest, TryReadCallerKeepsPreviousTorqueWhileWriterFrozen) {
    JointCommand<6> v1;
    for (size_t i = 0; i < 6; ++i) {
        v1.value[i] = 10.0 + static_cast<double>(i);
    }
    v1.update_ns = 100;
    shared_data_.command.write(v1);

    double persistent_torque[6] = {0};  // caller-side, outside the cycle loop
    int64_t last_update_ns = 0;

    JointCommand<6> tmp;
    if (shared_data_.command.try_read(tmp)) {
        for (size_t i = 0; i < 6; ++i) {
            persistent_torque[i] = tmp.value[i];
        }
        last_update_ns = tmp.update_ns;
    }
    ASSERT_EQ(last_update_ns, 100);

    // Next cycle: the command writer is preempted mid-write of the next command.
    SeqlockDataTestPeer::freeze_writer_mid_write(shared_data_.command);
    tmp = JointCommand<6>{};  // temp buffer content must not matter
    if (shared_data_.command.try_read(tmp)) {
        for (size_t i = 0; i < 6; ++i) {
            persistent_torque[i] = tmp.value[i];
        }
        last_update_ns = tmp.update_ns;
    }

    for (size_t i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(persistent_torque[i], 10.0 + static_cast<double>(i));
    }
    EXPECT_EQ(last_update_ns, 100);
}

// The payload is raw storage copied word by word; the layout (and so any
// process mixing builds) stays what `T data_` gave.
static_assert(sizeof(SeqlockData<JointActualValues<6>>) == 192);
static_assert(sizeof(SeqlockData<JointCommand<6>>) == 128);
static_assert(sizeof(SharedJointData<1>) == 256);
static_assert(sizeof(SharedJointData<6>) == 576);
static_assert(sizeof(SharedJointData<8>) == 768);

// A payload whose members have non-zero defaults.
struct Defaults {
    uint16_t interval_ms{1000};
    bool enabled{false};
};

TEST(SeqlockDataDefaults, FreshLockReadsTheTypesOwnDefaults) {
    SeqlockData<Defaults> lock;
    Defaults read_data{};
    read_data.interval_ms = 7;
    read_data.enabled = true;
    EXPECT_TRUE(lock.read(read_data));
    EXPECT_EQ(read_data.interval_ms, 1000);  // T's member default, not zero
    EXPECT_FALSE(read_data.enabled);
}

static_assert(alignof(SeqlockData<JointActualValues<6>>) == 64);
static_assert(alignof(SharedJointData<6>) == 64);

TEST(SeqlockDataLayout, PayloadSitsRightAfterTheEightByteSequence) {
    SeqlockData<JointCommand<6>> lock;
    JointCommand<6> value{};
    value.value[0] = 1.5;
    value.update_ns = 42;
    lock.write(value);
    EXPECT_EQ(std::memcmp(reinterpret_cast<const unsigned char*>(&lock) + 8, &value, sizeof(value)), 0);
}

// The byte paths under contention (the opt-in TSan target runs these): payloads
// whose size is not a multiple of 8 (38 B, and 260 B: 32 words
// and a 4-byte tail) and a reader copying a sub-range of what the writer publishes, as
// a byte-addressed region shared between processes allows. Every write fills all bytes with one counter
// value, so a copy spanning two writes shows up as unequal bytes.
template <std::size_t N>
struct Bytes {
    unsigned char b[N];
};

template <typename Write, typename TryRead>
void expect_no_torn_copy(Write write, TryRead try_read, std::size_t n) {
    std::atomic<bool> running{true};
    std::atomic<int> accepted{0};
    std::atomic<int> torn{0};
    std::thread writer([&] {
        unsigned char buf[260];
        for (unsigned counter = 0; running.load(std::memory_order_relaxed); ++counter) {
            std::memset(buf, static_cast<int>(counter & 0xFFU), sizeof(buf));
            write(buf);
            if (counter % 16 == 0) {  // quiet windows, so single-attempt reads also succeed under TSan
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
        }
    });
    std::thread reader([&] {
        unsigned char out[260];
        while (running.load(std::memory_order_relaxed)) {
            if (!try_read(out)) {
                continue;
            }
            ++accepted;
            for (std::size_t i = 1; i < n; ++i) {
                if (out[i] != out[0]) {
                    ++torn;
                    break;
                }
            }
        }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    running.store(false, std::memory_order_relaxed);
    writer.join();
    reader.join();
    EXPECT_GT(accepted.load(), 0);
    EXPECT_EQ(torn.load(), 0) << "accepted " << torn.load() << " torn copies of " << n << " B";
}

template <std::size_t N>
void expect_no_torn_slot_copy() {
    SeqlockData<Bytes<N>> lock;
    expect_no_torn_copy(
        [&](const unsigned char* src) {
            Bytes<N> v;
            std::memcpy(&v, src, N);
            lock.write(v);
        },
        [&](unsigned char* dst) {
            Bytes<N> v;
            if (!lock.try_read(v)) {
                return false;
            }
            std::memcpy(dst, &v, N);
            return true;
        },
        N);
}

TEST(SeqlockDataTail, ConcurrentTryReaderNeverAcceptsATorn38ByteSlot) {
    expect_no_torn_slot_copy<38>();
}

TEST(SeqlockDataTail, ConcurrentTryReaderNeverAcceptsATorn260ByteSlot) {
    expect_no_torn_slot_copy<260>();
}

TEST(SeqlockBytes, ConcurrentSubRangeReaderNeverAcceptsATornCopy) {
    alignas(8) unsigned char region[48]{};
    std::atomic<uint64_t> seq{0};
    constexpr std::size_t kOffset = 3;
    constexpr std::size_t kSize = 21;  // straddles three of the writer's would-be words
    expect_no_torn_copy(
        [&](const unsigned char* src) { seqlock::write(seq, region, src, sizeof(region), seqlock::Partition::BYTES); },
        [&](unsigned char* dst) {
            return seqlock::try_read(seq, dst, region + kOffset, kSize, seqlock::Partition::BYTES);
        },
        kSize);
}

TEST(SeqlockWords, UnalignedStartRoundTripsThroughTheHeadBytes) {
    alignas(8) unsigned char region[48]{};
    std::atomic<uint64_t> seq{0};
    unsigned char src[29];
    for (std::size_t i = 0; i < sizeof(src); ++i) {
        src[i] = static_cast<unsigned char>(i + 1);
    }
    seqlock::write(seq, region + 3, src, sizeof(src));  // 5 head bytes, 3 words, 0 tail
    unsigned char dst[sizeof(src)]{};
    ASSERT_TRUE(seqlock::try_read(seq, dst, region + 3, sizeof(dst)));
    EXPECT_EQ(std::memcmp(dst, src, sizeof(src)), 0);
    EXPECT_EQ(std::memcmp(region + 3, src, sizeof(src)), 0);
}

}  // namespace plem::core::threading
