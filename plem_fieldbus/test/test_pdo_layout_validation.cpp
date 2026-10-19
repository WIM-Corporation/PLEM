// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_pdo_layout_validation.cpp
/// @brief CiA 402 PDO layout semantic validation, pure, no fake master.
/// Checks CiA 402 role semantics on the declared pointer graph (the bus's
/// input). The assembly step checks fixed-array capacities; they are not
/// re-checked here. Structural defects are asserted by their structural
/// position, role defects by role, address and axis.

#include <gtest/gtest.h>

#include <cstdio>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "plem/fieldbus/ethercat/pdo_layout_validation.hpp"

namespace {

using plem::fieldbus::OperationMode;
using plem::fieldbus::ethercat::AxisPdoRequirements;
using plem::fieldbus::ethercat::PdoEntryMapping;
using plem::fieldbus::ethercat::PdoMapping;
using plem::fieldbus::ethercat::SlavePdoMapping;
using plem::fieldbus::ethercat::SyncManagerAssignment;
using plem::fieldbus::ethercat::validate_pdo_mapping;

// Owning layout builder (the production types are pointer views over
// constexpr storage; tests need per-case mutations without a static array
// per case)

struct Pdo {
    uint16_t index;
    std::vector<PdoEntryMapping> entries;
};

struct Sm {
    uint8_t sm_index;
    bool output;
    std::vector<Pdo> pdos;
    bool watchdog = false;
};

class Layout {
  public:
    explicit Layout(std::vector<Sm> sms) : sms_(std::move(sms)) {
        pdo_nodes_.reserve(sms_.size());
        for (const auto& sm : sms_) {
            std::vector<PdoMapping> nodes;
            for (const auto& pdo : sm.pdos) {
                nodes.push_back(PdoMapping{pdo.index, pdo.entries.empty() ? nullptr : pdo.entries.data(),
                                           static_cast<uint8_t>(pdo.entries.size())});
            }
            pdo_nodes_.push_back(std::move(nodes));
        }
        for (size_t i = 0; i < sms_.size(); ++i) {
            syncs_.push_back(SyncManagerAssignment{sms_[i].sm_index, sms_[i].output,
                                                   pdo_nodes_[i].empty() ? nullptr : pdo_nodes_[i].data(),
                                                   static_cast<uint8_t>(pdo_nodes_[i].size()), sms_[i].watchdog});
        }
    }

    [[nodiscard]] SlavePdoMapping view() const {
        return SlavePdoMapping{syncs_.data(), static_cast<uint8_t>(syncs_.size())};
    }

  private:
    std::vector<Sm> sms_;
    std::vector<std::vector<PdoMapping>> pdo_nodes_;
    std::vector<SyncManagerAssignment> syncs_;
};

AxisPdoRequirements axis(uint16_t od_offset, bool error_code = false) {
    AxisPdoRequirements req{};
    req.od_offset = od_offset;
    req.mode = OperationMode::CYCLIC_SYNC_TORQUE;
    if (error_code) {
        req.error_code_pdo = PdoEntryMapping{0x603F, 0, 16};
    }
    return req;
}

AxisPdoRequirements axis_with_error_code(uint16_t od_offset, PdoEntryMapping error_code) {
    AxisPdoRequirements req = axis(od_offset);
    req.error_code_pdo = error_code;
    return req;
}

std::string check(const Layout& layout, const std::vector<AxisPdoRequirements>& axes) {
    return validate_pdo_mapping(layout.view(), std::span<const AxisPdoRequirements>(axes.data(), axes.size()))
        .message();
}

bool mentions(const std::string& message, const std::string& needle) {
    return message.find(needle) != std::string::npos;
}

std::string hex(uint16_t index) {
    char buffer[16];
    snprintf(buffer, sizeof buffer, "0x%04x", index);
    return buffer;
}

// The CST-required set for one channel, byte-aligned in the simplest shape.
std::vector<PdoEntryMapping> rx_min(uint16_t ch) {
    return {{static_cast<uint16_t>(0x6040 + ch), 0, 16},
            {static_cast<uint16_t>(0x6060 + ch), 0, 8},
            {static_cast<uint16_t>(0x6071 + ch), 0, 16}};
}

std::vector<PdoEntryMapping> tx_min(uint16_t ch) {
    return {{static_cast<uint16_t>(0x6041 + ch), 0, 16},
            {static_cast<uint16_t>(0x6061 + ch), 0, 8},
            {static_cast<uint16_t>(0x6064 + ch), 0, 32},
            {static_cast<uint16_t>(0x606C + ch), 0, 32},
            {static_cast<uint16_t>(0x6077 + ch), 0, 16}};
}

std::vector<PdoEntryMapping> concat(std::vector<PdoEntryMapping> a, const std::vector<PdoEntryMapping>& b) {
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

/// Mailbox SM 0/1 without PDOs (the shape every CoE slave declares), one
/// RxPDO and one TxPDO with the required eight.
Layout minimal_single_axis() {
    return Layout(
        {{0, true, {}}, {1, false, {}}, {2, true, {{0x1600, rx_min(0)}}, true}, {3, false, {{0x1A00, tx_min(0)}}}});
}

/// Two channels in one RxPDO / one TxPDO, plus extra Tx entries after the
/// second channel's roles (for ErrorCode placement cases).
Layout two_channel(const std::vector<PdoEntryMapping>& extra_tx = {}) {
    return Layout({{2, true, {{0x1600, concat(rx_min(0), rx_min(0x800))}}, true},
                   {3, false, {{0x1A00, concat(concat(tx_min(0), tx_min(0x800)), extra_tx)}}}});
}

}  // namespace

// Acceptance

TEST(PdoLayoutValidation, MinimalCstSingleAxisIsAccepted) {
    // Target position/velocity are optional in CST; the required eight suffice.
    EXPECT_EQ(check(minimal_single_axis(), {axis(0)}), "");
}

TEST(PdoLayoutValidation, TwoChannelLayoutIsAcceptedInEitherAxisOrder) {
    // A shared PDO carrying both channels; the joint order on the slave may
    // be the reverse of the channel order ([0x800, 0]).
    EXPECT_EQ(check(two_channel({{0x603F, 0, 16}, {0x683F, 0, 16}}), {axis(0, true), axis(0x800, true)}), "");
    EXPECT_EQ(check(two_channel({{0x603F, 0, 16}, {0x683F, 0, 16}}), {axis(0x800, true), axis(0, true)}), "");
}

TEST(PdoLayoutValidation, CorrectlyMappedTargetVelocityIsAccepted) {
    // Optional, but when it is mapped with the right shape nothing rejects it.
    Layout layout({{2, true, {{0x1600, concat(rx_min(0), {{0x60FF, 0, 32}})}}}, {3, false, {{0x1A00, tx_min(0)}}}});
    EXPECT_EQ(check(layout, {axis(0)}), "");
}

TEST(PdoLayoutValidation, StructuralDefectsAreReportedEvenWithoutAxes) {
    Layout dup_sm({{2, true, {{0x1600, rx_min(0)}}}, {2, false, {{0x1A00, tx_min(0)}}}});
    EXPECT_EQ(check(minimal_single_axis(), {}), "");
    EXPECT_TRUE(mentions(check(dup_sm, {}), "sync manager 2"));
}

TEST(PdoLayoutValidation, EmptyLayoutWithAxesIsRejectedForMissingRoles) {
    // sync_count 0 as an input to the validator maps nothing, so every
    // required role is missing.
    const std::string message =
        validate_pdo_mapping(SlavePdoMapping{nullptr, 0}, std::vector<AxisPdoRequirements>{axis(0)}).message();
    EXPECT_TRUE(mentions(message, "required")) << message;
    EXPECT_TRUE(mentions(message, "0x6040")) << message;
}

// Structural rules (layout-wide)

TEST(PdoLayoutValidation, RejectsNullListsWithNonZeroCounts) {
    const SlavePdoMapping null_syncs{nullptr, 1};
    EXPECT_TRUE(mentions(validate_pdo_mapping(null_syncs, {}).message(), "sync_count"));

    static constexpr SyncManagerAssignment kNullPdos[] = {{2, true, nullptr, 1, false}};
    EXPECT_TRUE(mentions(validate_pdo_mapping(SlavePdoMapping{kNullPdos, 1}, {}).message(), "pdo_count"));

    static constexpr PdoMapping kNullEntries[] = {{0x1600, nullptr, 3}};
    static constexpr SyncManagerAssignment kSm[] = {{2, true, kNullEntries, 1, false}};
    EXPECT_TRUE(mentions(validate_pdo_mapping(SlavePdoMapping{kSm, 1}, {}).message(), "entry_count 3 with null"));
}

TEST(PdoLayoutValidation, RejectsSyncManagerIndexOutOfRange) {
    // The master exposes sync managers 0..15 (IgH EC_MAX_SYNC_MANAGERS).
    Layout layout({{2, true, {{0x1600, rx_min(0)}}}, {16, false, {{0x1A00, tx_min(0)}}}});
    EXPECT_TRUE(mentions(check(layout, {axis(0)}), "sm_index 16"));
}

TEST(PdoLayoutValidation, SyncManagersBeyondThreeAreAccepted) {
    // A CoE drive may place its process-data SMs at 4/5; the 0..3 shape of the
    // shipped models is not a core contract.
    Layout layout(
        {{0, true, {}}, {1, false, {}}, {4, true, {{0x1602, rx_min(0)}}, true}, {5, false, {{0x1A03, tx_min(0)}}}});
    EXPECT_EQ(check(layout, {axis(0)}), "");
}

TEST(PdoLayoutValidation, RejectsPdoIndexZero) {
    Layout layout({{2, true, {{0x0000, rx_min(0)}}}, {3, false, {{0x1A00, tx_min(0)}}}});
    EXPECT_TRUE(mentions(check(layout, {axis(0)}), "PDO index 0"));
}

TEST(PdoLayoutValidation, RejectsDuplicatePdoIndexAcrossTheLayout) {
    Layout layout({{2, true, {{0x1600, rx_min(0)}, {0x1600, rx_min(0x800)}}}, {3, false, {{0x1A00, tx_min(0)}}}});
    EXPECT_TRUE(mentions(check(layout, {axis(0)}), "PDO 0x1600 more than once"));
}

TEST(PdoLayoutValidation, RejectsPdoIndexOutsideItsSyncManagerDirection) {
    // CoE: RxPDO mapping 0x1600–0x17FF (outputs), TxPDO mapping 0x1A00–0x1BFF (inputs).
    Layout tx_in_output({{2, true, {{0x1A00, rx_min(0)}}}, {3, false, {{0x1A01, tx_min(0)}}}});
    EXPECT_TRUE(mentions(check(tx_in_output, {axis(0)}), "0x1a00"));
    Layout rx_in_input({{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1601, tx_min(0)}}}});
    EXPECT_TRUE(mentions(check(rx_in_input, {axis(0)}), "0x1601"));
}

TEST(PdoLayoutValidation, RejectsEntryLessPdoInDeclaredLayout) {
    // The master reads n_entries == 0 as "use the slave's default mapping";
    // entry_count 0 would let the master's default mapping in under a
    // declaration, and a declaration must list what it writes (a slave that
    // wants the drive's own mapping says so as a whole: scanned).
    // All roles are present and the empty PDO carries a NON-null entry
    // pointer, so the only rule that can reject it is the entry_count==0 one
    // (a null list would trip the structural null check instead).
    static constexpr PdoEntryMapping kRx[] = {{0x6040, 0, 16}, {0x6060, 0, 8}, {0x6071, 0, 16}};
    static constexpr PdoMapping kRxPdos[] = {{0x1600, kRx, 3}, {0x1601, kRx, 0}};
    static constexpr PdoEntryMapping kTx[] = {
        {0x6041, 0, 16}, {0x6061, 0, 8}, {0x6064, 0, 32}, {0x606C, 0, 32}, {0x6077, 0, 16}};
    static constexpr PdoMapping kTxPdos[] = {{0x1A00, kTx, 5}};
    static constexpr SyncManagerAssignment kSyncs[] = {{2, true, kRxPdos, 2, true}, {3, false, kTxPdos, 1, false}};
    const std::vector<AxisPdoRequirements> axes = {axis(0)};
    const std::string message = validate_pdo_mapping(SlavePdoMapping{kSyncs, 2}, axes).message();
    EXPECT_TRUE(mentions(message, "0x1601")) << message;
    EXPECT_TRUE(mentions(message, "entry_count 0")) << message;
}

TEST(PdoLayoutValidation, RejectsZeroWidthEntry) {
    auto tx = tx_min(0);
    tx.push_back({0x0000, 0, 0});  // a zero-width gap occupies nothing: a declaration typo
    Layout layout({{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1A00, tx}}}});
    EXPECT_TRUE(mentions(check(layout, {axis(0)}), "bit_length 0"));
}

// Semantic rules (per axis, index = base + od_offset)

TEST(PdoLayoutValidation, RejectsEveryMissingRequiredCstRole) {
    // Each of the eight the RT loop reads or writes every cycle: dropping any
    // one is rejected, and the message names the missing index. The expected
    // set is spelled out here, not derived from the implementation's table.
    const std::vector<PdoEntryMapping> rx = rx_min(0);
    const std::vector<PdoEntryMapping> tx = tx_min(0);
    ASSERT_EQ(rx.size() + tx.size(), 8u);
    for (size_t i = 0; i < rx.size(); ++i) {
        auto without = rx;
        without.erase(without.begin() + static_cast<std::ptrdiff_t>(i));
        Layout layout({{2, true, {{0x1600, without}}}, {3, false, {{0x1A00, tx}}}});
        const std::string message = check(layout, {axis(0)});
        EXPECT_TRUE(mentions(message, hex(rx[i].index))) << message;
        EXPECT_TRUE(mentions(message, "required")) << message;
    }
    for (size_t i = 0; i < tx.size(); ++i) {
        auto without = tx;
        without.erase(without.begin() + static_cast<std::ptrdiff_t>(i));
        Layout layout({{2, true, {{0x1600, rx}}}, {3, false, {{0x1A00, without}}}});
        const std::string message = check(layout, {axis(0)});
        EXPECT_TRUE(mentions(message, hex(tx[i].index))) << message;
        EXPECT_TRUE(mentions(message, "required")) << message;
    }
}

TEST(PdoLayoutValidation, RejectsRoleWidthMismatch) {
    auto tx = tx_min(0);
    tx[0] = {0x6041, 0, 32};  // statusword is read with a fixed 16-bit read
    Layout layout({{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1A00, tx}}}});
    const std::string message = check(layout, {axis(0)});
    EXPECT_TRUE(mentions(message, "0x6041")) << message;
    EXPECT_TRUE(mentions(message, "32 bits")) << message;
    EXPECT_TRUE(mentions(message, "16 bits")) << message;
}

TEST(PdoLayoutValidation, RejectsRoleSubindexMismatch) {
    auto rx = rx_min(0);
    rx[0] = {0x6040, 1, 16};
    Layout layout({{2, true, {{0x1600, rx}}}, {3, false, {{0x1A00, tx_min(0)}}}});
    const std::string message = check(layout, {axis(0)});
    EXPECT_TRUE(mentions(message, "0x6040")) << message;
    EXPECT_TRUE(mentions(message, "subindex 1")) << message;
}

TEST(PdoLayoutValidation, RejectsRoleInWrongDirectionSyncManager) {
    // Statusword declared inside the output sync manager's RxPDO: the drive
    // would never produce it there: a direction typo, not a missing role.
    auto rx = rx_min(0);
    rx.push_back({0x6041, 0, 16});
    auto tx = tx_min(0);
    tx.erase(tx.begin());
    Layout layout({{2, true, {{0x1600, rx}}}, {3, false, {{0x1A00, tx}}}});
    const std::string message = check(layout, {axis(0)});
    EXPECT_TRUE(mentions(message, "0x6041")) << message;
    EXPECT_TRUE(mentions(message, "direction")) << message;
}

TEST(PdoLayoutValidation, RejectsDuplicateRoleEntry) {
    // The same controlword mapped in two RxPDOs: registration would bind to
    // the first and the second would silently never be written.
    Layout layout({{2, true, {{0x1600, rx_min(0)}, {0x1601, {{0x6040, 0, 16}}}}}, {3, false, {{0x1A00, tx_min(0)}}}});
    const std::string message = check(layout, {axis(0)});
    EXPECT_TRUE(mentions(message, "0x6040")) << message;
    EXPECT_TRUE(mentions(message, "more than once")) << message;
}

TEST(PdoLayoutValidation, SecondAxisIsCheckedAtItsOwnChannel) {
    // A single-channel layout cannot host a second axis at 0x800: axis 1's
    // entries do not satisfy axis 2, and the message names the 0x68xx index.
    const std::string message = check(minimal_single_axis(), {axis(0), axis(0x800)});
    EXPECT_TRUE(mentions(message, "0x6840")) << message;
    EXPECT_TRUE(mentions(message, "axis 1")) << message;
    EXPECT_TRUE(mentions(message, "0x0800")) << message;
}

TEST(PdoLayoutValidation, OptionalRoleIsStillCheckedWhenMapped) {
    // Target position/velocity are optional in CST, but a mapped one is
    // written every cycle, so mapped means checked (width, direction).
    Layout narrow_tp({{2, true, {{0x1600, concat(rx_min(0), {{0x607A, 0, 16}})}}}, {3, false, {{0x1A00, tx_min(0)}}}});
    EXPECT_TRUE(mentions(check(narrow_tp, {axis(0)}), "0x607a"));
    Layout tv_in_input(
        {{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1A00, concat(tx_min(0), {{0x60FF, 0, 32}})}}}});
    const std::string message = check(tv_in_input, {axis(0)});
    EXPECT_TRUE(mentions(message, "0x60ff")) << message;
    EXPECT_TRUE(mentions(message, "direction")) << message;
}

// Byte alignment: the registration passes no bit position

TEST(PdoLayoutValidation, RejectsRoleEntryOffByteBoundary) {
    // A 4-bit gap before the statusword shifts every following role off its
    // byte; the master would refuse the registration after hardware contact.
    auto tx = tx_min(0);
    tx.insert(tx.begin(), {0x0000, 0, 4});
    Layout layout({{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1A00, tx}}}});
    const std::string message = check(layout, {axis(0)});
    EXPECT_TRUE(mentions(message, "0x6041")) << message;
    EXPECT_TRUE(mentions(message, "byte")) << message;
}

TEST(PdoLayoutValidation, BitOffsetsAccumulateAcrossPdosOfOneSyncManager) {
    // IgH packs the PDOs of one sync manager bit-contiguously: a 4-bit tail in
    // the first TxPDO shifts the second TxPDO's first role off its byte.
    const std::vector<PdoEntryMapping> rest = {{0x6061, 0, 8}, {0x6064, 0, 32}, {0x606C, 0, 32}, {0x6077, 0, 16}};
    Layout shifted(
        {{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1A00, {{0x6041, 0, 16}, {0x0000, 0, 4}}}, {0x1A01, rest}}}});
    const std::string message = check(shifted, {axis(0)});
    EXPECT_TRUE(mentions(message, "0x6061")) << message;
    EXPECT_TRUE(mentions(message, "byte")) << message;
    // Padding the tail to a byte boundary makes the same shape legal.
    Layout padded(
        {{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1A00, {{0x6041, 0, 16}, {0x0000, 0, 8}}}, {0x1A01, rest}}}});
    EXPECT_EQ(check(padded, {axis(0)}), "");
}

TEST(PdoLayoutValidation, SubByteGapsThatReturnToAByteBoundaryAreAccepted) {
    // Only the role start bits must be byte-aligned; gaps and vendor entries
    // may be any width as long as the roles land on bytes.
    auto tx = tx_min(0);
    tx.insert(tx.begin(), {0x0000, 0, 4});
    tx.insert(tx.begin(), {0x0000, 0, 4});
    Layout layout({{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1A00, tx}}}});
    EXPECT_EQ(check(layout, {axis(0)}), "");
}

TEST(PdoLayoutValidation, VendorEntriesCountInTheBitAccumulation) {
    // A 4-bit vendor object ahead of the roles shifts them exactly like a gap
    // does: alone it misaligns (rejected); with a 4-bit gap after it the
    // roles are back on a byte (accepted). Skipping vendor entries in the
    // accumulation would invert both outcomes.
    auto vendor_only = tx_min(0);
    vendor_only.insert(vendor_only.begin(), {0x3000, 0, 4});
    Layout misaligned({{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1A00, vendor_only}}}});
    const std::string message = check(misaligned, {axis(0)});
    EXPECT_TRUE(mentions(message, "0x6041")) << message;
    EXPECT_TRUE(mentions(message, "byte")) << message;

    auto vendor_plus_gap = tx_min(0);
    vendor_plus_gap.insert(vendor_plus_gap.begin(), {0x0000, 0, 4});
    vendor_plus_gap.insert(vendor_plus_gap.begin(), {0x3000, 0, 4});
    Layout realigned({{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1A00, vendor_plus_gap}}}});
    EXPECT_EQ(check(realigned, {axis(0)}), "");
}

TEST(PdoLayoutValidation, BitOffsetsRestartAtEachSyncManager) {
    // The output SM ends mid-byte; the input SM's first role still starts at
    // bit 0 of its image: accumulation never crosses a sync manager.
    Layout layout({{2, true, {{0x1600, concat(rx_min(0), {{0x0000, 0, 4}})}}}, {3, false, {{0x1A00, tx_min(0)}}}});
    EXPECT_EQ(check(layout, {axis(0)}), "");
}

// ErrorCode role (declared index, fixed 16-bit decode)

TEST(PdoLayoutValidation, ErrorCodeRoleDeclaredButUnmappedIsRejected) {
    const std::string message = check(minimal_single_axis(), {axis(0, true)});
    EXPECT_TRUE(mentions(message, "0x603f")) << message;
    EXPECT_TRUE(mentions(message, "required")) << message;
}

TEST(PdoLayoutValidation, ErrorCodeRoleMustMatchDeclaredSubindexAndWidth) {
    // The role is keyed by the declared (index, subindex): an entry at another
    // subindex is a different object, so the declared one is simply unmapped.
    auto tx = tx_min(0);
    tx.push_back({0x603F, 1, 16});
    Layout sub({{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1A00, tx}}}});
    const std::string sub_message = check(sub, {axis(0, true)});
    EXPECT_TRUE(mentions(sub_message, "0x603f:00")) << sub_message;
    EXPECT_TRUE(mentions(sub_message, "required")) << sub_message;

    tx.back() = {0x603F, 0, 32};
    Layout width({{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1A00, tx}}}});
    EXPECT_TRUE(mentions(check(width, {axis(0, true)}), "32 bits"));

    tx.back() = {0x603F, 0, 16};
    Layout ok({{2, true, {{0x1600, rx_min(0)}}}, {3, false, {{0x1A00, tx}}}});
    EXPECT_EQ(check(ok, {axis(0, true)}), "");
}

TEST(PdoLayoutValidation, ErrorCodeRoleUsesTheDeclaredBaseIndexPlusChannel) {
    // A vendor may place the error code elsewhere than 0x603F; the declared
    // base is what the axis registers, with the channel stride applied once.
    Layout layout(
        {{2, true, {{0x1600, rx_min(0x800)}}}, {3, false, {{0x1A00, concat(tx_min(0x800), {{0x2801, 0, 16}})}}}});
    EXPECT_EQ(check(layout, {axis_with_error_code(0x800, {0x2001, 0, 16})}), "");
}

TEST(PdoLayoutValidation, ErrorCodeMappingIsKeyedByIndexAndSubindex) {
    // 0x2000:1 and 0x2000:2 are different objects: two axes may declare error
    // codes that resolve to the same index at different subindices, and each
    // finds exactly its own mapping (no false "mapped twice").
    Layout layout = two_channel({{0x2000, 1, 16}, {0x2000, 2, 16}});
    EXPECT_EQ(check(layout, {axis_with_error_code(0, {0x2000, 1, 16}), axis_with_error_code(0x800, {0x1800, 2, 16})}),
              "");
}

// Address collisions between mapped requirements: one entry, one consumer

TEST(PdoLayoutValidation, ErrorCodeAliasingTheStatuswordIsRejected) {
    // Declaring the error code at the statusword address passes every
    // per-role check (mapped once, 16 bits, input, aligned) and would feed
    // the statusword into the fault diagnostic. Rejected as a collision.
    const std::string message = check(minimal_single_axis(), {axis_with_error_code(0, {0x6041, 0, 16})});
    EXPECT_TRUE(mentions(message, "0x6041")) << message;
    EXPECT_TRUE(mentions(message, "Statusword")) << message;
    EXPECT_TRUE(mentions(message, "Error code")) << message;
}

TEST(PdoLayoutValidation, TwoAxesErrorCodesResolvingToOneAddressAreRejected) {
    // axis 0 at offset 0 declares base 0x2801; axis 1 at offset 0x800 declares
    // base 0x2001; both resolve to 0x2801:00.
    Layout layout = two_channel({{0x2801, 0, 16}});
    const std::string message =
        check(layout, {axis_with_error_code(0, {0x2801, 0, 16}), axis_with_error_code(0x800, {0x2001, 0, 16})});
    EXPECT_TRUE(mentions(message, "0x2801")) << message;
    EXPECT_TRUE(mentions(message, "axis 0")) << message;
    EXPECT_TRUE(mentions(message, "axis 1")) << message;
}

TEST(PdoLayoutValidation, TwoAxesOnTheSameChannelAreRejected) {
    // Self-contained: even without the bus's (slave, od_offset) uniqueness
    // check, two axes at one channel collide on every standard role.
    EXPECT_TRUE(mentions(check(minimal_single_axis(), {axis(0), axis(0)}), "0x6040"));
}

// Mode input: the required set is mode-owned; unknown modes have no table

TEST(PdoLayoutValidation, UnsupportedModeIsRejectedNotGuessed) {
    AxisPdoRequirements req = axis(0);
    req.mode = OperationMode::NO_MODE;
    const std::string message = check(minimal_single_axis(), {req});
    EXPECT_TRUE(mentions(message, "mode")) << message;
}

// Each mode requires the common roles plus its own target; the other targets
// are optional. The minimal layout maps only 0x6071.
TEST(PdoLayoutValidation, EachModeRequiresItsOwnTargetOnly) {
    EXPECT_EQ(check(minimal_single_axis(), {axis(0)}), "");
    AxisPdoRequirements csp = axis(0);
    csp.mode = OperationMode::CYCLIC_SYNC_POSITION;
    EXPECT_TRUE(mentions(check(minimal_single_axis(), {csp}), "0x607a")) << check(minimal_single_axis(), {csp});
    AxisPdoRequirements csv = axis(0);
    csv.mode = OperationMode::CYCLIC_SYNC_VELOCITY;
    EXPECT_TRUE(mentions(check(minimal_single_axis(), {csv}), "0x60ff")) << check(minimal_single_axis(), {csv});

    const Layout csp_only({{0, true, {}},
                           {1, false, {}},
                           {2, true, {{0x1600, {{0x6040, 0, 16}, {0x6060, 0, 8}, {0x607A, 0, 32}}}}, true},
                           {3, false, {{0x1A00, tx_min(0)}}}});
    EXPECT_EQ(check(csp_only, {csp}), "");
    EXPECT_TRUE(mentions(check(csp_only, {axis(0)}), "0x6071"));
}
