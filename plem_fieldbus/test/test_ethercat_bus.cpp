// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <memory>
#include <type_traits>
#include <vector>
#include "plem/fieldbus/test/fake_ecrt.hpp"
#include "plem/fieldbus/test/single_axis_cst_layout.hpp"

#include "plem/fieldbus/ethercat/bus.hpp"

using namespace plem::fieldbus::ethercat;

// The bus owns its master reservation; a copy or move would release it twice.
static_assert(!std::is_copy_constructible_v<EtherCATBus> && !std::is_copy_assignable_v<EtherCATBus>);
static_assert(!std::is_move_constructible_v<EtherCATBus> && !std::is_move_assignable_v<EtherCATBus>);

// domain-registration capacity
// `EtherCATBus::configure_PDOs` hands one `ec_pdo_entry_reg_t[kMaxDomainRegs]`
// to every device in turn, then passes it to `ecrt_domain_reg_pdo_entry_list`,
// which reads a ZERO-TERMINATED list, so the last slot is the terminator's
// and the filler must stop one short of the capacity.
namespace {

/// The seam that reaches the protected filler: declares `count` entries and
/// registers them into the caller's array.
class CountingRegDevice : public DeviceBase {
  public:
    explicit CountingRegDevice(size_t count)
        : DeviceBase(plem::fieldbus::test::FakeEcrt::master(), 0, plem::fieldbus::test::kSyntheticVendorId,
                     plem::fieldbus::test::kSyntheticProductCode),
          entries_(count, PDOEntryDefinition{0x6041, 0x00, &offset_}) {}

    plem::core::Result<void> register_PDOs(ec_pdo_entry_reg_t* domain_regs, uint32_t& domain_index) override {
        return register_PDO_entries(domain_regs, domain_index, entries_);
    }

  private:
    uint32_t offset_ = 0;
    std::vector<PDOEntryDefinition> entries_;
};

}  // namespace

TEST(DomainRegistrationCapacity, FillsUpToTheTerminatorSlotAndRefusesPastIt) {
    ec_pdo_entry_reg_t domain_regs[kMaxDomainRegs] = {};

    uint32_t domain_index = 0;
    CountingRegDevice fits(kMaxDomainRegs - 1);
    const auto fitted = fits.register_PDOs(domain_regs, domain_index);
    EXPECT_TRUE(fitted.is_ok()) << fitted.message();
    EXPECT_EQ(domain_index, kMaxDomainRegs - 1) << "the last free slot must still be usable";

    uint32_t fresh_index = 0;
    CountingRegDevice one_too_many(kMaxDomainRegs);
    const auto overrun = one_too_many.register_PDOs(domain_regs, fresh_index);
    EXPECT_FALSE(overrun.is_ok()) << "taking the terminator's slot would overrun the list ecrt walks";
    EXPECT_NE(overrun.message().find("registration slots are free"), std::string::npos) << overrun.message();
    EXPECT_EQ(fresh_index, 0u) << "a refusal writes nothing — no partial registration";
}

TEST(DomainRegistrationCapacity, CountsTheSlotsALREADYTakenByEarlierDevices) {
    ec_pdo_entry_reg_t domain_regs[kMaxDomainRegs] = {};

    uint32_t domain_index = kMaxDomainRegs - 2;  // one free slot left before the terminator
    CountingRegDevice two_entries(2);
    EXPECT_FALSE(two_entries.register_PDOs(domain_regs, domain_index).is_ok());
    EXPECT_EQ(domain_index, kMaxDomainRegs - 2);
}
