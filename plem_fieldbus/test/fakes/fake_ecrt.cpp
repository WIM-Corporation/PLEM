// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/fieldbus/test/fake_ecrt.hpp"
#include "plem/fieldbus/test/single_axis_cst_layout.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <set>
#include <tuple>
#include <utility>

#include <ecrt.h>
#include <fmt/format.h>

// Fake state (file-scope so the extern-C definitions below can reach it).
namespace {

std::vector<std::string> g_calls;
uint32_t g_sync_monitor_drift_ns = 0;
std::map<uint16_t, uint32_t> g_slave_serials;
// Identity get_slave reports: a set_slave_identity knob, else the first slave
// config's per position, else the synthetic drive.
std::map<uint16_t, std::pair<uint32_t, uint32_t>> g_configured_identity;
std::map<uint16_t, std::pair<uint32_t, uint32_t>> g_slave_identity;
std::map<uint16_t, std::string> g_sdo_upload_strings;
/// set_sdo_upload_entry knob, keyed by {position, index, subindex}.
struct UploadKnob {
    int ret;
    std::vector<uint8_t> bytes;
    uint32_t abort_code;
};
std::map<std::array<unsigned int, 3>, UploadKnob> g_sdo_upload_entries;
/// set_sdo_upload_transient_failures knob: -EIO calls left per key.
std::map<std::array<unsigned int, 3>, int> g_sdo_upload_transient_failures;

/// set_slave_scan knob: what the scan queries answer for a slave.
struct ScanKnob {
    uint32_t revision;
    const plem::fieldbus::ethercat::SlavePdoMapping* layout;
};
std::map<uint16_t, ScanKnob> g_scans;
std::set<uint16_t> g_scan_query_failures;
std::set<uint16_t> g_slave_config_failures;  ///< set_slave_config_failure knob
/// The effective (index, subindex) mapping of every knob slave: the scan
/// layout until ecrt_slave_config_pdos() replaces it. Registration of an entry
/// outside it fails with -ENOENT, like the real master's offset resolution.
std::map<uint16_t, std::set<std::pair<uint16_t, uint8_t>>> g_effective_mapping;

constexpr size_t kMaxSlaves = plem::fieldbus::test::FakeEcrt::kMaxSlaves;

/// A position past the fake's ring is a test bug, not a master refusal: say so and stop.
[[noreturn]] void past_the_fake_ring(const char* call, long position) {
    std::fprintf(stderr, "FakeEcrt::%s: slave position %ld is outside the fake's %zu-slave ring (0..%zu)\n", call,
                 position, kMaxSlaves, kMaxSlaves - 1);
    std::abort();
}

// ecrt.h only forward-declares the handle types, so distinct byte slots
// reinterpreted as the opaque pointers are sufficient identities for the fake.
char g_master_storage;
char g_domain_storage;
char g_slave_config_storage[kMaxSlaves];
uint8_t g_domain_pd[4096];
std::atomic<unsigned int> g_domain_wc_state{EC_WC_COMPLETE};
std::vector<std::array<unsigned int, 3>> g_registered_entries;
std::set<std::array<unsigned int, 3>> g_unmapped_entries;
/// Next free fake offset; single-entry registrations continue the grid the
/// list registration started (entry #n at byte n * 8).
unsigned int g_next_offset = 0;
std::map<uint16_t, const plem::fieldbus::ethercat::SlavePdoMapping*> g_domain_layouts;
std::map<std::array<unsigned int, 2>, unsigned int> g_sm_base;  // {pos, sm} → domain offset of the SM image
std::vector<std::array<unsigned int, 3>> g_domain_sms;
std::map<std::array<unsigned int, 3>, unsigned int> g_entry_offsets;
std::map<std::array<unsigned int, 2>, int> g_config_sdo_failures;  // {index, subindex} → ret
bool g_request_master_fails = false;
std::set<int> g_config_pdos_failures;
int g_activate_ret = 0;
bool g_create_domain_fails = false;
bool g_domain_data_fails = false;
std::set<uint16_t> g_absent_slaves;
std::map<uint16_t, int> g_dc_failures;  // position → ret
int g_reference_clock_ret = 0;
/// set_slave_config_state knob per position (default: OP, operational).
struct ConfigStateKnob {
    uint8_t al = 8;      ///< AL state the query reports (8 = OP, 0 = detached)
    bool force = false;  ///< force_config pending
    bool fails = false;  ///< the query itself fails
};
std::array<ConfigStateKnob, kMaxSlaves> g_config_states{};

/// A new domain starts empty (as in the master): the allocation state lives
/// with the domain, cleared here, at ecrt_master_create_domain and reset().
void clear_domain_allocation() {
    g_next_offset = 0;
    g_sm_base.clear();
    g_domain_sms.clear();
    g_entry_offsets.clear();
}

/// Log label for an assign_offset() error.
const char* error_name(int error) {
    return error == -EINVAL ? "EINVAL" : "ENOENT";
}

/// Offset for one registration: the SM-image model for a layout slave, else
/// the 8-byte grid. Returns -ENOENT for an entry outside the layout.
int assign_offset(unsigned int position, unsigned int index, unsigned int subindex) {
    const auto layout = g_domain_layouts.find(static_cast<uint16_t>(position));
    if (layout == g_domain_layouts.end()) {
        const unsigned int offset = g_next_offset;
        g_next_offset += 8;
        g_entry_offsets[{position, index, subindex}] = offset;
        return static_cast<int>(offset);
    }
    for (uint8_t s = 0; s < layout->second->sync_count; ++s) {
        const auto& sm = layout->second->syncs[s];
        unsigned int bits = 0;
        int found_bits = -1;
        for (uint8_t p = 0; p < sm.pdo_count; ++p) {
            for (uint8_t e = 0; e < sm.pdos[p].entry_count; ++e) {
                const auto& entry = sm.pdos[p].entries[e];
                if (found_bits < 0 && entry.index == index && entry.subindex == subindex) {
                    found_bits = static_cast<int>(bits);
                }
                bits += entry.bit_length;
            }
        }
        if (found_bits < 0) {
            continue;
        }
        if (found_bits % 8 != 0) {
            return -EINVAL;  // byte-aligned entries only (no bit_position model)
        }
        const auto key = std::array<unsigned int, 2>{position, sm.sm_index};
        auto base = g_sm_base.find(key);
        if (base == g_sm_base.end()) {  // first entry of this SM: append its whole image
            base = g_sm_base.emplace(key, g_next_offset).first;
            g_domain_sms.push_back({position, sm.sm_index, sm.output ? 1u : 0u});
            g_next_offset += (bits + 7) / 8;
        }
        const unsigned int offset = base->second + static_cast<unsigned int>(found_bits) / 8;
        g_entry_offsets[{position, index, subindex}] = offset;
        return static_cast<int>(offset);
    }
    return -ENOENT;
}

std::function<void()>& send_observer() {
    static std::function<void()> observer;
    return observer;
}

void record(std::string call) {
    g_calls.push_back(std::move(call));
}

int position_of(const ec_slave_config_t* sc) {
    const char* base = &g_slave_config_storage[0];
    const char* p = reinterpret_cast<const char*>(sc);
    if (p < base || p >= base + kMaxSlaves) {
        return -1;
    }
    return static_cast<int>(p - base);
}

}  // namespace

namespace plem::fieldbus::test {

std::vector<std::string>& FakeEcrt::calls() {
    return g_calls;
}

void FakeEcrt::reset() {
    g_calls.clear();
    g_sync_monitor_drift_ns = 0;
    g_slave_serials.clear();
    g_configured_identity.clear();
    g_slave_identity.clear();
    g_sdo_upload_strings.clear();
    g_sdo_upload_entries.clear();
    g_sdo_upload_transient_failures.clear();
    g_scans.clear();
    g_scan_query_failures.clear();
    g_slave_config_failures.clear();
    g_effective_mapping.clear();
    // The domain buffer is shared process-global state; a stale frame from a
    // previous test must not leak into the next one (hermetic SetUp).
    std::memset(g_domain_pd, 0, sizeof(g_domain_pd));
    send_observer() = nullptr;
    g_domain_wc_state = EC_WC_COMPLETE;
    g_registered_entries.clear();
    g_unmapped_entries.clear();
    g_domain_layouts.clear();
    clear_domain_allocation();
    g_config_sdo_failures.clear();
    g_config_states.fill({});
    g_request_master_fails = false;
    g_activate_ret = 0;
    g_config_pdos_failures.clear();
    g_create_domain_fails = false;
    g_domain_data_fails = false;
    g_absent_slaves.clear();
    g_dc_failures.clear();
    g_reference_clock_ret = 0;
}

void FakeEcrt::set_config_pdos_failure(unsigned int position) {
    g_config_pdos_failures.insert(static_cast<int>(position));
}

void FakeEcrt::set_request_master_failure() {
    g_request_master_fails = true;
}

void FakeEcrt::set_activate_failure(int ret) {
    g_activate_ret = ret;
}

void FakeEcrt::set_create_domain_failure() {
    g_create_domain_fails = true;
}

void FakeEcrt::set_domain_data_failure() {
    g_domain_data_fails = true;
}

void FakeEcrt::set_absent_slave(unsigned int position) {
    g_absent_slaves.insert(static_cast<uint16_t>(position));
}

void FakeEcrt::set_dc_failure(unsigned int position, int ret) {
    g_dc_failures[static_cast<uint16_t>(position)] = ret;
}

void FakeEcrt::set_reference_clock_failure(int ret) {
    g_reference_clock_ret = ret;
}

void FakeEcrt::set_config_sdo_failure(unsigned int index, unsigned int subindex, int ret) {
    g_config_sdo_failures[{index, subindex}] = ret;
}

void FakeEcrt::set_slave_config_state(unsigned int position, unsigned int al_state, bool force_config) {
    if (position >= kMaxSlaves) {
        past_the_fake_ring("set_slave_config_state", position);
    }
    g_config_states[position] = {.al = static_cast<uint8_t>(al_state), .force = force_config};
}

void FakeEcrt::set_slave_config_state_failure(unsigned int position) {
    if (position >= kMaxSlaves) {
        past_the_fake_ring("set_slave_config_state_failure", position);
    }
    g_config_states[position].fails = true;
}

void FakeEcrt::set_slave_domain_layout(unsigned int position, const plem::fieldbus::ethercat::SlavePdoMapping& layout) {
    g_domain_layouts[static_cast<uint16_t>(position)] = &layout;
}

int FakeEcrt::domain_offset_of(unsigned int position, unsigned int index, unsigned int subindex) {
    const auto it = g_entry_offsets.find({position, index, subindex});
    return it == g_entry_offsets.end() ? -1 : static_cast<int>(it->second);
}

const std::vector<std::array<unsigned int, 3>>& FakeEcrt::domain_sync_managers() {
    return g_domain_sms;
}

void FakeEcrt::set_unmapped_entry(unsigned int position, unsigned int index, unsigned int subindex) {
    g_unmapped_entries.insert({position, index, subindex});
}

void FakeEcrt::set_domain_wc_state(unsigned int wc_state) {
    g_domain_wc_state = wc_state;
}

const std::vector<std::array<unsigned int, 3>>& FakeEcrt::registered_entries() {
    return g_registered_entries;
}

void FakeEcrt::set_send_observer(std::function<void()> observer) {
    send_observer() = std::move(observer);
}

ec_master_t* FakeEcrt::master() {
    return reinterpret_cast<ec_master_t*>(&g_master_storage);
}

void FakeEcrt::set_sync_monitor_drift_ns(unsigned int value) {
    g_sync_monitor_drift_ns = value;
}

void FakeEcrt::set_slave_serial(unsigned int position, unsigned int serial) {
    g_slave_serials[static_cast<uint16_t>(position)] = serial;
}

void FakeEcrt::set_slave_identity(unsigned int position, unsigned int vendor_id, unsigned int product_code) {
    g_slave_identity[static_cast<uint16_t>(position)] = {vendor_id, product_code};
}

void FakeEcrt::set_sdo_upload_string(unsigned int index, std::string bytes) {
    g_sdo_upload_strings[static_cast<uint16_t>(index)] = std::move(bytes);
}

void FakeEcrt::set_sdo_upload_entry(unsigned int position, unsigned int index, unsigned int subindex, int ret,
                                    std::vector<uint8_t> bytes, uint32_t abort_code) {
    g_sdo_upload_entries[{position, index, subindex}] = UploadKnob{ret, std::move(bytes), abort_code};
}

void FakeEcrt::set_sdo_upload_transient_failures(unsigned int position, unsigned int index, unsigned int subindex,
                                                 int count) {
    g_sdo_upload_transient_failures[{position, index, subindex}] = count;
}

void FakeEcrt::set_slave_scan(unsigned int position, unsigned int revision,
                              const plem::fieldbus::ethercat::SlavePdoMapping& layout) {
    const auto pos = static_cast<uint16_t>(position);
    g_scans[pos] = ScanKnob{revision, &layout};
    auto& mapping = g_effective_mapping[pos];
    mapping.clear();
    for (uint8_t s = 0; s < layout.sync_count; ++s) {
        const auto& sm = layout.syncs[s];
        for (uint8_t p = 0; p < sm.pdo_count; ++p) {
            for (uint8_t e = 0; e < sm.pdos[p].entry_count; ++e) {
                mapping.emplace(sm.pdos[p].entries[e].index, sm.pdos[p].entries[e].subindex);
            }
        }
    }
}

void FakeEcrt::set_slave_scan_query_failure(unsigned int position) {
    g_scan_query_failures.insert(static_cast<uint16_t>(position));
}

void FakeEcrt::set_slave_config_failure(unsigned int position) {
    g_slave_config_failures.insert(static_cast<uint16_t>(position));
}

}  // namespace plem::fieldbus::test

// ecrt C API fakes (link seam; signatures mirror /opt/etherlab/include/ecrt.h)

ec_master_t* ecrt_request_master(unsigned int master_index) {
    record(fmt::format("request_master(index={})", master_index));
    return g_request_master_fails ? nullptr : reinterpret_cast<ec_master_t*>(&g_master_storage);
}

void ecrt_release_master(ec_master_t* /*master*/) {
    record("release_master");
}

ec_domain_t* ecrt_master_create_domain(ec_master_t* /*master*/) {
    record("create_domain");
    if (g_create_domain_fails) {
        return nullptr;
    }
    clear_domain_allocation();
    return reinterpret_cast<ec_domain_t*>(&g_domain_storage);
}

int ecrt_master(ec_master_t* /*master*/, ec_master_info_t* master_info) {
    std::memset(master_info, 0, sizeof(*master_info));
    master_info->slave_count = 0;
    master_info->link_up = 1;
    record("master_info");
    return 0;
}

ec_slave_config_t* ecrt_master_slave_config(ec_master_t* /*master*/, uint16_t alias, uint16_t position,
                                            uint32_t vendor_id, uint32_t product_code) {
    record(fmt::format("slave_config(alias={},pos={},vendor=0x{:x},product=0x{:x})", alias, position, vendor_id,
                       product_code));
    if (position >= kMaxSlaves) {
        past_the_fake_ring("ecrt_master_slave_config", position);
    }
    if (g_slave_config_failures.count(position) != 0) {
        return nullptr;
    }
    g_configured_identity.emplace(position, std::make_pair(vendor_id, product_code));
    return reinterpret_cast<ec_slave_config_t*>(&g_slave_config_storage[position]);
}

int ecrt_master_get_slave(ec_master_t* /*master*/, uint16_t slave_position, ec_slave_info_t* slave_info) {
    std::memset(slave_info, 0, sizeof(*slave_info));
    if (g_absent_slaves.count(slave_position) != 0) {
        return -EINVAL;
    }
    slave_info->position = slave_position;
    if (const auto knob = g_slave_identity.find(slave_position); knob != g_slave_identity.end()) {
        std::tie(slave_info->vendor_id, slave_info->product_code) = knob->second;
    } else if (const auto configured = g_configured_identity.find(slave_position);
               configured != g_configured_identity.end()) {
        std::tie(slave_info->vendor_id, slave_info->product_code) = configured->second;
    } else {
        // Not configured yet (a scan reads it first): the synthetic drive.
        slave_info->vendor_id = plem::fieldbus::test::kSyntheticVendorId;
        slave_info->product_code = plem::fieldbus::test::kSyntheticProductCode;
    }
    if (const auto serial = g_slave_serials.find(slave_position); serial != g_slave_serials.end()) {
        slave_info->serial_number = serial->second;
    }
    if (const auto scan = g_scans.find(slave_position); scan != g_scans.end()) {
        slave_info->revision_number = scan->second.revision;
        slave_info->sync_count = scan->second.layout->sync_count;
    }
    return 0;
}

// Scan queries (set_slave_scan knob): mirror ecrt_master_get_sync_manager /
// get_pdo / get_pdo_entry: each fills its output struct from the knob layout
// and records the call; a slave without a knob (or with the failure knob)
// answers non-zero like a master that cannot serve the query.

namespace {
const plem::fieldbus::ethercat::SlavePdoMapping* scan_layout_of(uint16_t position) {
    if (g_scan_query_failures.count(position) != 0) {
        return nullptr;
    }
    const auto scan = g_scans.find(position);
    return scan == g_scans.end() ? nullptr : scan->second.layout;
}
}  // namespace

int ecrt_master_get_sync_manager(ec_master_t* /*master*/, uint16_t slave_position, uint8_t sync_index,
                                 ec_sync_info_t* sync) {
    record(fmt::format("get_sync_manager(pos={},sm={})", slave_position, sync_index));
    const auto* layout = scan_layout_of(slave_position);
    if (layout == nullptr || sync_index >= layout->sync_count) {
        return -EIO;
    }
    const auto& sm = layout->syncs[sync_index];
    *sync = ec_sync_info_t{sm.sm_index, sm.output ? EC_DIR_OUTPUT : EC_DIR_INPUT, sm.pdo_count, nullptr,
                           sm.watchdog ? EC_WD_ENABLE : EC_WD_DISABLE};
    return 0;
}

int ecrt_master_get_pdo(ec_master_t* /*master*/, uint16_t slave_position, uint8_t sync_index, uint16_t pos,
                        ec_pdo_info_t* pdo) {
    record(fmt::format("get_pdo(pos={},sm={},pdo={})", slave_position, sync_index, pos));
    const auto* layout = scan_layout_of(slave_position);
    if (layout == nullptr || sync_index >= layout->sync_count || pos >= layout->syncs[sync_index].pdo_count) {
        return -EIO;
    }
    const auto& mapped = layout->syncs[sync_index].pdos[pos];
    *pdo = ec_pdo_info_t{mapped.index, mapped.entry_count, nullptr};
    return 0;
}

int ecrt_master_get_pdo_entry(ec_master_t* /*master*/, uint16_t slave_position, uint8_t sync_index, uint16_t pdo_pos,
                              uint16_t entry_pos, ec_pdo_entry_info_t* entry) {
    record(fmt::format("get_pdo_entry(pos={},sm={},pdo={},entry={})", slave_position, sync_index, pdo_pos, entry_pos));
    const auto* layout = scan_layout_of(slave_position);
    if (layout == nullptr || sync_index >= layout->sync_count || pdo_pos >= layout->syncs[sync_index].pdo_count ||
        entry_pos >= layout->syncs[sync_index].pdos[pdo_pos].entry_count) {
        return -EIO;
    }
    const auto& e = layout->syncs[sync_index].pdos[pdo_pos].entries[entry_pos];
    *entry = ec_pdo_entry_info_t{e.index, e.subindex, e.bit_length};
    return 0;
}

int ecrt_slave_config_pdos(ec_slave_config_t* sc, unsigned int /*n_syncs*/, const ec_sync_info_t* syncs) {
    // Record the applied assignment compactly: per SM its index/dir/watchdog
    // and the mapped PDO indices (enough for once-per-slave + shape pins).
    std::string summary;
    for (const ec_sync_info_t* sync = syncs; sync != nullptr && sync->index != 0xff; ++sync) {
        summary += fmt::format("[sm{}:{}{}", sync->index, sync->dir == EC_DIR_OUTPUT ? "out" : "in",
                               sync->watchdog_mode == EC_WD_DISABLE ? ",wd_off" : ",wd_on");
        for (unsigned int j = 0; j < sync->n_pdos && sync->pdos != nullptr; ++j) {
            summary += fmt::format(",0x{:x}({})", sync->pdos[j].index, sync->pdos[j].n_entries);
        }
        summary += "]";
    }
    record(fmt::format("config_pdos(pos={}){}", position_of(sc), summary));
    if (g_config_pdos_failures.count(position_of(sc)) != 0) {
        return -1;
    }
    // A knob slave's effective mapping becomes what was just configured (the
    // real master resolves later registrations against the configured
    // assignment, not the scanned one).
    if (const int pos = position_of(sc); pos >= 0) {
        if (const auto effective = g_effective_mapping.find(static_cast<uint16_t>(pos));
            effective != g_effective_mapping.end()) {
            effective->second.clear();
            for (const ec_sync_info_t* sync = syncs; sync != nullptr && sync->index != 0xff; ++sync) {
                for (unsigned int j = 0; j < sync->n_pdos && sync->pdos != nullptr; ++j) {
                    const ec_pdo_info_t& pdo = sync->pdos[j];
                    for (unsigned int k = 0; k < pdo.n_entries && pdo.entries != nullptr; ++k) {
                        effective->second.emplace(pdo.entries[k].index, pdo.entries[k].subindex);
                    }
                }
            }
        }
    }
    return 0;
}

namespace {
int config_sdo_result(uint16_t index, uint8_t subindex) {
    const auto failure = g_config_sdo_failures.find({index, subindex});
    return failure == g_config_sdo_failures.end() ? 0 : failure->second;
}
}  // namespace

int ecrt_slave_config_sdo8(ec_slave_config_t* sc, uint16_t sdo_index, uint8_t sdo_subindex, uint8_t value) {
    record(fmt::format("slave_config_sdo8(pos={},idx=0x{:x},sub=0x{:x},val=0x{:x})", position_of(sc), sdo_index,
                       sdo_subindex, value));
    return config_sdo_result(sdo_index, sdo_subindex);
}

int ecrt_slave_config_sdo16(ec_slave_config_t* sc, uint16_t sdo_index, uint8_t sdo_subindex, uint16_t value) {
    record(fmt::format("slave_config_sdo16(pos={},idx=0x{:x},sub=0x{:x},val=0x{:x})", position_of(sc), sdo_index,
                       sdo_subindex, value));
    return config_sdo_result(sdo_index, sdo_subindex);
}

int ecrt_slave_config_sdo32(ec_slave_config_t* sc, uint16_t sdo_index, uint8_t sdo_subindex, uint32_t value) {
    record(fmt::format("slave_config_sdo32(pos={},idx=0x{:x},sub=0x{:x},val=0x{:x})", position_of(sc), sdo_index,
                       sdo_subindex, value));
    return config_sdo_result(sdo_index, sdo_subindex);
}

int ecrt_domain_reg_pdo_entry_list(ec_domain_t* /*domain*/, const ec_pdo_entry_reg_t* pdo_entry_regs) {
    // Assign monotonically increasing offsets like the real master would, so
    // device read()/write() paths stay in-bounds if a test exercises them.
    size_t count = 0;
    for (const ec_pdo_entry_reg_t* reg = pdo_entry_regs; reg->index != 0 || reg->vendor_id != 0; ++reg) {
        // A knob slave resolves only entries of its effective mapping; the
        // first one outside it fails the whole list, as on hardware (the
        // entries before it already received offsets).
        if (g_unmapped_entries.count({reg->position, reg->index, reg->subindex}) != 0) {
            record(fmt::format("reg_pdo_entry_list(ENOENT pos={},idx=0x{:x},sub=0x{:x})", reg->position, reg->index,
                               reg->subindex));
            return -ENOENT;
        }
        if (const auto effective = g_effective_mapping.find(reg->position); effective != g_effective_mapping.end()) {
            if (effective->second.count({reg->index, reg->subindex}) == 0) {
                record(fmt::format("reg_pdo_entry_list(ENOENT pos={},idx=0x{:x},sub=0x{:x})", reg->position, reg->index,
                                   reg->subindex));
                return -ENOENT;
            }
        }
        const int offset = assign_offset(reg->position, reg->index, reg->subindex);
        if (offset < 0) {
            record(fmt::format("reg_pdo_entry_list({} pos={},idx=0x{:x},sub=0x{:x})", error_name(offset), reg->position,
                               reg->index, reg->subindex));
            return offset;
        }
        if (reg->offset != nullptr) {
            *reg->offset = static_cast<unsigned int>(offset);
        }
        if (reg->bit_position != nullptr) {
            *reg->bit_position = 0;
        }
        // Per-entry record: multi-axis pins assert exact (slave, index) pairs.
        record(fmt::format("reg_entry(pos={},idx=0x{:x})", reg->position, reg->index));
        g_registered_entries.push_back({reg->position, reg->index, reg->subindex});
        ++count;
    }
    record(fmt::format("reg_pdo_entry_list(entries={})", count));
    return 0;
}

int ecrt_slave_config_reg_pdo_entry(ec_slave_config_t* sc, uint16_t index, uint8_t subindex, ec_domain_t* /*domain*/,
                                    unsigned int* bit_position) {
    const auto position = static_cast<unsigned int>(position_of(sc));
    if (g_unmapped_entries.count({position, index, subindex}) != 0) {
        record(fmt::format("reg_pdo_entry(ENOENT pos={},idx=0x{:x},sub=0x{:x})", position, index, subindex));
        return -ENOENT;
    }
    const int offset = assign_offset(position, index, subindex);
    if (offset < 0) {
        record(fmt::format("reg_pdo_entry({} pos={},idx=0x{:x},sub=0x{:x})", error_name(offset), position, index,
                           subindex));
        return offset;
    }
    if (bit_position != nullptr) {
        *bit_position = 0;
    }
    record(fmt::format("reg_pdo_entry(pos={},idx=0x{:x},sub=0x{:x})", position, index, subindex));
    g_registered_entries.push_back({position, index, subindex});
    return offset;
}

int ecrt_master_activate(ec_master_t* /*master*/) {
    record("master_activate");
    return g_activate_ret;
}

uint8_t* ecrt_domain_data(const ec_domain_t* /*domain*/) {
    return g_domain_data_fails ? nullptr : g_domain_pd;
}

int ecrt_master_set_send_interval(ec_master_t* /*master*/, size_t send_interval) {
    record(fmt::format("set_send_interval(us={})", send_interval));
    return 0;
}

int ecrt_master_application_time(ec_master_t* /*master*/, uint64_t /*app_time*/) {
    record("application_time");
    return 0;
}

int ecrt_master_receive(ec_master_t* /*master*/) {
    record("master_receive");
    return 0;
}

int ecrt_master_send(ec_master_t* /*master*/) {
    record("master_send");
    if (send_observer()) {
        send_observer()();
    }
    return 0;
}

int ecrt_domain_process(ec_domain_t* /*domain*/) {
    return 0;
}

int ecrt_domain_queue(ec_domain_t* /*domain*/) {
    return 0;
}

size_t ecrt_domain_size(const ec_domain_t* /*domain*/) {
    return g_next_offset;  // grid bytes + the SM images of layout slaves (set_slave_domain_layout)
}

int ecrt_domain_state(const ec_domain_t* /*domain*/, ec_domain_state_t* state) {
    std::memset(state, 0, sizeof(*state));
    state->wc_state = static_cast<ec_wc_state_t>(g_domain_wc_state.load());
    return 0;
}

int ecrt_master_sdo_upload(ec_master_t* /*master*/, uint16_t slave_position, uint16_t index, uint8_t subindex,
                           uint8_t* target, size_t target_size, size_t* result_size, uint32_t* abort_code) {
    record(fmt::format("master_sdo_upload(pos={},idx=0x{:x},sub=0x{:x})", slave_position, index, subindex));
    if (const auto failing = g_sdo_upload_transient_failures.find({slave_position, index, subindex});
        failing != g_sdo_upload_transient_failures.end() && failing->second > 0) {
        --failing->second;
        if (abort_code != nullptr) {
            *abort_code = 0;  // as the library on EIO; result_size stays untouched
        }
        return -EIO;
    }
    if (const auto knob = g_sdo_upload_entries.find({slave_position, index, subindex});
        knob != g_sdo_upload_entries.end()) {
        if (knob->second.ret == 0) {
            // A reply larger than the buffer: the master answers -ENOBUFS and writes
            // nothing (bus.cpp reads that code); the string knob below answers the same.
            if (knob->second.bytes.size() > target_size) {
                return -ENOBUFS;
            }
            std::memcpy(target, knob->second.bytes.data(), knob->second.bytes.size());
            *result_size = knob->second.bytes.size();
        } else if (knob->second.ret == -EIO) {
            *abort_code = knob->second.abort_code;
        }
        return knob->second.ret;
    }
    if (const auto text = g_sdo_upload_strings.find(index); text != g_sdo_upload_strings.end()) {
        if (target_size < text->second.size()) {
            return -ENOBUFS;  // as the entry knob above: nothing written
        }
        if (result_size != nullptr) {
            *result_size = text->second.size();
        }
        if (abort_code != nullptr) {
            *abort_code = 0;
        }
        std::memcpy(target, text->second.data(), text->second.size());
        return 0;
    }
    // Anything else fails: the fake never invents drive-declared data.
    if (result_size != nullptr) {
        *result_size = 0;
    }
    if (abort_code != nullptr) {
        *abort_code = 0x06090011;  // CoE "subindex does not exist" style abort
    }
    return -1;
}

int ecrt_slave_config_state(const ec_slave_config_t* sc, ec_slave_config_state_t* state) {
    const int position = position_of(sc);
    record(fmt::format("slave_config_state(pos={})", position));
    if (position < 0) {
        past_the_fake_ring("ecrt_slave_config_state (a slave config the fake did not hand out)", position);
    }
    const ConfigStateKnob& knob = g_config_states[static_cast<size_t>(position)];
    if (knob.fails) {
        return -EINVAL;
    }
    state->online = knob.al != 0 ? 1 : 0;
    state->al_state = knob.al;
    state->operational = knob.al == 8 && !knob.force ? 1 : 0;
    return 0;
}

// Distributed clocks

int ecrt_slave_config_dc(ec_slave_config_t* sc, uint16_t assign_activate, uint32_t sync0_cycle, int32_t sync0_shift,
                         uint32_t sync1_cycle, int32_t sync1_shift) {
    record(fmt::format("slave_config_dc(pos={},aa=0x{:x},cycle={},shift={},sync1_cycle={},sync1_shift={})",
                       position_of(sc), assign_activate, sync0_cycle, sync0_shift, sync1_cycle, sync1_shift));
    const auto failure = g_dc_failures.find(static_cast<uint16_t>(position_of(sc)));
    return failure == g_dc_failures.end() ? 0 : failure->second;
}

int ecrt_master_select_reference_clock(ec_master_t* /*master*/, ec_slave_config_t* sc) {
    record(fmt::format("select_reference_clock(pos={})", position_of(sc)));
    return g_reference_clock_ret;
}

int ecrt_master_sync_reference_clock(ec_master_t* /*master*/) {
    record("sync_reference_clock");
    return 0;
}

int ecrt_master_sync_slave_clocks(ec_master_t* /*master*/) {
    record("sync_slave_clocks");
    return 0;
}

int ecrt_master_sync_monitor_queue(ec_master_t* /*master*/) {
    record("sync_monitor_queue");
    return 0;
}

uint32_t ecrt_master_sync_monitor_process(const ec_master_t* /*master*/) {
    record("sync_monitor_process");
    return g_sync_monitor_drift_ns;
}
