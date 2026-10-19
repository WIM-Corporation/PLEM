// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/fieldbus/ethercat/bus.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iterator>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <plem/core/logging/logger.hpp>
#include <plem/core/timing/cycle_conversion.hpp>

#include "plem/fieldbus/ethercat/config/configuration.hpp"
#include "plem/fieldbus/ethercat/pdo_layout_validation.hpp"
#include "plem/fieldbus/ethercat/pdo_scan.hpp"

#include "ethercat/pdo_scan_messages.hpp"
#include "ethercat/slave_axes.hpp"

namespace plem::fieldbus::ethercat {

static auto& logger = plem::core::Logger::instance();

namespace {

/// An SDO answer's first `size` bytes (CoE is little-endian), zero-extended.
uint32_t le_u32(const uint8_t* data, size_t size) noexcept {
    uint32_t value = 0;
    for (size_t b = 0; b < size; ++b) {
        value |= uint32_t{data[b]} << (8 * b);
    }
    return value;
}

/// What a blocking SDO upload answered, after `attempts` calls.
struct SdoUpload {
    int ret = 0;
    size_t result_size = 0;
    uint32_t abort_code = 0;
    size_t attempts = 0;
};

/// Blocking SDO upload: init context only, never the RT loop. One that fails
/// with -EIO and no abort code is read again after 100, 300 and 500 ms: IgH 1.6
/// can hand it the reply to the master's own mailbox traffic (its SDO
/// dictionary read after a bus scan). A heuristic: the retries span 0.9 s, and
/// a success after them does not prove that was the cause. Each attempt is a
/// full blocking transfer, and a signal during a delay does not end the reads.
/// Both outputs are reset before every attempt.
SdoUpload upload_sdo(ec_master_t* master, uint32_t position, uint16_t index, uint8_t subindex, uint8_t* data,
                     size_t size) {
    using namespace std::chrono_literals;
    constexpr std::chrono::milliseconds kRetryDelays[] = {100ms, 300ms, 500ms};
    SdoUpload upload;
    while (true) {
        upload = SdoUpload{.attempts = upload.attempts + 1};
        upload.ret = ecrt_master_sdo_upload(master, static_cast<uint16_t>(position), index, subindex, data, size,
                                            &upload.result_size, &upload.abort_code);
        if (upload.ret != -EIO || upload.abort_code != 0 || upload.attempts > std::size(kRetryDelays)) {
            break;
        }
        std::this_thread::sleep_for(kRetryDelays[upload.attempts - 1]);
    }
    if (upload.ret == 0 && upload.attempts > 1) {
        logger.info(
            "[EtherCATBus] slave {}: 0x{:04x}:{:02x} read on attempt {}; the earlier ones failed without an SDO "
            "abort. A likely cause (not confirmed) is the master's own SDO dictionary read after a bus scan",
            position, index, subindex, upload.attempts);
    }
    return upload;
}

/// CoE 0x1C32 (SM2 synchronization parameters) :02 Cycle time.
constexpr uint16_t kSmOutputParamsIndex = 0x1C32;
constexpr uint8_t kCycleTimeSubindex = 0x02;

/// PDO mapping (0x1600–0x17FF, 0x1A00–0x1BFF) or sync-manager PDO assignment
/// (0x1C10–0x1C2F, SM0..SM31) objects: PDO configuration, not a stored setting.
bool is_pdo_configuration_object(uint16_t index) {
    return (index >= 0x1600 && index <= 0x17FF) || (index >= 0x1A00 && index <= 0x1BFF) ||
           (index >= 0x1C10 && index <= 0x1C2F);
}

/// True when `layout` maps (index, subindex) into an output sync manager: the
/// master sends that whole image every cycle, whatever the axis mode uses.
bool in_output_process_data(const SlavePdoMapping& layout, const StartupSdo& sdo) {
    for (uint8_t s = 0; s < layout.sync_count; ++s) {
        const SyncManagerAssignment& sm = layout.syncs[s];
        for (uint8_t p = 0; sm.output && p < sm.pdo_count; ++p) {
            for (uint8_t e = 0; e < sm.pdos[p].entry_count; ++e) {
                if (sm.pdos[p].entries[e].index == sdo.index && sm.pdos[p].entries[e].subindex == sdo.subindex) {
                    return true;
                }
            }
        }
    }
    return false;
}

/// Why an entry list registration failed, by name: for each slave in
/// `regs[0, count)` not in `declared` (whose configured layout supersedes what
/// the master read), the registered entries the master's mapping of it lacks,
/// with its identity. Empty when every such slave maps every entry or its
/// mapping cannot be read; the caller then keeps the generic causes.
std::string describe_unmapped_entries(ec_master_t* master, unsigned int master_index, const ec_pdo_entry_reg_t* regs,
                                      uint32_t count, const std::vector<uint32_t>& declared) {
    std::vector<uint16_t> positions;
    for (uint32_t i = 0; i < count; ++i) {
        if (std::find(declared.begin(), declared.end(), regs[i].position) == declared.end() &&
            std::find(positions.begin(), positions.end(), regs[i].position) == positions.end()) {
            positions.push_back(regs[i].position);
        }
    }
    std::string text;
    for (const uint16_t position : positions) {
        const auto scanned = scan_pdo_mapping(master, position);
        if (!scanned.is_ok()) {
            continue;
        }
        const SlavePdoMapping& layout = scanned.value().view;
        const auto mapped = [&](const ec_pdo_entry_reg_t& reg) {
            for (uint8_t s = 0; s < layout.sync_count; ++s) {
                const SyncManagerAssignment& sm = layout.syncs[s];
                for (uint8_t p = 0; p < sm.pdo_count; ++p) {
                    for (uint8_t e = 0; e < sm.pdos[p].entry_count; ++e) {
                        if (sm.pdos[p].entries[e].index == reg.index &&
                            sm.pdos[p].entries[e].subindex == reg.subindex) {
                            return true;
                        }
                    }
                }
            }
            return false;
        };
        std::vector<std::string> missing;
        size_t registered = 0;
        for (uint32_t i = 0; i < count; ++i) {
            if (regs[i].position == position) {
                ++registered;
                if (!mapped(regs[i])) {
                    missing.push_back(fmt::format("0x{:04x}:{:02x}", regs[i].index, regs[i].subindex));
                }
            }
        }
        if (missing.empty()) {
            continue;
        }
        text += fmt::format(
            " slave {} (vendor 0x{:x} product 0x{:x} revision 0x{:x}) does not map {} of the {} PDO entries registered "
            "for it: {} — the mapping the master read from the slave lacks them, typically because the slave runs "
            "firmware other than the one its device was written for. Read the firmware version with "
            "`ethercat -m {} upload -p {} 0x100A 0 --type string` and install firmware that maps these entries.",
            position, scanned.value().vendor_id, scanned.value().product_code, scanned.value().revision_number,
            missing.size(), registered, fmt::join(missing, ", "), master_index, position);
    }
    return text;
}

/// A declared identity text for the log: the array is length-bounded, not NUL-terminated.
template <size_t N>
std::string declared_text(const std::optional<std::array<char, N>>& text) {
    return text ? std::string(text->data(), strnlen(text->data(), N)) : std::string("not declared");
}

}  // namespace

EtherCATBus::EtherCATBus(unsigned int master_index)
    : master_index_(master_index), master_(nullptr), domain_(nullptr), domain_pd_(nullptr) {}

EtherCATBus::~EtherCATBus() {
    release_master();
}

void EtherCATBus::release_master() noexcept {
    // The devices borrow slave configs the master owns; they go first, so no
    // device outlives its master and a retried initialize() starts empty.
    servo_devices_.clear();
    io_devices_.clear();
    // IgH keeps the /dev/EtherCAT reservation until ecrt_release_master; dropping
    // the handle without releasing leaks the master for the process lifetime and
    // blocks every later request (e.g. a bring-up retry after a failed init).
    if (master_) {
        ecrt_release_master(master_);
        master_ = nullptr;
    }
    domain_ = nullptr;  // owned by the master, released with it
    domain_pd_ = nullptr;
}

void EtherCATBus::set_probe(bool probe) noexcept {
    probe_ = probe;
}

bool EtherCATBus::identity_rebenchable() const noexcept {
    return identity_rebenchable_;
}

void EtherCATBus::set_servo_config(std::vector<ServoInfo> servo_infos) {
    injected_servo_config_ = std::move(servo_infos);
}

void EtherCATBus::set_io_config(std::vector<IOInfo> io_infos) {
    injected_io_config_ = std::move(io_infos);
}

void EtherCATBus::set_io_device_factory(IODeviceFactory factory) {
    io_device_factory_ = std::move(factory);
}

void EtherCATBus::retain_pdo_mapping_storage(std::shared_ptr<const void> storage) {
    if (storage) {
        retained_pdo_mapping_storage_.push_back(std::move(storage));
    }
}

void EtherCATBus::set_cycle_time_ns(uint64_t cycle_time_ns) noexcept {
    injected_cycle_time_ns_ = cycle_time_ns;
}

void EtherCATBus::set_dc_enable(bool enable) noexcept {
    dc_enable_ = enable;
}

bool EtherCATBus::dc_enabled() const noexcept {
    return dc_enable_;
}

void EtherCATBus::set_startup_sdos(std::vector<SlaveSdos> slaves) {
    injected_startup_sdos_ = std::move(slaves);
}

void EtherCATBus::set_stored_sdos(std::vector<SlaveSdos> slaves) {
    injected_stored_sdos_ = std::move(slaves);
}

void EtherCATBus::set_expected_slave_identity(std::vector<SlaveExpectedIdentity> slaves) {
    injected_expected_identity_ = std::move(slaves);
}

std::string EtherCATBus::scanned_pdo_mapping_digest(uint32_t position) const {
    const auto it = std::find_if(scanned_pdo_mapping_digests_.begin(), scanned_pdo_mapping_digests_.end(),
                                 [position](const auto& entry) { return entry.first == position; });
    return it == scanned_pdo_mapping_digests_.end() ? std::string{} : it->second;
}

core::Result<void> EtherCATBus::initialize() {
    // A refusal goes to the caller, which prints it once (the drive host's or
    // the robot's create()); after the master is acquired it is released
    // first, so a retry starts clean.
    auto refuse = [this](std::string why) {
        release_master();
        return core::Result<void>::error(std::move(why));
    };
    // A refusal releases the master, so a held master means a completed initialize();
    // requesting again would overwrite the handle and leak the first reservation.
    if (master_) {
        return core::Result<void>::error(
            "[EtherCATBus] this bus already holds its master (initialize() ran); build a new EtherCATBus to initialize "
            "again");
    }
    identity_rebenchable_ = false;
    // Step 0. Validate the injected servo configuration (Fail Fast)
    // validate_bus_configuration (configuration.hpp): zero ecrt calls on a
    // refusal, so a garbage config never leaves a half-acquired master behind.
    if (core::Result<void> config = validate_bus_configuration(injected_servo_config_, injected_startup_sdos_,
                                                               injected_stored_sdos_, injected_expected_identity_);
        !config) {
        return config;
    }
    // Step 1. Generate EtherCAT Master and Domain
    logger.debug("[EtherCATBus] Requesting master index {}", master_index_);
    master_ = ecrt_request_master(master_index_);
    if (!master_) {
        return core::Result<void>::error(fmt::format(
            "Failed to request EtherCAT master {} (ecrt_request_master() returned NULL) — possible causes: another "
            "process holds it (another drive bus or robot control node on this master: stop it first); the IgH "
            "master is not running (check `ethercat master` and `systemctl status ethercat`); or this user cannot "
            "open /dev/EtherCAT{} — the udev rule /etc/udev/rules.d/99-ethercat.rules (RT setup menu 3) sets its "
            "group (the setup wizard's default: realtime), which `id -nG` must list; if not, `sudo usermod -aG "
            "realtime $USER`, then log in again (docs/install.md §3).",
            master_index_, master_index_));
    }
    logger.debug("[EtherCATBus] EtherCAT Master requested successfully.");

    domain_ = ecrt_master_create_domain(master_);
    if (!domain_) {
        return refuse(
            "Failed to create the EtherCAT domain (ecrt_master_create_domain() returned NULL) — possibly the master "
            "is out of memory or was left in a bad state by an earlier process. Restarting it "
            "(`sudo systemctl restart ethercat`) may clear that, but it stops every EtherCAT bus on this machine.");
    }
    logger.debug("[EtherCATBus] EtherCAT Domain created successfully.");

    ec_master_info_t master_info{};
    if (const int ret = ecrt_master(master_, &master_info); ret < 0) {
        return refuse(fmt::format(
            "Failed to read the EtherCAT master information (ecrt_master() returned {}) — retry; if it repeats, "
            "check `ethercat master`.",
            ret));
    }
    logger.info("[EtherCATBus] Slave count: {}, Link state: {}", master_info.slave_count,
                master_info.link_up ? "up" : "down");

    // Step 2. Load Slave information from the injected configuration
    if (injected_servo_config_.empty()) {
        return refuse("No servo config provided. Call set_servo_config() before initialize().");
    }

    const std::vector<ServoInfo>& servo_list = injected_servo_config_;
    const std::vector<IOInfo>& io_list = injected_io_config_;
    const size_t servo_count = servo_list.size();
    const size_t io_count = io_list.size();
    // The active control period is injected by the bus's assembler
    // (set_cycle_time_ns); without one the bus runs the 1 ms default.
    cycle_time_ns_ = injected_cycle_time_ns_ != 0 ? injected_cycle_time_ns_ : kDefaultCycleTimeNs;
    logger.info("[EtherCATBus] EtherCAT configuration ready. Cycle time: {} ns, Servo count: {}, IO count: {}",
                cycle_time_ns_, servo_count, io_count);

    // Step 3. Register slaves (servos and IOs) to the master
    // All servos must initialize - partial operation is unsafe for robot control.
    //
    // A servo's slave position defaults to its index in the servo list (one
    // axis per slave). Multi-axis drives override it: two joints sharing a
    // slave carry the same slave_position and are told apart by
    // ServoInfo::od_offset (the CiA 402 channel base).
    //
    // A scanned slave (null layout) is read from the master here, before its
    // devices exist: its first axis scans the mapping (scan_pdo_mapping), the
    // same validator that checked the declarations at Step 0 checks it with
    // every axis of the slave, and the assembled layout (retained by the bus
    // like a materialized declaration) is what every axis's DeviceServo
    // borrows. The device surface does not change: a device always holds a
    // non-null layout it did not allocate. A refusal here has the shape of
    // every post-acquisition refusal (release the master, refuse). The scanned
    // slave's revision is logged with the mapping so a rejection names the
    // firmware it saw; the identity line for every servo slave comes from the
    // device's own cached ec_slave_info_t (no extra master query).
    uint32_t max_servo_slave = 0;
    // Slaves seen so far, in first-axis order: position → the layout every
    // axis of it borrows (the declaration, or the scan this run assembled). A
    // position not yet listed is being met by its first axis (scan / identity
    // line); a listed one is a later axis of a multi-axis slave (share the
    // scan).
    std::vector<std::pair<uint32_t, const SlavePdoMapping*>> slaves_seen;
    scanned_pdo_mapping_digests_.clear();
    // The master configures a slave under the declared identity but reads its
    // mapping and registers its PDO entries under the identity on the ring, so
    // a mismatch would surface only as a scan rejection or IgH's misleading
    // "Failed to create slave config"; name both pairs instead.
    // ok when the slave at that position is the declared drive, else why not.
    auto check_identity = [&](size_t i, uint32_t slave_pos, const ServoInfo& declared, uint32_t vendor_id,
                              uint32_t product_code) -> core::Result<void> {
        if (vendor_id == declared.vendor_id && product_code == declared.product_code) {
            return core::Result<void>::ok();
        }
        return core::Result<void>::error(fmt::format(
            "axis {} (slave {}): the configuration declares vendor 0x{:x} product 0x{:x}, but the "
            "slave at that ring position is vendor 0x{:x} product 0x{:x} — a different drive sits there, or the "
            "cabling order differs from the configuration. In a drive model file, copy vendor_id and product_code "
            "from `ethercat slaves -v` or fix ring_position; from C++, check ServoInfo::vendor_id/product_code "
            "and the wiring. Refusing to initialize.",
            i, slave_pos, declared.vendor_id, declared.product_code, vendor_id, product_code));
    };
    for (size_t i = 0; i < servo_count; ++i) {
        ServoInfo servo_info = servo_list[i];  // a copy: the injected list keeps its null (= scanned)
        const uint32_t slave_pos = static_cast<uint32_t>(slave_of(servo_list, i));
        max_servo_slave = std::max(max_servo_slave, slave_pos);
        const auto seen = std::find_if(slaves_seen.begin(), slaves_seen.end(),
                                       [&](const auto& entry) { return entry.first == slave_pos; });
        const bool first_axis = seen == slaves_seen.end();
        if (servo_info.pdo_mapping == nullptr) {
            if (!first_axis) {
                servo_info.pdo_mapping = seen->second;  // a later axis of an already-scanned slave
            } else {
                const SlaveAxes group = axes_of_slave(injected_servo_config_, static_cast<int32_t>(slave_pos));
                auto scanned = scan_pdo_mapping(master_, static_cast<uint16_t>(slave_pos));
                if (!scanned.is_ok()) {
                    return refuse(fmt::format(
                        "slave {} (axes {}): cannot read the slave's PDO mapping from the master — {}. Refusing to "
                        "initialize.",
                        slave_pos, group.members, scanned.message()));
                }
                auto layout = std::make_shared<const ScannedPdoMapping>(std::move(scanned).value());
                if (const core::Result<void> identity =
                        check_identity(i, slave_pos, servo_info, layout->vendor_id, layout->product_code);
                    !identity) {
                    return refuse(identity.message());
                }
                if (const core::Result<void> mapping = validate_pdo_mapping(layout->view, group.axes); !mapping) {
                    return refuse(describe_scan_rejection(static_cast<uint16_t>(slave_pos), group.members, *layout,
                                                          mapping.message()) +
                                  " Refusing to initialize.");
                }
                logger.info("[EtherCATBus] slave {} (axes {}): using the scanned PDO mapping — {}", slave_pos,
                            group.members, describe_scanned_mapping(*layout));
                logger.debug("[EtherCATBus] slave {}: scanned PDO mapping\n{}", slave_pos,
                             format_pdo_mapping(layout->view));
                scanned_pdo_mapping_digests_.emplace_back(slave_pos, pdo_mapping_digest(layout->view));
                servo_info.pdo_mapping = &layout->view;
                retain_pdo_mapping_storage(std::move(layout));
            }
        }
        if (first_axis) {
            slaves_seen.emplace_back(slave_pos, servo_info.pdo_mapping);
        }
        std::shared_ptr<DeviceServo> device;
        try {
            device = std::make_shared<DeviceServo>(master_, slave_pos, servo_info);
        } catch (const std::exception& e) {
            return refuse(fmt::format(
                "axis {} (slave {}, vendor=0x{:x}, product=0x{:x}): {} — check that a slave sits at that position on "
                "the ring and that the configuration declares one identity per position. Refusing to initialize.",
                i, slave_pos, servo_info.vendor_id, servo_info.product_code, e.what()));
        }
        const ec_slave_info_t& found = device->get_slave_info();
        if (const core::Result<void> identity =
                check_identity(i, slave_pos, servo_info, found.vendor_id, found.product_code);
            !identity) {
            return refuse(identity.message());
        }
        // Declared in ms, counted in frames: the bus period converts it (0 ms
        // = one frame at any period).
        device->set_fault_reset_pulse_frames(
            plem::core::cycles_at_least_ms(servo_info.fault_reset_pulse_ms, static_cast<int64_t>(cycle_time_ns_)));
        if (servo_info.position_before_enable_index != 0) {
            seed_position_before_enable(*device, slave_pos, servo_info);
        }
        if (first_axis) {
            const ec_slave_info_t& id = device->get_slave_info();
            logger.info(
                "[EtherCATBus] slave {}: vendor 0x{:x} product 0x{:x} revision 0x{:x} serial {} ({} PDO layout)",
                slave_pos, id.vendor_id, id.product_code, id.revision_number, id.serial_number,
                servo_list[i].pdo_mapping ? "declared" : "scanned");
        }
        register_servo(std::move(device));
    }

    // IO slaves follow the last servo slave, not the last servo index; those
    // differ as soon as any slave carries more than one axis. Every IO is
    // served by the injected factory (one call per IO, live master + bus
    // slave position + its IOInfo + the bus period); a missing factory, a
    // throw or a null return fails initialization fast.
    const uint32_t io_base = servo_count > 0 ? max_servo_slave + 1 : 0;
    if (io_count > 0 && !io_device_factory_) {
        return refuse(fmt::format(
            "{} IO slave(s) declared but no IO device factory is injected — call set_io_device_factory() before "
            "initialize().",
            io_count));
    }
    if (io_device_factory_ && io_count == 0) {
        return refuse(
            "an IO device factory is injected but no IO slave is declared — the factory can never fire (declare the "
            "IO, or drop the factory).");
    }
    for (size_t i = 0; i < io_count; ++i) {
        const auto& io_info = io_list[i];
        const uint32_t slave_pos = io_base + static_cast<uint32_t>(i);
        std::shared_ptr<DeviceBase> device;
        try {
            device =
                io_device_factory_(IODeviceContext{master_, slave_pos, io_info, static_cast<int64_t>(cycle_time_ns_)});
        } catch (const std::exception& e) {
            // A throwing factory must not leak the requested master:
            // every other initialize() failure path releases it (an IgH
            // master reservation blocks the whole interface until process
            // exit), and the factory is user code, the one place a throw is
            // likely rather than theoretical. A device that cannot configure
            // its slave throws here too.
            return refuse(fmt::format(
                "IO device factory threw for IO {} (slave {}, vendor=0x{:x}, product=0x{:x}): {} — if the slave "
                "configuration failed, check that a slave sits at that position on the ring and that the "
                "configuration declares one identity per position. Refusing to initialize.",
                i, slave_pos, io_info.vendor_id, io_info.product_code, e.what()));
        } catch (...) {
            // The factory type does not restrict throws to std::exception, so
            // a non-standard throw must also refuse and release the master.
            return refuse(
                fmt::format("IO device factory threw a NON-STANDARD exception for IO {} (slave {}, vendor=0x{:x}, "
                            "product=0x{:x}) — no reason text available.",
                            i, slave_pos, io_info.vendor_id, io_info.product_code));
        }
        if (!device) {
            return refuse(fmt::format(
                "IO device factory returned null for IO {} (slave {}, vendor=0x{:x}, product=0x{:x}) — the factory "
                "must construct the device.",
                i, slave_pos, io_info.vendor_id, io_info.product_code));
        }
        register_io(std::move(device));
    }
    logger.debug("[EtherCATBus] All devices initialized and registered successfully.");

    // Step 3.5. Verify the expected slave identities (drive-model schema §6)
    // An independent step (not DC's, so it runs with DC off) after the
    // master is acquired and before any configuration: a verified record
    // binds motion to the drives it was made on.
    if (const core::Result<void> identity = verify_slave_identity(); !identity) {
        return refuse(identity.message());
    }
    if (probe_) {
        release_master();
        return core::Result<void>::ok();
    }

    // Step 3.6. Compare the stored drive values (drive-model schema §4.9)
    // Warn only: a difference or a failed read never refuses, and nothing is
    // written. Before any configuration, against the layouts Step 3 settled.
    check_stored_sdos(slaves_seen);

    // Step 4. Configure PDOs & SDOs for all devices
    if (core::Result<void> pdos = configure_PDOs(); !pdos) {
        return refuse(pdos.message());
    }
    logger.debug("[EtherCATBus] PDOs configured successfully.");

    // Each IO device finishes against the registered domain (entries
    // registered one at a time, its own startup SDOs), before the servo
    // startup SDOs.
    // release_master() clears the lists: return right after it.
    for (const auto& io_device : io_devices_) {
        if (const core::Result<void> configured = io_device->configure_after_domain(domain_); !configured) {
            return refuse(fmt::format("IO device (slave {}) failed to configure after the domain: {}",
                                      io_device->position(), configured.message()));
        }
    }

    if (const core::Result<void> sdos = configure_SDOs(); !sdos) {
        return refuse(sdos.message());
    }
    logger.debug("[EtherCATBus] SDOs configured successfully.");

    // Step 4.5. Configure distributed clocks (DC) (before activation)
    // IgH rejects DC configuration after ecrt_master_activate(); this ordering
    // is a contract, pinned by the fake-master sequence test.
    if (dc_enable_) {
        if (const core::Result<void> dc = configure_DC(); !dc) {
            return refuse(dc.message());
        }
        logger.debug("[EtherCATBus] Distributed clocks configured successfully.");
    }

    // Step 5. Activate the EtherCAT master
    if (const core::Result<void> active = activate_master(); !active) {
        return refuse(active.message());
    }
    logger.info("[EtherCATBus] EtherCAT master activated successfully.");
    // The domain's process-data size: what a PDO registration change must
    // (or must not) move; compare it across builds on the same bus.
    logger.info("[EtherCATBus] Domain process data: {} bytes.", ecrt_domain_size(domain_));

    return core::Result<void>::ok();
}

void EtherCATBus::register_servo(std::shared_ptr<DeviceServo> servo_device) {
    servo_devices_.push_back(servo_device);
    logger.debug("[EtherCATBus] Servo device registered. Total servo devices: {}", servo_devices_.size());
}

void EtherCATBus::register_io(std::shared_ptr<DeviceBase> device) {
    io_devices_.push_back(std::move(device));
    logger.debug("[EtherCATBus] IO device registered. Total IO devices: {}", io_devices_.size());
}

core::Result<void> EtherCATBus::configure_PDOs() {
    ec_pdo_entry_reg_t domain_regs[kMaxDomainRegs] = {};
    uint32_t domain_index = 0;

    // Write declared PDO mappings before registering any entry (PREOP:
    // 0x1c12/0x1c13 are writable only before activation). A slave whose
    // power-on assignment is incomplete otherwise fails registration with an
    // opaque ENOENT, because there is no entry list to resolve offsets
    // against. A scanned slave (injected null) is not written: the master
    // already holds the mapping Step 3 validated, and it copies that very
    // cache into the slave config at attach; registration resolves against
    // what the validator saw. Its position is entered as "configured" up
    // front, so the loop below skips it like an already-applied slave.
    //
    // Once per slave, not per device: the axes of a multi-axis slave share one
    // ec_slave_config_t, so applying per device would have each axis overwrite
    // the previous one's mapping. initialize() Step 0 already rejected axes of
    // one slave disagreeing on the layout, so first-seen == every axis's view.
    std::vector<uint32_t> layout_configured_slaves;
    for (size_t i = 0; i < injected_servo_config_.size(); ++i) {
        if (injected_servo_config_[i].pdo_mapping == nullptr) {
            layout_configured_slaves.push_back(static_cast<uint32_t>(slave_of(injected_servo_config_, i)));
        }
    }
    for (auto& servo_device : servo_devices_) {
        const uint32_t pos = servo_device->position();
        if (std::find(layout_configured_slaves.begin(), layout_configured_slaves.end(), pos) !=
            layout_configured_slaves.end()) {
            continue;  // already applied by this slave's first axis (same layout, validated)
        }
        if (core::Result<void> applied = servo_device->apply_pdo_mapping(); !applied) {
            return applied;
        }
        layout_configured_slaves.push_back(pos);
    }

    for (auto servo_device : servo_devices_) {
        if (const core::Result<void> registered = servo_device->register_PDOs(domain_regs, domain_index); !registered) {
            return core::Result<void>::error(fmt::format("Failed to register PDOs for servo device (slave {}): {}",
                                                         servo_device->position(), registered.message()));
        }
    }

    for (const auto& io_device : io_devices_) {
        if (const core::Result<void> registered = io_device->register_PDOs(domain_regs, domain_index); !registered) {
            return core::Result<void>::error(fmt::format("Failed to register PDOs for IO device (slave {}): {}",
                                                         io_device->position(), registered.message()));
        }
    }

    if (const int ret = ecrt_domain_reg_pdo_entry_list(domain_, domain_regs); ret < 0) {
        std::vector<uint32_t> declared_slaves;
        for (size_t i = 0; i < injected_servo_config_.size(); ++i) {
            if (injected_servo_config_[i].pdo_mapping != nullptr) {
                declared_slaves.push_back(static_cast<uint32_t>(slave_of(injected_servo_config_, i)));
            }
        }
        if (std::string unmapped =
                describe_unmapped_entries(master_, master_index_, domain_regs, domain_index, declared_slaves);
            !unmapped.empty()) {
            return core::Result<void>::error(fmt::format(
                "Failed to register the PDO entry list with the domain (ecrt_domain_reg_pdo_entry_list() returned "
                "{}):{}",
                ret, unmapped));
        }
        return core::Result<void>::error(fmt::format(
            "Failed to register the PDO entry list with the domain (ecrt_domain_reg_pdo_entry_list() returned {}) — "
            "possible causes: an entry a slave does not map, a slave whose configuration the master rejected, or the "
            "master out of memory; compare `ethercat pdos` with the declared mapping.",
            ret));
    }

    logger.debug("[EtherCATBus] PDO entry list registered successfully.");
    return core::Result<void>::ok();
}

core::Result<void> EtherCATBus::configure_SDOs() {
    // Servo slaves get the injected startup lists (IO devices reserved their
    // own in configure_after_domain).
    // In list order on the slave's one ec_slave_config_t; Step 0 proved every
    // position hosts a servo and every write is reservable.
    for (const SlaveSdos& list : injected_startup_sdos_) {
        const auto host = std::find_if(servo_devices_.begin(), servo_devices_.end(),
                                       [&](const auto& servo) { return servo->position() == list.slave_position; });
        ec_slave_config_t* sc = (*host)->get_config();
        for (const StartupSdo& sdo : list.sdos) {
            const int ret =
                sdo.size == 1   ? ecrt_slave_config_sdo8(sc, sdo.index, sdo.subindex, static_cast<uint8_t>(sdo.value))
                : sdo.size == 2 ? ecrt_slave_config_sdo16(sc, sdo.index, sdo.subindex, static_cast<uint16_t>(sdo.value))
                                : ecrt_slave_config_sdo32(sc, sdo.index, sdo.subindex, sdo.value);
            if (ret < 0) {
                return core::Result<void>::error(
                    fmt::format("slave {}: reserving startup SDO 0x{:04x}:{:02x} failed ({}).", list.slave_position,
                                sdo.index, sdo.subindex, ret));
            }
            logger.info("[EtherCATBus] slave {}: startup SDO 0x{:04x}:{:02x} = 0x{:x} ({} byte) reserved",
                        list.slave_position, sdo.index, sdo.subindex, sdo.value, sdo.size);
        }
    }
    return core::Result<void>::ok();
}

core::Result<void> EtherCATBus::configure_DC() {
    // The SYNC0 latch cycle is a drive property (the cycle the drive runs),
    // never the bus period: a drive supervises SYNC0 against the cycle it
    // supports; on the observed drive, programming the bus period when it
    // differed from the supported SYNC0 cycle caused AL 0x002C Fatal Sync →
    // SAFEOP looping. The bus
    // period is still demanded explicitly: every latch must align with a
    // completed frame exchange, so SYNC0 must be an integer multiple of it,
    // and falling through to the 1 ms default would misalign any other
    // period. The drive's 0x1C32 is
    // never read: before DC is programmed a drive that mirrors its SYNC0
    // register reads 0 or a stale value. A cycle the drive cannot run is the
    // drive's to refuse, at PREOP→SAFEOP, or as a sync error that drops the
    // slave out of OP (the AL 0x002C loop above); not every drive is known to.
    if (injected_cycle_time_ns_ == 0) {
        return core::Result<void>::error(
            "DC enabled but no cycle time was set. Call set_cycle_time_ns() with the "
            "deployed control period before initialize() — the SYNC0-multiple check needs the real bus period, "
            "never the 1 ms default.");
    }

    for (size_t i = 0; i < servo_devices_.size(); ++i) {
        const auto& info = injected_servo_config_[i];
        if (info.dc_assign_activate == 0) {
            return core::Result<void>::error(fmt::format(
                "DC enabled but axis {} has no dc_assign_activate registered. Declare the "
                "activation word from the drive's ESI (Dc/OpMode AssignActivate) — dc.assign_activate in a drive "
                "model file, ServoInfo::dc_assign_activate from C++ — before enabling DC — refusing to guess an "
                "activation word.",
                i));
        }
        if (info.dc_sync0_cycle_ns == 0) {
            return core::Result<void>::error(
                fmt::format("DC enabled but axis {} has no dc_sync0_cycle_ns registered. Declare the SYNC0 "
                            "cycle the drive runs — dc.sync0_cycle_ns in a drive model file, "
                            "ServoInfo::dc_sync0_cycle_ns from C++ — "
                            "before enabling DC — refusing to guess a SYNC0 cycle.",
                            i));
        }
        // Divisor is the very variable the guard above proved non-zero, never
        // cycle_time_ns_, whose equality with it is an initialize()-order detail.
        if (info.dc_sync0_cycle_ns % injected_cycle_time_ns_ != 0) {
            return core::Result<void>::error(fmt::format(
                "SYNC0 cycle {} ns (drive constant, axis {}) is not an integer multiple of the bus "
                "period {} ns — every SYNC0 event must align with a completed frame exchange "
                "(N frames : 1 SYNC0 event).",
                info.dc_sync0_cycle_ns, i, injected_cycle_time_ns_));
        }
        if (info.dc_write_cycle_time && first_axis_index_of_slave(injected_servo_config_, i) == i) {
            // The drive takes its cycle time from the master (drive-model
            // schema §5): one U32 startup SDO per slave, from the same field
            // that programs SYNC0 below. 0x1C32:01 is left to the slave, which
            // derives it from the activation register.
            const int ret = ecrt_slave_config_sdo32(servo_devices_[i]->get_config(), kSmOutputParamsIndex,
                                                    kCycleTimeSubindex, info.dc_sync0_cycle_ns);
            if (ret < 0) {
                return core::Result<void>::error(
                    fmt::format("slave {}: reserving the cycle time 0x1C32:02 = {} ns failed ({}).",
                                servo_devices_[i]->position(), info.dc_sync0_cycle_ns, ret));
            }
            logger.info("[EtherCATBus] slave {}: startup SDO 0x1C32:02 = {} ns (dc.write_cycle_time) reserved",
                        servo_devices_[i]->position(), info.dc_sync0_cycle_ns);
        }
        const int ret = ecrt_slave_config_dc(servo_devices_[i]->get_config(), info.dc_assign_activate,
                                             info.dc_sync0_cycle_ns, info.dc_sync0_shift_ns, 0, 0);
        if (ret < 0) {
            return core::Result<void>::error(
                fmt::format("Failed to configure DC on axis {} (ecrt_slave_config_dc() returned {}): check "
                            "dc.assign_activate against the drive's ESI",
                            i, ret));
        }
    }

    // Explicit reference clock: the servo slave with the lowest ring position,
    // i.e. the first DC slave the frame meets (IO slaves sit after the servos).
    // IgH compensates propagation delay only downstream of the reference, so
    // file order must not pick it.
    const auto& reference =
        *std::min_element(servo_devices_.begin(), servo_devices_.end(),
                          [](const auto& a, const auto& b) { return a->position() < b->position(); });
    const int ret = ecrt_master_select_reference_clock(master_, reference->get_config());
    if (ret < 0) {
        return core::Result<void>::error(
            fmt::format("Failed to select slave {} as DC reference clock (ecrt_master_select_reference_clock() "
                        "returned {})",
                        reference->position(), ret));
    }
    return core::Result<void>::ok();
}

core::Result<void> EtherCATBus::verify_slave_identity() {
    // Step 0 refused an entry for a position without a servo before any ecrt call.
    const auto device_at = [this](uint32_t position) -> const DeviceServo& {
        return **std::find_if(servo_devices_.begin(), servo_devices_.end(),
                              [&](const auto& servo) { return servo->position() == position; });
    };
    // The order first, over every slot: a drive whose serial the record holds
    // for another slot is not re-benchable: a bench would bind the axis names
    // to other joints. Evidence of a move only: the other slot is of the same
    // model (Step 3 holds each slot to its declared model) and no longer
    // shows that serial (else a duplicate serial, not a move). Serial 0 (a
    // drive without one, or a model-scope record) says nothing.
    for (const SlaveExpectedIdentity& expected : injected_expected_identity_) {
        const ec_slave_info_t& here = device_at(expected.slave_position).get_slave_info();
        const uint32_t observed = here.serial_number;
        if (observed == 0 || (expected.identity.serial_number && *expected.identity.serial_number == observed)) {
            continue;
        }
        for (const SlaveExpectedIdentity& other : injected_expected_identity_) {
            if (other.slave_position == expected.slave_position || !other.identity.serial_number ||
                *other.identity.serial_number != observed) {
                continue;
            }
            const ec_slave_info_t& there = device_at(other.slave_position).get_slave_info();
            if (there.vendor_id == here.vendor_id && there.product_code == here.product_code &&
                there.serial_number != observed) {
                return core::Result<void>::error(
                    fmt::format("slave {} holds the drive the approval recorded for slave {} (serial {}) — the "
                                "drives are in another order than at the bench: restore the cabling",
                                expected.slave_position, other.slave_position, observed));
            }
        }
    }
    for (const SlaveExpectedIdentity& expected : injected_expected_identity_) {
        const DeviceServo* device = &device_at(expected.slave_position);
        const SlaveIdentity& id = expected.identity;
        const uint32_t position = expected.slave_position;
        // Every refusal below is one a bench at these drives would replace.
        identity_rebenchable_ = true;
        if (id.serial_number) {
            const uint32_t observed = device->get_slave_info().serial_number;
            if (observed != *id.serial_number) {
                return core::Result<void>::error(
                    fmt::format("slave {}: SII serial number is {} but the approval record declares {} — this is "
                                "not the drive the record was made on (or its SII changed): run the record's drive, or "
                                "re-bench this one (`plem drive run <file>` on a terminal offers the bench).",
                                position, observed, *id.serial_number));
            }
        }
        if (id.revision_number) {
            const uint32_t observed = device->get_slave_info().revision_number;
            if (observed != *id.revision_number) {
                return core::Result<void>::error(fmt::format(
                    "slave {}: SII revision number is 0x{:x} but the approval record declares 0x{:x} — "
                    "another drive revision than the record's, a changed SII, or a mistyped record: check the "
                    "nameplate and the record, then run the record's drive or re-bench this one "
                    "(`plem drive run <file>` on a terminal offers the bench).",
                    position, observed, *id.revision_number));
            }
        }
        if (id.hardware_version) {
            if (core::Result<void> matches =
                    slave_string_matches(position, 0x1009, *id.hardware_version, "hardware version");
                !matches) {
                return matches;
            }
        }
        if (id.software_version) {
            if (core::Result<void> matches =
                    slave_string_matches(position, 0x100A, *id.software_version, "software version");
                !matches) {
                return matches;
            }
        }
        if (id.pdo_mapping_digest) {
            // Compared with what Step 3 scanned; no master query. The loader
            // refuses a digest on a declared slave; a spec built another way
            // can still carry one, and a value with nothing to compare is a
            // mismatch, never a skip.
            const std::string_view declared(id.pdo_mapping_digest->data(),
                                            strnlen(id.pdo_mapping_digest->data(), id.pdo_mapping_digest->size()));
            const auto scanned = std::find_if(scanned_pdo_mapping_digests_.begin(), scanned_pdo_mapping_digests_.end(),
                                              [&](const auto& entry) { return entry.first == position; });
            if (scanned == scanned_pdo_mapping_digests_.end()) {
                return core::Result<void>::error(describe_digest_without_scan(position, declared));
            }
            if (scanned->second != declared) {
                return core::Result<void>::error(describe_digest_mismatch(position, scanned->second, declared));
            }
        }
        logger.info(
            "[EtherCATBus] slave {}: identity matches the approval record (serial {}, revision number {}, hardware "
            "version {}, software version {}, PDO mapping digest {}).",
            position, id.serial_number ? std::to_string(*id.serial_number) : "not declared",
            id.revision_number ? fmt::format("0x{:x}", *id.revision_number) : "not declared",
            declared_text(id.hardware_version), declared_text(id.software_version),
            declared_text(id.pdo_mapping_digest));
    }
    identity_rebenchable_ = false;
    return core::Result<void>::ok();
}

void EtherCATBus::seed_position_before_enable(DeviceServo& servo, uint32_t position, const ServoInfo& info) {
    const auto read_s32 = [&](uint16_t index, int32_t& value) {
        uint8_t data[4] = {};
        const SdoUpload upload = upload_sdo(master_, position, index, 0, data, sizeof(data));
        if (upload.ret != 0 || upload.result_size != sizeof(data)) {
            logger.warn(
                "[EtherCATBus] slave {}: could not read 0x{:04x}:00 for the position before the first enable "
                "(ret={}, {} bytes, abort=0x{:08x}, attempts: {}); the position is not seeded, so if this drive "
                "reports 0x{:04x} = 0 until its first enable, the axis shows that 0 and mirrors it into 0x{:04x} "
                "until then",
                position, index, upload.ret, upload.result_size, upload.abort_code, upload.attempts,
                0x6064 + info.od_offset, 0x607A + info.od_offset);
            return false;
        }
        value = static_cast<int32_t>(le_u32(data, sizeof(data)));
        return true;
    };
    const auto actual_index = static_cast<uint16_t>(0x6064 + info.od_offset);
    int32_t actual = 0;
    if (!read_s32(actual_index, actual) || actual != 0) {
        return;  // unread, or the drive already reports its position
    }
    int32_t before_enable = 0;
    if (!read_s32(info.position_before_enable_index, before_enable)) {
        return;
    }
    servo.seed_position_before_enable(before_enable);
    logger.info(
        "[EtherCATBus] slave {}: 0x{:04x} reads 0 before the first enable — the axis reports 0x{:04x}:00 = {} until "
        "the drive reports its own position",
        position, actual_index, info.position_before_enable_index, before_enable);
}

void EtherCATBus::check_stored_sdos(const std::vector<std::pair<uint32_t, const SlavePdoMapping*>>& layouts) {
    for (const SlaveSdos& list : injected_stored_sdos_) {
        const uint32_t position = list.slave_position;
        // Step 0 refused a list for a position without a servo; Step 3 listed its layout.
        const SlavePdoMapping& layout = *std::find_if(layouts.begin(), layouts.end(), [&](const auto& seen) {
                                             return seen.first == position;
                                         })->second;
        size_t matched = 0;
        for (size_t v = 0; v < list.sdos.size(); ++v) {
            const StartupSdo& expected = list.sdos[v];
            if (is_pdo_configuration_object(expected.index)) {
                logger.warn(
                    "[EtherCATBus] slave {}: 0x{:04x}:{:02x} — a PDO mapping or assignment object is PDO "
                    "configuration, not a stored drive setting to compare here — drop it from stored_sdo (schema "
                    "§4.9)",
                    position, expected.index, expected.subindex);
                continue;
            }
            if (in_output_process_data(layout, expected)) {
                logger.warn(
                    "[EtherCATBus] slave {}: 0x{:04x}:{:02x} — this object is in the slave's output process data, "
                    "which the master sends every cycle; a stored value cannot be judged here — drop it from "
                    "stored_sdo (schema §4.9)",
                    position, expected.index, expected.subindex);
                continue;
            }
            uint8_t data[4] = {};
            const auto [ret, result_size, abort_code, attempts] =
                upload_sdo(master_, position, expected.index, expected.subindex, data, sizeof(data));
            if (ret == -EINTR) {
                // PLEM sets no timeout of its own. A signal ends a read still
                // queued in the master; once the transfer runs, only IgH's CoE
                // handling bounds it. The signal is consumed (a further read
                // could wait again), so the whole check ends here and
                // initialize() carries on.
                logger.warn(
                    "[EtherCATBus] slave {}: reading 0x{:04x}:{:02x} was interrupted by a signal — the stored_sdo "
                    "check stops here; no further value is read (schema §4.9)",
                    position, expected.index, expected.subindex);
                return;
            }
            if (ret == 0 && result_size == expected.size) {
                const uint32_t actual = le_u32(data, result_size);
                if (actual == expected.value) {
                    ++matched;
                    continue;
                }
                logger.warn(
                    "[EtherCATBus] slave {0}: 0x{1:04x}:{2:02x} reads {3} (0x{3:x}), the model file's stored_sdo "
                    "expects {4} (0x{4:x}) (schema §4.9); this comparison performs no writes. If the file is right "
                    "and the drive manual lists the object as writable and storable, power-cycle first, then with "
                    "the node stopped run ethercat -m {5} -p {0} download -t uint{6} 0x{1:04x} {2} 0x{4:x}. Then run "
                    "ethercat -m {5} -p {0} download -t uint32 0x1010 1 0x65766173 (CiA 301 Store parameters, if "
                    "the drive implements it; see its manual). A save usually stores every storable value, "
                    "including unsaved changes since power-up. Power-cycle again if the manual says the object "
                    "takes effect at power-up. If the drive is right, change the file's stored_sdo.",
                    position, expected.index, expected.subindex, actual, expected.value, master_index_,
                    8 * expected.size);
                continue;
            }
            if (ret == 0) {
                logger.warn(
                    "[EtherCATBus] slave {}: 0x{:04x}:{:02x} answered {} bytes but the model file's stored_sdo says "
                    "size {} — not compared; correct the size (schema §4.9)",
                    position, expected.index, expected.subindex, result_size, expected.size);
                continue;
            }
            if (ret == -ENOBUFS) {
                logger.warn(
                    "[EtherCATBus] slave {}: 0x{:04x}:{:02x} answered more than 4 bytes — stored_sdo compares 1, 2 "
                    "or 4 bytes; not compared, drop it from stored_sdo (schema §4.9)",
                    position, expected.index, expected.subindex);
                continue;
            }
            if (abort_code != 0) {
                logger.warn(
                    "[EtherCATBus] slave {}: could not read 0x{:04x}:{:02x} for stored_sdo (SDO abort 0x{:08x}) — "
                    "not compared; check the object on this drive (schema §4.9)",
                    position, expected.index, expected.subindex, abort_code);
                continue;
            }
            logger.warn(
                "[EtherCATBus] slave {}: could not read 0x{:04x}:{:02x} for stored_sdo — upload failed without an "
                "SDO abort (ret={}, attempts: {}) — no answer, a slave without CoE, or a mailbox error{}; its "
                "remaining {} stored_sdo values are not read (schema §4.9)",
                position, expected.index, expected.subindex, ret, attempts,
                ret == -EIO ? " such as the master's own SDO dictionary read after a bus scan (to compare, start "
                              "again once the bus has been idle for a while)"
                            : "",
                list.sdos.size() - v - 1);
            break;
        }
        if (!list.sdos.empty() && matched == list.sdos.size()) {  // every value read and equal
            logger.info("[EtherCATBus] slave {}: stored_sdo {}/{} match the drive's current values", position, matched,
                        list.sdos.size());
        }
    }
}

core::Result<void> EtherCATBus::slave_string_matches(uint32_t position, uint16_t index,
                                                     const std::array<char, SlaveIdentity::kVersionLength>& declared,
                                                     const char* what) {
    // The answer is length-counted, not NUL-terminated (a 24-bit "0.2" is exact).
    uint8_t data[64] = {};
    const auto [ret, result_size, abort_code, attempts] = upload_sdo(master_, position, index, 0, data, sizeof(data));
    const std::string_view expected(declared.data(), strnlen(declared.data(), declared.size()));
    if (ret == -EIO && abort_code == 0) {
        // Not the record's fault: no re-bench offer.
        identity_rebenchable_ = false;
        return core::Result<void>::error(fmt::format(
            "slave {}: could not read the {} (0x{:04x}:00, ret={}, abort=0x0, attempts: {}) but the approval "
            "record declares '{}' — the read failed without an SDO abort; causes include the master's own SDO "
            "dictionary read after a bus scan (kernel log: `Received unknown response while uploading SDO`) and a "
            "drive that does not answer: start again once the bus has been idle for a while; if it keeps failing, "
            "check the drive's mailbox (CoE), its state (`ethercat slaves`) and the cabling.",
            position, what, index, ret, attempts, expected));
    }
    if (ret != 0) {
        return core::Result<void>::error(fmt::format(
            "slave {}: could not read the {} (0x{:04x}:00, ret={}, abort=0x{:x}, attempts: {}) but the approval "
            "record declares '{}' — a declared value that cannot be read is a mismatch. If this drive has no such "
            "object, the record must declare it null (re-bench: `plem drive run <file>` on a terminal offers the "
            "bench); otherwise fix the drive "
            "or the bus.",
            position, what, index, ret, abort_code, attempts, expected));
    }
    const std::string_view observed(reinterpret_cast<const char*>(data), std::min(result_size, sizeof(data)));
    if (observed != expected) {
        std::string hex;
        for (const unsigned char c : observed) {
            hex += fmt::format("{:02x}", c);
        }
        return core::Result<void>::error(fmt::format(
            "slave {}: {} (0x{:04x}) is '{}' ({} bytes, hex {}) but the approval record declares '{}' "
            "({} bytes) — the drive differs from what the bench observed: run the record's drive, or re-bench "
            "this one (`plem drive run <file>` on a terminal offers the bench). If the two only differ in length (NUL "
            "padding, trailing spaces), this "
            "drive's string cannot be recorded — declare that field null and re-bench.",
            position, what, index, observed, observed.size(), hex, expected, expected.size()));
    }
    return core::Result<void>::ok();
}

void EtherCATBus::sync_distributed_clocks() noexcept {
    if (!dc_enable_ || !master_) {
        return;
    }
    (void)ecrt_master_sync_reference_clock(master_);
    (void)ecrt_master_sync_slave_clocks(master_);
    (void)ecrt_master_sync_monitor_queue(master_);
}

uint32_t EtherCATBus::dc_sync_monitor_process() noexcept {
    if (!dc_enable_ || !master_) {
        return 0;
    }
    return ecrt_master_sync_monitor_process(master_);
}

core::Result<void> EtherCATBus::activate_master() {
    if (!master_ || !domain_) {
        return core::Result<void>::error(
            "EtherCAT master or domain not initialized: initialize() must succeed before activation");
    }

    const auto send_interval_us = static_cast<size_t>(cycle_time_ns_ / 1000U);
    if (send_interval_us > 0) {
        const int interval_ret = ecrt_master_set_send_interval(master_, send_interval_us);
        if (interval_ret < 0) {
            logger.warn("[EtherCATBus] Failed to set EtherCAT master send interval to {} us: {}", send_interval_us,
                        interval_ret);
        } else {
            logger.info("[EtherCATBus] EtherCAT master send interval set to {} us", send_interval_us);
        }
    }

    int ret = ecrt_master_activate(master_);
    if (ret < 0) {
        return core::Result<void>::error(fmt::format(
            "Failed to activate EtherCAT master (ecrt_master_activate() returned {}): the kernel log has the "
            "cause (sudo dmesg | grep -i ethercat)",
            ret));
    }

    domain_pd_ = ecrt_domain_data(domain_);
    if (!domain_pd_) {
        return core::Result<void>::error(
            "Failed to get domain process data (ecrt_domain_data() returned null after activation): restart the "
            "master (sudo systemctl restart ethercat)");
    }
    return core::Result<void>::ok();
}

void EtherCATBus::set_application_time(uint64_t app_time_ns) noexcept {
    if (master_) {
        (void)ecrt_master_application_time(master_, app_time_ns);
    }
}

void EtherCATBus::receive() {
    if (master_)
        ecrt_master_receive(master_);
    if (domain_)
        ecrt_domain_process(domain_);
}

void EtherCATBus::send() {
    if (domain_)
        ecrt_domain_queue(domain_);
    if (master_)
        ecrt_master_send(master_);
}

size_t EtherCATBus::servo_count() const {
    return servo_devices_.size();
}

size_t EtherCATBus::reset_all_faults() {
    // No is_faulted() filter: this runs off the RT thread, which writes the
    // state machine, so reading it here would be a data race. The pulse is
    // decided on the axis state read in the publishing cycle (DeviceServo::write
    // re-checks eligibility).
    for (auto& servo : servo_devices_) {
        servo->request_fault_reset();
    }
    logger.debug("[EtherCATBus] Posted fault-reset requests on {} servo(s)", servo_devices_.size());
    return servo_devices_.size();
}

core::TriState EtherCATBus::motion_possible() const {
    using plem::core::TriState;
    if (servo_devices_.empty()) {
        return TriState::UNDEFINED;
    }
    for (const auto& servo : servo_devices_) {
        if (servo->get_servo_state() != ServoState::OPERATION_ENABLED) {
            return TriState::NO;
        }
    }
    return TriState::YES;
}

uint8_t* EtherCATBus::domain_pd() {
    return domain_pd_;
}

EtherCATBus::DomainState EtherCATBus::get_domain_state() const noexcept {
    DomainState state;
    if (domain_) {
        ec_domain_state_t ds{};
        ecrt_domain_state(domain_, &ds);
        state.wkc = static_cast<uint16_t>(ds.working_counter);
        state.wkc_ok = (ds.wc_state == EC_WC_COMPLETE);
        state.wc_state = static_cast<uint8_t>(ds.wc_state);
    }
    return state;
}

void EtherCATBus::sample_servo_slave_states(std::span<SlaveState> out) noexcept {
    // Bounded by the configuration too: first_axis_index_of_slave reads it, and
    // set_servo_config() on a started bus could have shortened it.
    const size_t count = std::min({out.size(), servo_devices_.size(), injected_servo_config_.size()});
    for (size_t i = 0; i < count; ++i) {
        // The slave's first axis holds the sample, grouped by the same
        // ServoInfo::slave_position as the per-slave configuration steps.
        const size_t first = first_axis_index_of_slave(injected_servo_config_, i);
        if (first < i) {
            out[i] = out[first];
            continue;
        }
        ec_slave_config_state_t state{};
        out[i] =
            ecrt_slave_config_state(servo_devices_[i]->get_config(), &state) == 0
                ? SlaveState{.al_state = static_cast<uint8_t>(state.al_state), .operational = state.operational != 0}
                : SlaveState{};
    }
}

}  // namespace plem::fieldbus::ethercat
