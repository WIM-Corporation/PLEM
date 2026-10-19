// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "plem/fieldbus/ethercat/pdo_layout.hpp"

// ecrt.h only forward-declares the handle structs; mirror its typedef so this
// header stays includable without ecrt.h (identical redeclaration is legal).
struct ec_master;
typedef struct ec_master ec_master_t;

namespace plem::fieldbus::test {

/**
 * @brief Link-seam fake of the IgH ecrt API (test-only).
 *
 * Shipped as the static library plem_fieldbus_fake_master (built, installed
 * and exported only with BUILD_TESTING). A test executable that links it
 * defines every ecrt_* function itself, so EtherCATBus::initialize() runs
 * hermetically and the exact ecrt call sequence is assertable (e.g.
 * "slave_config_dc happens before master_activate") — whether the bus code is
 * compiled into the test (this package's own fake-master targets) or comes
 * from the shared plem_fieldbus library (another package's test: the
 * executable's definitions take precedence over libethercat's). Not linked
 * into the production library.
 *
 * @thread_safety Set knobs and read the log while no RT loop runs; only
 * set_domain_wc_state() is atomic. The send observer runs on the loop thread.
 */
struct FakeEcrt {
    /// Slave positions the fake models: 0..kMaxSlaves-1. A position past it
    /// (a slave config, a state knob) aborts the test with a message naming
    /// the limit; it is the fake's ring size, not a master refusal.
    static constexpr unsigned int kMaxSlaves = 16;

    /// Ordered log of every faked ecrt_* call, formatted with its
    /// load-bearing arguments (e.g. "slave_config_dc(pos=0,aa=0x300,...)").
    static std::vector<std::string>& calls();

    /// Clear the call log and reset all knobs to defaults.
    static void reset();

    /// Value ecrt_master_sync_monitor_process() returns (default 0).
    /// Set to UINT32_MAX to simulate the IgH monitor-unavailable error sentinel.
    static void set_sync_monitor_drift_ns(unsigned int value);

    /// Make ecrt_master_get_slave() report this SII serial number for the
    /// slave at `position` (default, and after reset(): 0, what a drive
    /// without an SII serial reports).
    static void set_slave_serial(unsigned int position, unsigned int serial);

    /// Make ecrt_master_sdo_upload() of `index` (any position, subindex 0)
    /// answer exactly these bytes: length-counted, no NUL, as a real 0x1009
    /// / 0x100A string arrives. Any upload no knob answers fails (ret < 0);
    /// the fake never invents drive-declared data. Cleared by reset().
    static void set_sdo_upload_string(unsigned int index, std::string bytes);

    /// Make ecrt_master_sdo_upload() of (position, index, subindex) return
    /// `ret` the way the userspace library does: 0 = these bytes and their
    /// length, -EIO = this abort code, any other code touches neither output.
    /// Takes precedence over the string knob above. Cleared by reset().
    static void set_sdo_upload_entry(unsigned int position, unsigned int index, unsigned int subindex, int ret,
                                     std::vector<uint8_t> bytes, uint32_t abort_code = 0);

    /// Make the next `count` ecrt_master_sdo_upload() calls of (position,
    /// index, subindex) fail with -EIO and no abort code (IgH 1.6 when the
    /// master's own mailbox traffic answers the upload); later calls answer
    /// from the knobs above. Cleared by reset().
    static void set_sdo_upload_transient_failures(unsigned int position, unsigned int index, unsigned int subindex,
                                                  int count);

    /// Make the slave at `position` scannable: ecrt_master_get_slave() reports
    /// `layout.sync_count` sync managers and this SII `revision`, and the three
    /// scan queries (get_sync_manager / get_pdo / get_pdo_entry) answer from
    /// `layout` (borrowed, static test storage). From then on `layout` is the
    /// slave's effective mapping: ecrt_domain_reg_pdo_entry_list() fails with
    /// -ENOENT at the first entry of that slave outside it (the real master
    /// resolves offsets against the slave's current mapping), and a later
    /// ecrt_slave_config_pdos() on that slave replaces the effective mapping
    /// with what it was handed (the real master's order: the configured
    /// assignment supersedes the scanned one). Slaves without this knob report
    /// sync_count 0 and accept every registration. Cleared by reset().
    static void set_slave_scan(unsigned int position, unsigned int revision,
                               const plem::fieldbus::ethercat::SlavePdoMapping& layout);

    /// Make every scan query (get_sync_manager / get_pdo / get_pdo_entry) of
    /// the slave at `position` fail with -EIO, the "master could not answer"
    /// branch. Combine with set_slave_scan (the slave still reports its
    /// sync_count). Cleared by reset().
    static void set_slave_scan_query_failure(unsigned int position);

    /// Make ecrt_master_get_slave() report this vendor/product for the slave at
    /// `position`. Default: the identity its first slave config named (the
    /// ring holds what the configuration declares), or the synthetic drive
    /// (kSyntheticVendorId/kSyntheticProductCode) before any. Cleared by reset().
    static void set_slave_identity(unsigned int position, unsigned int vendor_id, unsigned int product_code);

    /// Make ecrt_master_slave_config() of the slave at `position` return null:
    /// the master refuses to create the slave configuration (the device
    /// constructor throws). Cleared by reset().
    static void set_slave_config_failure(unsigned int position);

    /// Run this on every ecrt_master_send(), i.e. once per cycle with the
    /// domain holding exactly what that cycle wrote. It is the only way to see
    /// a one-cycle wire value from a FREE-RUNNING loop: polling the domain from
    /// the test thread both races the writer and misses short pulses. The
    /// observer runs on the worker thread; record into storage the test reads
    /// only after join(), which is the happens-before edge. Cleared by reset().
    static void set_send_observer(std::function<void()> observer);

    /// The fake's single master handle, the same pointer ecrt_request_master()
    /// hands the bus. Lets a test construct an IO device with the live master
    /// the creation contract requires, without recording a request call.
    [[nodiscard]] static ec_master_t* master();

    /// wc_state ecrt_domain_state() reports (default, and after reset():
    /// EC_WC_COMPLETE). Atomic: a test may flip it under a running loop.
    static void set_domain_wc_state(unsigned int wc_state);

    /// Every entry ecrt_domain_reg_pdo_entry_list() registered, in order, as
    /// {position, index, subindex}; the call log keeps only (pos, index).
    /// Cleared by reset().
    [[nodiscard]] static const std::vector<std::array<unsigned int, 3>>& registered_entries();

    /// Make the slave at `position` not map (index, subindex): registering it
    /// (list or single entry) fails with -ENOENT, as the real master does for
    /// an entry outside the slave's mapping. Cleared by reset().
    static void set_unmapped_entry(unsigned int position, unsigned int index, unsigned int subindex);

    /// Model the slave at `position` by `layout`: an entry outside it fails
    /// with -ENOENT; the first entry registered from a sync manager appends
    /// that SM's whole PDO image (every assigned PDO, in order) to the domain,
    /// later entries of the same SM reuse it (one FMMU, no growth) and get
    /// their byte offset inside it. The reuse rule is an assumption carried
    /// over from the IgH contract (ecrt.h, ecrt_slave_config_reg_pdo_entry:
    /// the SM's PDOs are appended "if not already done"), not a measurement.
    /// Byte-aligned entries only (an unaligned one fails with -EINVAL).
    /// Slaves without this knob keep the 8-byte-per-entry grid. `layout` is
    /// borrowed (static test storage), kept until reset(). Offset allocation
    /// belongs to the domain: every ecrt_master_create_domain starts it at 0
    /// with no SM image (one helper clears it, also run by reset()).
    static void set_slave_domain_layout(unsigned int position, const plem::fieldbus::ethercat::SlavePdoMapping& layout);

    /// Domain offset the fake handed out for (position, index, subindex); -1 if never registered.
    [[nodiscard]] static int domain_offset_of(unsigned int position, unsigned int index, unsigned int subindex);

    /// Make ecrt_request_master() return null (the master is held elsewhere or
    /// not running) until reset().
    static void set_request_master_failure();

    /// Make ecrt_slave_config_pdos() of the slave at `position` return -1 (the
    /// master refuses to record the layout) until reset().
    static void set_config_pdos_failure(unsigned int position);

    /// Make ecrt_master_create_domain() return null until reset().
    static void set_create_domain_failure();

    /// Make ecrt_domain_data() return null (no process image) until reset().
    static void set_domain_data_failure();

    /// Model no slave at `position`: ecrt_master_slave_config() still hands
    /// out its config (IgH applies a config once a slave appears there), but
    /// ecrt_master_get_slave() fails (< 0). Cleared by reset().
    static void set_absent_slave(unsigned int position);

    /// Make ecrt_slave_config_dc() of the slave at `position` return `ret`
    /// (< 0) until reset().
    static void set_dc_failure(unsigned int position, int ret);

    /// Make ecrt_master_select_reference_clock() return `ret` (< 0) until reset().
    static void set_reference_clock_failure(int ret);

    /// Make ecrt_master_activate() return `ret` (< 0) until reset().
    static void set_activate_failure(int ret);

    /// Make ecrt_slave_config_sdo8/16/32() of (index, subindex) return `ret`
    /// (< 0) on every slave: a startup SDO the master refuses to reserve.
    /// Cleared by reset().
    static void set_config_sdo_failure(unsigned int index, unsigned int subindex, int ret);

    /// What ecrt_slave_config_state() reports for the slave at `position`:
    /// `al_state` (1 INIT, 2 PREOP, 4 SAFEOP, 8 OP) and `operational` =
    /// al_state OP and no `force_config` pending, as the master derives it;
    /// al_state 0 = a detached slave (online 0, the query still succeeds, as
    /// IgH answers for a slave that is not on the bus).
    /// Default, and after reset(): every slave OP and operational, the
    /// behaviour every other test assumes. Not synchronized: set it while no
    /// RT loop runs (the synchronous harnesses).
    static void set_slave_config_state(unsigned int position, unsigned int al_state, bool force_config = false);

    /// Make ecrt_slave_config_state() of the slave at `position` fail (< 0)
    /// until reset() or the next set_slave_config_state() of it.
    static void set_slave_config_state_failure(unsigned int position);

    /// Sync managers appended to the domain for layout slaves, in order, as
    /// {position, sm_index, output}; under the model above, the set the
    /// domain's FMMUs (and the expected working counter) are built from.
    [[nodiscard]] static const std::vector<std::array<unsigned int, 3>>& domain_sync_managers();
};

}  // namespace plem::fieldbus::test
