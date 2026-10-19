// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file reader.cpp
/// @brief The drive model file's schema layer: YamlNode tree →
/// DriveModelSpec<N>. Field names and the shape are the
/// schema's (cia402-drive-model.md). The rules here are mostly ones only a
/// file can break; the value rules are the value gate's, reused as is. A few
/// value checks are repeated here to word the refusal at the file line
/// (non-zero identity, acceleration_rad_s2 > 0, a blank model name).

#include "loader/reader.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <plem/fieldbus/cia402_profile.hpp>
#include <plem/fieldbus/ethercat/config/configuration.hpp>

namespace plem::drive::detail {
namespace {

using fieldbus::ethercat::PdoEntryMapping;
using Kind = YamlNode::Kind;

/// Channel numbers counted from 1 (od_offset = (channel - 1) * 0x800), up to
/// the CiA 402 logical-device count; whether the drive has that channel is
/// the bus's Step 0 and the bench.
constexpr int64_t kMaxChannel = fieldbus::ethercat::kMaxCia402Channels;

/// Text bytes a model name holds (its array keeps one for the NUL).
constexpr size_t kMaxModelNameBytes = models::kMaxModelNameLength - 1;
constexpr size_t kMaxBusNameBytes = 64;
/// The one key whose absence selects a default (the built-in table):
/// project() reads it, the debug mapping line reports it.
constexpr std::string_view kProfileKey = "cia402_profile";

/// Error code (0x603F) as the generated mapping carries it and as a missing
/// error_code_pdo is read from a mapping (channel-1 base, like the key).
constexpr PdoEntryMapping kStandardErrorCode{0x603f, 0, 16};

/// A file-layer refusal, carrying "<key path> <what> — <how>".
struct FileError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

[[noreturn]] void reject(const std::string& path, const std::string& what) {
    throw FileError((path.empty() ? std::string("the file") : path) + " " + what);
}

std::string child(const std::string& path, std::string_view key) {
    return path.empty() ? std::string(key) : path + "." + std::string(key);
}

std::string item(const std::string& path, size_t index) {
    return path + "[" + std::to_string(index) + "]";
}

bool is_null(const YamlNode& n) {
    return n.kind == Kind::SCALAR && n.plain && n.scalar == "null";
}

std::string got(const YamlNode& n) {
    switch (n.kind) {
        case Kind::MAPPING:
            return "a mapping";
        case Kind::SEQUENCE:
            return "a sequence";
        case Kind::SCALAR:
            break;
    }
    return is_null(n) ? "null" : "'" + n.scalar + "'";
}

bool is_digit(char c) {
    return c >= '0' && c <= '9';
}

/// -?(0|[1-9][0-9]*)(\.[0-9]+)?([eE][-+]?[0-9]+)? (JSON's number, so `010`,
/// `+1.0`, `.5` and `1.` are not numbers).
bool is_real_literal(std::string_view t) {
    size_t i = 0;
    const auto digits = [&] {
        const size_t start = i;
        while (i < t.size() && is_digit(t[i])) {
            ++i;
        }
        return i - start;
    };
    if (i < t.size() && t[i] == '-') {
        ++i;
    }
    const size_t first = i;
    const size_t whole = digits();
    if (whole == 0 || (whole > 1 && t[first] == '0')) {
        return false;
    }
    if (i < t.size() && t[i] == '.') {
        ++i;
        if (digits() == 0) {
            return false;
        }
    }
    if (i < t.size() && (t[i] == 'e' || t[i] == 'E')) {
        ++i;
        if (i < t.size() && (t[i] == '+' || t[i] == '-')) {
            ++i;
        }
        if (digits() == 0) {
            return false;
        }
    }
    return i == t.size();
}

bool is_hex_literal(std::string_view t) {
    return t.size() > 2 && t.starts_with("0x") &&
           t.find_first_not_of("0123456789abcdefABCDEF", 2) == std::string_view::npos;
}

const YamlNode& scalar(const YamlNode& n, const std::string& path) {
    if (n.kind != Kind::SCALAR) {
        reject(path, "must be a single value (got " + got(n) + ")");
    }
    return n;
}

/// Integer grammar: decimal without leading zeros, or 0x hex; a sign
/// only on decimal and only where `min` < 0. Parsed in 64 bits and
/// range-checked before the field's own width; an overflowing literal is
/// refused, never wrapped.
int64_t integer(const YamlNode& n, const std::string& path, int64_t min, int64_t max) {
    const std::string& t = scalar(n, path).scalar;
    if (!n.plain) {
        reject(path, "is a quoted string — write the number unquoted");
    }
    std::string_view digits = t;
    const bool hex = is_hex_literal(t);
    const bool negative = !hex && digits.starts_with('-');
    if (hex) {
        digits.remove_prefix(2);
    } else if (negative) {
        digits.remove_prefix(1);
    }
    const bool decimal = !digits.empty() && digits.find_first_not_of("0123456789") == std::string_view::npos &&
                         (digits.size() == 1 || digits[0] != '0');
    if (!(hex || decimal) || (negative && min >= 0)) {
        reject(path, "('" + t + "') is not " + (min < 0 ? "an integer" : "an unsigned integer") +
                         " — write decimal without leading zeros, or 0x hex (no +, _ or 0X" +
                         (min < 0 ? ", no sign on hex)" : ", no sign)"));
    }
    uint64_t magnitude = 0;
    const auto parsed = std::from_chars(digits.data(), digits.data() + digits.size(), magnitude, hex ? 16 : 10);
    const bool fits =
        parsed.ec == std::errc{} && magnitude <= static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
    // Built only once it fits: negating 2^63 would itself overflow.
    const int64_t value = !fits ? 0 : (negative ? -static_cast<int64_t>(magnitude) : static_cast<int64_t>(magnitude));
    if (!fits || value < min || value > max) {
        reject(path, "(" + t + ") is outside " + std::to_string(min) + ".." + std::to_string(max));
    }
    return value;
}

/// Real grammar: JSON number syntax, then from_chars, which refuses a
/// value past double range (result_out_of_range), so whatever passes is finite.
double real(const YamlNode& n, const std::string& path) {
    const std::string& t = scalar(n, path).scalar;
    if (!n.plain) {
        reject(path, "is a quoted string — write the number unquoted");
    }
    double value = 0.0;
    if (!is_real_literal(t) || std::from_chars(t.data(), t.data() + t.size(), value).ec != std::errc{}) {
        reject(path, "('" + t +
                         "') is not a number this schema reads — write JSON number syntax within double range, like 2, "
                         "-0.5 or 1.5e-3");
    }
    return value;
}

bool boolean(const YamlNode& n, const std::string& path) {
    const std::string& t = scalar(n, path).scalar;
    if (n.plain && (t == "true" || t == "false")) {
        return t == "true";
    }
    reject(path, "('" + t + "') must be true or false, unquoted");
}

/// Text: quoted, or plain unless the loader itself reads it as null, a boolean
/// or a number (such text must be quoted); no ASCII control character (C0 or
/// DEL: a NUL would cut the name short, a line break would split the log line
/// that prints it); at most `max_bytes` UTF-8 bytes (longer is refused, never truncated).
std::string text(const YamlNode& n, const std::string& path, size_t max_bytes) {
    const std::string& t = scalar(n, path).scalar;
    if (n.plain && (t.empty() || t == "~" || t == "null" || t == "true" || t == "false" || is_real_literal(t) ||
                    is_hex_literal(t))) {
        reject(path, "('" + t + "') reads as null, a boolean or a number — quote it to use it as text");
    }
    if (std::any_of(t.begin(), t.end(), [](unsigned char c) { return c < 0x20 || c == 0x7F; })) {
        reject(path,
               "contains an ASCII control character (e.g. a NUL or a line break) — remove it: a NUL would cut the name "
               "short and a line break would split the log line that prints it");
    }
    if (t.size() > max_bytes) {
        reject(path, "is " + std::to_string(t.size()) + " bytes — at most " + std::to_string(max_bytes) +
                         " (UTF-8) fit; shorten it (a name is never truncated)");
    }
    return t;
}

template <typename T>
using Choices = std::initializer_list<std::pair<std::string_view, T>>;

template <typename T>
T choice(const YamlNode& n, const std::string& path, Choices<T> accepted) {
    const std::string& t = scalar(n, path).scalar;
    std::string names;
    for (const auto& [name, value] : accepted) {
        if (t == name) {
            return value;
        }
        names += (names.empty() ? "" : ", ") + std::string(name);
    }
    reject(path, "('" + t + "') is not one of: " + names);
}

const std::vector<YamlNode>& sequence(const YamlNode& n, const std::string& path, size_t capacity,
                                      const char* items_in_owner) {
    if (n.kind != Kind::SEQUENCE) {
        reject(path, "must be a sequence (got " + got(n) + ")");
    }
    if (n.items.size() > capacity) {
        reject(path, "lists " + std::to_string(n.items.size()) + " — at most " + std::to_string(capacity) + " fit " +
                         items_in_owner);
    }
    return n.items;
}

/// One mapping of the file: typed reads by key, then finish() refuses every
/// key nothing read; an unknown key is never ignored (and a misspelt
/// optional key cannot pass for an omitted one).
class Mapping {
  public:
    Mapping(const YamlNode& node, std::string path) : node_(node), path_(std::move(path)) {
        if (node.kind != Kind::MAPPING) {
            reject(path_, "must be a mapping (got " + got(node) + ")");
        }
        read_.assign(node.members.size(), false);
    }

    [[nodiscard]] std::string at(std::string_view key) const {
        return child(path_, key);
    }

    const YamlNode* optional(std::string_view key) {
        for (size_t i = 0; i < node_.members.size(); ++i) {
            if (node_.members[i].first == key) {
                read_[i] = true;
                return &node_.members[i].second;
            }
        }
        return nullptr;
    }

    const YamlNode& required(std::string_view key) {
        const YamlNode* value = optional(key);
        if (value == nullptr) {
            reject(at(key),
                   "is missing — write it (schema §1 lists the keys that may be left out, and the few whose "
                   "absence is written as null or [])");
        }
        return *value;
    }

    Mapping mapping(std::string_view key) {
        return Mapping(required(key), at(key));
    }
    const std::vector<YamlNode>& list(std::string_view key, size_t capacity, const char* items_in_owner) {
        return sequence(required(key), at(key), capacity, items_in_owner);
    }
    template <typename T>
    T uint(std::string_view key) {
        return static_cast<T>(integer(required(key), at(key), 0, std::numeric_limits<T>::max()));
    }
    int64_t integer_in(std::string_view key, int64_t min, int64_t max) {
        return integer(required(key), at(key), min, max);
    }
    double number(std::string_view key) {
        return real(required(key), at(key));
    }
    bool flag(std::string_view key) {
        return boolean(required(key), at(key));
    }
    std::string name(std::string_view key, size_t max_bytes) {
        return text(required(key), at(key), max_bytes);
    }
    template <typename T>
    T pick(std::string_view key, Choices<T> accepted) {
        return choice(required(key), at(key), accepted);
    }
    /// `absent` when the key is left out.
    template <typename T>
    T pick_or(std::string_view key, T absent, Choices<T> accepted) {
        const YamlNode* value = optional(key);
        if (value == nullptr) {
            return absent;
        }
        return choice(*value, at(key), accepted);
    }

    void finish() const {
        for (size_t i = 0; i < read_.size(); ++i) {
            if (!read_[i]) {
                reject(at(node_.members[i].first),
                       "is not a key of this schema — an unknown key is refused, never ignored (check its spelling)");
            }
        }
    }

  private:
    const YamlNode& node_;
    std::string path_;
    std::vector<bool> read_;
};

PdoEntryMapping read_entry(Mapping m) {
    const PdoEntryMapping entry{m.uint<uint16_t>("index"), m.uint<uint8_t>("subindex"), m.uint<uint8_t>("bits")};
    m.finish();
    return entry;
}

models::SlavePdoMappingSpec read_pdo_layout(Mapping m) {
    models::SlavePdoMappingSpec layout{};
    const auto& syncs = m.list("sync_managers", models::SlavePdoMappingSpec::kMaxSyncManagers, "per slave");
    if (syncs.empty()) {
        reject(m.at("sync_managers"),
               "is empty — declare the drive's sync managers as its `ethercat pdos` dump lists them, or write "
               "`pdo_mapping: scanned` to take the mapping the slave powers up with (schema §4.1)");
    }
    layout.sync_count = static_cast<uint8_t>(syncs.size());
    for (size_t s = 0; s < syncs.size(); ++s) {
        Mapping sync(syncs[s], item(m.at("sync_managers"), s));
        auto& out = layout.syncs[s];
        out.sm_index = sync.uint<uint8_t>("sm");
        out.output = sync.pick<bool>("direction", {{"output", true}, {"input", false}});
        out.watchdog = sync.flag("watchdog");
        const auto& pdos = sync.list("pdos", models::SyncManagerAssignmentSpec::kMaxPdos, "per sync manager");
        out.pdo_count = static_cast<uint8_t>(pdos.size());
        for (size_t p = 0; p < pdos.size(); ++p) {
            Mapping pdo(pdos[p], item(sync.at("pdos"), p));
            out.pdos[p].index = pdo.uint<uint16_t>("index");
            const auto& entries = pdo.list("entries", models::PdoMappingSpec::kMaxEntries, "per PDO");
            out.pdos[p].entry_count = static_cast<uint8_t>(entries.size());
            for (size_t e = 0; e < entries.size(); ++e) {
                out.pdos[p].entries[e] = read_entry(Mapping(entries[e], item(pdo.at("entries"), e)));
            }
            pdo.finish();
        }
        sync.finish();
    }
    m.finish();
    return layout;
}

uint32_t identity(Mapping& m, std::string_view key) {
    const auto value = m.uint<uint32_t>(key);
    if (value == 0) {
        reject(m.at(key), "is 0 — a drive slave's identity is non-zero; copy it from `ethercat slaves -v`");
    }
    return value;
}

/// A block a file may leave out (dc, pacing): dropping its key leaves it out,
/// null does not.
Mapping block(const YamlNode& node, const std::string& path) {
    if (is_null(node)) {
        reject(path,
               "is null — null neither declares nor leaves out a block: write its values while its switch is on "
               "(dc: bus.sync_mode dc_sync0; pacing: fault_reset.policy bounded_auto_reset), drop the key while "
               "it is off (schema §1)");
    }
    return Mapping(node, path);
}

/// One `{ index, subindex, size, value }` list of a slave (startup_sdo,
/// stored_sdo); the value gate checks size and value together.
void read_sdo_list(const std::vector<YamlNode>& items, const std::string& path, models::StartupSdoList& list) {
    list.count = items.size();
    for (size_t w = 0; w < items.size(); ++w) {
        Mapping sdo(items[w], item(path, w));
        list.writes[w] = {sdo.uint<uint16_t>("index"), sdo.uint<uint8_t>("subindex"), sdo.uint<uint8_t>("size"),
                          sdo.uint<uint32_t>("value")};
        sdo.finish();
    }
}

/// A slave entry's ring positions: one number, or a list of them for drives
/// that are the same in every other key (schema §4.1); the entry then stands
/// for one slave per position, in list order.
std::vector<int32_t> ring_positions(Mapping& m) {
    constexpr int64_t kMax = std::numeric_limits<int32_t>::max();
    const YamlNode& node = m.required("ring_position");
    if (node.kind != Kind::SEQUENCE) {
        return {static_cast<int32_t>(integer(node, m.at("ring_position"), 0, kMax))};
    }
    if (node.items.size() < 2) {
        reject(m.at("ring_position"), "lists " + std::to_string(node.items.size()) +
                                          " position(s) — a list declares two or more identical drives; write one "
                                          "drive's position as a number");
    }
    std::vector<int32_t> out;
    for (size_t i = 0; i < node.items.size(); ++i) {
        out.push_back(static_cast<int32_t>(integer(node.items[i], item(m.at("ring_position"), i), 0, kMax)));
    }
    return out;
}

/// The three ways a slave's PDO mapping is given (schema §4.1).
enum class PdoPolicy : uint8_t {
    DECLARED,   ///< a mapping: the layout the core writes in PREOP
    SCANNED,    ///< the bus reads the slave's mapping at start
    GENERATED,  ///< the core builds the CiA 402 objects its axes use and writes them in PREOP
};

/// Everything a slave declares except its ring position and axes (the caller
/// walks those into the joint arrays). Returns how its PDO mapping is given;
/// a generated mapping is built once the axes are read (generated_layout).
PdoPolicy read_slave(Mapping& m, bool dc_enable, models::SlaveSpec& slave, models::StartupSdoList& startup,
                     models::StoredSdoList& stored) {
    slave.vendor_id = identity(m, "vendor_id");
    slave.product_code = identity(m, "product_code");
    // A mapping = the declared layout; the plain word `scanned` or `generated`
    // = the other two policies (sync_count stays 0 here). Nothing else (null,
    // a quoted word, another word) declares any.
    PdoPolicy policy = PdoPolicy::DECLARED;
    const YamlNode& layout = m.required("pdo_mapping");
    const bool word = layout.kind == Kind::SCALAR && layout.plain;
    if (layout.kind == Kind::MAPPING) {
        slave.pdo_mapping = read_pdo_layout(Mapping(layout, m.at("pdo_mapping")));
    } else if (word && layout.scalar == "scanned") {
        policy = PdoPolicy::SCANNED;
    } else if (word && layout.scalar == "generated") {
        policy = PdoPolicy::GENERATED;
    } else {
        reject(m.at("pdo_mapping"),
               "(" + got(layout) +
                   ") is none of the three forms: the plain (unquoted) word generated (the core writes the CiA 402 "
                   "objects its axes use in PREOP), the plain word scanned (the bus reads the slave's sync managers "
                   "and PDOs from the master at start and validates them), or a mapping with sync_managers (the "
                   "drive's `ethercat pdos` dump, which the core writes to the slave in PREOP); see schema §4.1");
    }
    if (dc_enable && m.optional("dc") == nullptr) {
        reject(m.at("dc"),
               "is missing — bus.sync_mode: dc_sync0 needs every slave's dc block (assign_activate, "
               "sync0_cycle_ns, sync0_shift_ns); declare it, or set bus.sync_mode: drive_default");
    }
    if (const YamlNode* dc = m.optional("dc")) {
        if (!dc_enable) {
            reject(m.at("dc"),
                   "is declared while bus.sync_mode is drive_default — a switched-off feature takes no sub-block "
                   "(schema §1); "
                   "drop dc, or set bus.sync_mode: dc_sync0");
        }
        Mapping d = block(*dc, m.at("dc"));
        slave.dc_assign_activate = d.uint<uint16_t>("assign_activate");
        slave.dc_sync0_cycle_ns = d.uint<uint32_t>("sync0_cycle_ns");
        slave.dc_sync0_shift_ns = static_cast<int32_t>(
            d.integer_in("sync0_shift_ns", std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()));
        // Optional (schema §5): true = the core writes sync0_cycle_ns to the
        // drive's 0x1C32:02. Absent is the one spelling of "do not write".
        if (const YamlNode* write = d.optional("write_cycle_time")) {
            if (!boolean(*write, d.at("write_cycle_time"))) {
                reject(d.at("write_cycle_time"),
                       "is false — leave the key out for a drive that does not take its cycle time from the master "
                       "(schema §5)");
            }
            slave.dc_write_cycle_time = true;
        }
        d.finish();
    }
    read_sdo_list(m.list("startup_sdo", models::StartupSdoList::kMaxWrites, "per slave"), m.at("startup_sdo"), startup);
    // Optional (schema §4.9): absent = nothing is read. [] is not a second
    // spelling of that.
    if (const YamlNode* values = m.optional("stored_sdo")) {
        const auto& items = sequence(*values, m.at("stored_sdo"), models::StoredSdoList::kMaxWrites, "per slave");
        if (items.empty()) {
            reject(m.at("stored_sdo"), "is empty — leave the key out to compare nothing (schema §4.9)");
        }
        read_sdo_list(items, m.at("stored_sdo"), stored);
    }
    return policy;
}

/// The generated mapping of one slave (schema §4.1): per axis, in channel
/// order, one RxPDO (0x1600 + channel - 1) with Controlword, Modes of
/// operation and the mode's target, and one TxPDO (0x1a00 + channel - 1)
/// with Statusword, Modes of operation display, position, velocity and
/// torque actual values and Error code: the objects the core reads and
/// writes, at the axis's channel offset. SM 2 carries the outputs (watchdog
/// on), SM 3 the inputs.
template <size_t N>
models::SlavePdoMappingSpec generated_layout(const models::DriveModelSpec<N>& spec, size_t slave, size_t axes_read,
                                             const std::string& path) {
    using fieldbus::OperationMode;
    models::SlavePdoMappingSpec layout{};
    layout.sync_count = 2;
    auto& out = layout.syncs[0];
    auto& in = layout.syncs[1];
    out = {.sm_index = 2, .output = true, .watchdog = true};
    in = {.sm_index = 3, .output = false};
    std::vector<const models::AxisSpec*> axes;
    for (size_t a = 0; a < axes_read; ++a) {  // the axes not yet read still hold slave_index 0
        if (spec.axes[a].slave_index == slave) {
            axes.push_back(&spec.axes[a]);
        }
    }
    if (axes.size() > models::SyncManagerAssignmentSpec::kMaxPdos) {
        reject(path, "is generated on a drive with " + std::to_string(axes.size()) + " axes — the generated mapping " +
                         "holds " + std::to_string(models::SyncManagerAssignmentSpec::kMaxPdos) +
                         " (one PDO per axis); declare this drive's mapping instead");
    }
    std::sort(axes.begin(), axes.end(), [](const auto* x, const auto* y) { return x->od_offset < y->od_offset; });
    const auto pdo = [](uint16_t index, uint16_t offset, std::initializer_list<PdoEntryMapping> entries) {
        models::PdoMappingSpec p{.index = index};
        for (const PdoEntryMapping& e : entries) {
            p.entries[p.entry_count++] = {static_cast<uint16_t>(e.index + offset), e.subindex, e.bit_length};
        }
        return p;
    };
    for (const models::AxisSpec* axis : axes) {
        const uint16_t channel = axis->od_offset / 0x800;
        const PdoEntryMapping target =
            axis->modes_of_operation == OperationMode::CYCLIC_SYNC_TORQUE     ? PdoEntryMapping{0x6071, 0, 16}
            : axis->modes_of_operation == OperationMode::CYCLIC_SYNC_VELOCITY ? PdoEntryMapping{0x60ff, 0, 32}
                                                                              : PdoEntryMapping{0x607a, 0, 32};
        out.pdos[out.pdo_count++] =
            pdo(static_cast<uint16_t>(0x1600 + channel), axis->od_offset, {{0x6040, 0, 16}, {0x6060, 0, 8}, target});
        in.pdos[in.pdo_count++] = pdo(
            static_cast<uint16_t>(0x1a00 + channel), axis->od_offset,
            {{0x6041, 0, 16}, {0x6061, 0, 8}, {0x6064, 0, 32}, {0x606c, 0, 32}, {0x6077, 0, 16}, kStandardErrorCode});
    }
    return layout;
}

/// The axis's factor group (CiA 402 0x608F–0x60A9): one gear ratio and one
/// encoder resolution per axis, copied into each of the three conversions the
/// core carries.
fieldbus::ethercat::AxisFactors read_factors(Mapping m) {
    using fieldbus::ethercat::MotionUnit;
    using fieldbus::ethercat::TorqueUnit;
    // The position and velocity unit has one value, so it may be left out,
    // and with it a position block that would hold nothing else.
    const auto motion_unit = [](Mapping& block) {
        const YamlNode* unit = block.optional("unit");
        return unit == nullptr ? MotionUnit::ROTARY_MOTOR_ENCODER
                               : choice<MotionUnit>(*unit, block.at("unit"),
                                                    {{"rotary_motor_encoder", MotionUnit::ROTARY_MOTOR_ENCODER}});
    };
    const double gear_ratio = m.number("gear_ratio");
    const auto encoder_resolution = m.uint<uint32_t>("encoder_resolution");
    fieldbus::ethercat::AxisFactors channels{};
    // The unit picks the torque keys; any other key is refused by finish().
    Mapping torque = m.mapping("torque");
    const auto torque_unit = torque.pick<TorqueUnit>(
        "unit", {{"scaled_current", TorqueUnit::SCALED_CURRENT}, {"rated_per_mille", TorqueUnit::RATED_PER_MILLE}});
    if (torque_unit == TorqueUnit::RATED_PER_MILLE) {
        // ‰ = the scaled_current formula with scale 1000 and Kt = rated torque.
        channels.torque = {torque_unit, fieldbus::ethercat::TorqueChannel::kPerMilleScale,
                           torque.number("motor_rated_torque_nm"), gear_ratio};
    } else {
        channels.torque = {torque_unit, torque.number("counts_per_amp"), torque.number("torque_constant"), gear_ratio};
    }
    torque.finish();
    channels.position = {MotionUnit::ROTARY_MOTOR_ENCODER, encoder_resolution, gear_ratio};
    if (const YamlNode* declared = m.optional("position")) {
        Mapping position(*declared, m.at("position"));
        channels.position.unit = motion_unit(position);
        position.finish();
    }
    Mapping velocity = m.mapping("velocity");
    channels.velocity = {motion_unit(velocity), encoder_resolution, gear_ratio, velocity.number("velocity_factor")};
    velocity.finish();
    m.finish();
    return channels;
}

/// How the axis's slave gives its PDO mapping: what a missing error_code_pdo
/// is read from.
struct SlaveMappingView {
    PdoPolicy policy;
    const models::SlavePdoMappingSpec& layout;
};

/// Whether an input sync manager of `layout` maps (index, subindex).
bool maps(const models::SlavePdoMappingSpec& layout, uint16_t index, uint8_t subindex) {
    for (uint8_t s = 0; s < layout.sync_count; ++s) {
        if (layout.syncs[s].output) {
            continue;
        }
        for (uint8_t p = 0; p < layout.syncs[s].pdo_count; ++p) {
            const auto& pdo = layout.syncs[s].pdos[p];
            if (std::any_of(pdo.entries.begin(), pdo.entries.begin() + pdo.entry_count,
                            [&](const PdoEntryMapping& e) { return e.index == index && e.subindex == subindex; })) {
                return true;
            }
        }
    }
    return false;
}

/// error_code_pdo: null, an entry, or left out; then Error code (0x603F)
/// when the mapping is known here and carries it at the axis's channel.
PdoEntryMapping read_error_code_pdo(Mapping& m, const SlaveMappingView& slave, uint16_t od_offset) {
    const YamlNode* declared = m.optional("error_code_pdo");
    if (declared == nullptr) {
        if (slave.policy == PdoPolicy::GENERATED ||
            (slave.policy == PdoPolicy::DECLARED &&
             maps(slave.layout, static_cast<uint16_t>(kStandardErrorCode.index + od_offset), 0))) {
            return kStandardErrorCode;
        }
        reject(m.at("error_code_pdo"),
               slave.policy == PdoPolicy::SCANNED
                   ? "is missing — a scanned mapping is read only when the bus starts, so name the error code entry "
                     "here ({ index: 0x603f, subindex: 0, bits: 16 } for the standard Error code), or write null "
                     "for none"
                   : "is missing and the declared mapping has no Error code (0x603F) at this axis's channel — write "
                     "the drive's own error code entry, or null for none");
    }
    const YamlNode& node = *declared;
    if (is_null(node)) {
        return {0, 0, 0};
    }
    const PdoEntryMapping role = read_entry(Mapping(node, m.at("error_code_pdo")));
    if (role.index == 0) {
        reject(m.at("error_code_pdo") + ".index",
               "is 0 — 0 means \"no Error code (0x603F) role\", so the declared role would vanish; declare none with "
               "error_code_pdo: null");
    }
    return role;
}

void read_limits(Mapping m, fieldbus::OperationMode mode, models::AxisSafety& safety) {
    // null = an axis without end stops: no travel, held as (-inf, +inf) so the
    // limit checks pass every position (schema §4.4).
    if (const YamlNode& node = m.required("position"); is_null(node)) {
        safety.position_min_rad = -std::numeric_limits<double>::infinity();
        safety.position_max_rad = std::numeric_limits<double>::infinity();
    } else {
        Mapping position(node, m.at("position"));
        safety.position_min_rad = position.number("min_rad");
        safety.position_max_rad = position.number("max_rad");
        position.finish();
    }
    safety.velocity_rad_s = m.number("velocity_rad_s");
    // A cst axis's key only: csp and csv send no torque command to clamp;
    // the drive's own loop sets the torque there (bounded by its 0x6072).
    if (mode == fieldbus::OperationMode::CYCLIC_SYNC_TORQUE) {
        Mapping torque = m.mapping("torque_nm");
        safety.torque_min_nm = torque.number("min");
        safety.torque_max_nm = torque.number("max");
        torque.finish();
        if (m.optional("acceleration_rad_s2") != nullptr) {
            reject(m.at("acceleration_rad_s2"),
                   "is declared on a cst axis — it is the deceleration a csp or csv axis stops at (a cst axis stops "
                   "by its drive's own disable reaction, 0x605C); drop the key");
        }
    } else {
        if (m.optional("torque_nm") != nullptr) {
            reject(m.at("torque_nm"),
                   "is declared on a csp or csv axis — it clamps cst torque commands only (in csp and csv the "
                   "drive's own loop sets the torque, bounded by the drive's 0x6072 Max torque); drop the key");
        }
        if (m.optional("acceleration_rad_s2") == nullptr) {
            reject(m.at("acceleration_rad_s2"),
                   "is missing — a csp or csv axis stops at this deceleration (rad/s²) when a stop is asked for "
                   "(the protective_stop command, the end of a run); set what the axis can brake at, no more than "
                   "the drive's own 0x6085 Quick stop deceleration");
        }
        safety.acceleration_rad_s2 = m.number("acceleration_rad_s2");
        if (!(safety.acceleration_rad_s2 > 0.0)) {
            reject(m.at("acceleration_rad_s2"),
                   "(" + m.required("acceleration_rad_s2").scalar +
                       ") must be > 0 — the deceleration a stop ramps this axis to rest at");
        }
    }
    m.finish();
}

/// Where one drive of a slave entry sits: `copy` of `copies` (a ring_position
/// list makes copies > 1).
struct DriveCopy {
    size_t copy = 0;
    size_t copies = 1;
};

/// The value of a per-drive key (name, flip, zero_offset_count) for one drive:
/// the key itself for a single drive, its `copy`-th item for a ring_position
/// list: one item per drive, never one value shared by all. `path` gets the
/// item's path.
const YamlNode& per_drive(Mapping& m, std::string_view key, const DriveCopy& drive, std::string& path) {
    const YamlNode& node = m.required(key);
    path = m.at(key);
    if (drive.copies == 1) {
        return node;
    }
    if (node.kind != Kind::SEQUENCE || node.items.size() != drive.copies) {
        reject(path, "(" + got(node) + ") must list one value per drive — this slave's ring_position lists " +
                         std::to_string(drive.copies) + " drives, so write [" + std::string(key) + " of the first, …]");
    }
    path = item(path, drive.copy);
    return node.items[drive.copy];
}

void read_axis(Mapping m, size_t slave_index, const DriveCopy& drive, const SlaveMappingView& slave,
               models::AxisSpec& axis, models::AxisSafety& safety,
               std::array<char, models::kMaxAxisNameLength>& label) {
    std::string path;
    const YamlNode& name_node = per_drive(m, "name", drive, path);
    const std::string name = text(name_node, path, models::kMaxAxisNameLength - 1);
    std::memcpy(label.data(), name.data(), name.size());
    axis.slave_index = slave_index;
    // Compared before the multiply, so no literal can wrap onto the grid.
    const int64_t channel = m.integer_in("channel", 1, kMaxChannel);
    axis.od_offset = static_cast<uint16_t>((channel - 1) * 0x800);
    axis.modes_of_operation =
        m.pick<fieldbus::OperationMode>("mode", {{"cst", fieldbus::OperationMode::CYCLIC_SYNC_TORQUE},
                                                 {"csp", fieldbus::OperationMode::CYCLIC_SYNC_POSITION},
                                                 {"csv", fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY}});
    // Left out = pre_enable, the stricter admission (schema §4.2a).
    axis.mode_display_timing =
        m.pick_or<fieldbus::ModeDisplayTiming>("mode_display_timing", fieldbus::ModeDisplayTiming::PRE_ENABLE,
                                               {{"pre_enable", fieldbus::ModeDisplayTiming::PRE_ENABLE},
                                                {"post_enable", fieldbus::ModeDisplayTiming::POST_ENABLE}});
    axis.factors = read_factors(m.mapping("factors"));
    const YamlNode& flip = per_drive(m, "flip", drive, path);
    axis.flip = boolean(flip, path);
    const YamlNode& zero = per_drive(m, "zero_offset_count", drive, path);
    safety.zero_offset_count = static_cast<int32_t>(
        integer(zero, path, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max()));
    axis.error_code_pdo = read_error_code_pdo(m, slave, axis.od_offset);
    read_limits(m.mapping("limits"), axis.modes_of_operation, safety);
    // The retired csp ramp: named, so its refusal says what replaced it.
    if (m.optional("ramp")) {
        reject(m.at("ramp"),
               "is retired — delete the block: a stop decelerates at limits.acceleration_rad_s2, and the client "
               "plans the motion (Bus.move_to in the Python plem.drive, or a FollowJointTrajectory goal to the bus "
               "that plem drive run starts)");
    }
    m.finish();
}

/// A profile state by the core's own name (fieldbus::cia402_state_name),
/// lower-cased; no second name table.
fieldbus::ServoState profile_state(const YamlNode& n, const std::string& path) {
    const std::string& t = scalar(n, path).scalar;
    std::string names;
    for (const fieldbus::ServoState state : fieldbus::kCia402ProfileStates) {
        std::string name = fieldbus::cia402_state_name(state);
        std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::tolower(c); });
        if (t == name) {
            return state;
        }
        names += (names.empty() ? "" : ", ") + name;
    }
    reject(path, "('" + t + "') is not a CiA 402 state — one of: " + names);
}

/// The drive's own transition table. Its rules are the value gate's
/// (validate_cia402_profile); refused here is only what a file alone can get
/// wrong: the key's shape, unknown state names, more rows than fit, and an
/// empty table, whose C++-layer refusal would point a file author at C++.
fieldbus::Cia402TransitionProfile read_profile(const YamlNode& node) {
    const std::string path(kProfileKey);
    if (is_null(node)) {
        reject(path,
               "is null — null does not select the built-in table; leave the key out for that, or list the "
               "drive's rows");
    }
    const auto& rows = sequence(node, path, fieldbus::Cia402TransitionProfile::kMaxTransitions, "in one table");
    if (rows.empty()) {
        reject(path, "is empty — leave the key out to use the core's built-in table, or list the drive's rows");
    }
    fieldbus::Cia402TransitionProfile profile{};
    profile.transition_count = rows.size();
    for (size_t i = 0; i < rows.size(); ++i) {
        Mapping row(rows[i], item(path, i));
        auto& transition = profile.transitions[i];
        transition.from = profile_state(row.required("from"), row.at("from"));
        transition.to = profile_state(row.required("to"), row.at("to"));
        transition.controlword = row.uint<uint16_t>("controlword");
        row.finish();
    }
    return profile;
}

/// Text bytes a 0x1009 / 0x100A string holds (its array keeps one for the
/// NUL; a drive whose string is longer can only be declared null).
constexpr size_t kMaxVersionBytes = fieldbus::ethercat::SlaveIdentity::kVersionLength - 1;
/// Digests, the revision and the bench conditions are free text the record
/// compares or carries (never interprets): a generous cap (over
/// it, text() refuses; nothing is ever cut).
constexpr size_t kMaxRecordTextBytes = 4096;

/// `null` or text of at most kMaxVersionBytes into a fixed array.
std::optional<std::array<char, fieldbus::ethercat::SlaveIdentity::kVersionLength>> nullable_version(
    Mapping& m, std::string_view key) {
    const YamlNode& node = m.required(key);
    if (is_null(node)) {
        return std::nullopt;
    }
    const std::string value = text(node, m.at(key), kMaxVersionBytes);
    std::array<char, fieldbus::ethercat::SlaveIdentity::kVersionLength> out{};
    std::memcpy(out.data(), value.data(), value.size());
    return out;
}

/// `null` or exactly `sha256:` + 64 lowercase hex digits: the bus INFO
/// line's digest of a scanned slave's mapping, copied, never computed here.
std::optional<std::array<char, fieldbus::ethercat::SlaveIdentity::kDigestLength>> nullable_digest(
    Mapping& m, std::string_view key) {
    const YamlNode& node = m.required(key);
    if (is_null(node)) {
        return std::nullopt;
    }
    const std::string value = text(node, m.at(key), fieldbus::ethercat::SlaveIdentity::kDigestLength - 1);  // NUL slot
    constexpr std::string_view kPrefix = "sha256:";
    const bool shaped = value.size() == kPrefix.size() + 64 && value.compare(0, kPrefix.size(), kPrefix) == 0 &&
                        std::all_of(value.begin() + static_cast<std::ptrdiff_t>(kPrefix.size()), value.end(),
                                    [](unsigned char c) { return std::isdigit(c) || (c >= 'a' && c <= 'f'); });
    if (!shaped) {
        reject(m.at(key), "('" + value +
                              "') is not a PDO mapping digest — write `sha256:` + 64 lowercase hex digits exactly as "
                              "the bus INFO line `using the scanned PDO mapping — …` prints it, or null");
    }
    std::array<char, fieldbus::ethercat::SlaveIdentity::kDigestLength> out{};
    std::memcpy(out.data(), value.data(), value.size());
    return out;
}

/// One approval.record.slaves[] entry: the file's slaves[] index and the
/// five observed values, each `null` when the drive gives none.
SlaveIdentityRecord read_slave_identity(Mapping m) {
    SlaveIdentityRecord out;
    out.slave = static_cast<size_t>(m.integer_in("slave", 0, static_cast<int64_t>(kMaxAxes) - 1));
    const YamlNode& serial = m.required("serial_number");
    if (!is_null(serial)) {
        out.identity.serial_number = static_cast<uint32_t>(
            integer(serial, m.at("serial_number"), 0, static_cast<int64_t>(std::numeric_limits<uint32_t>::max())));
    }
    const YamlNode& revision = m.required("revision_number");
    if (!is_null(revision)) {
        out.identity.revision_number = static_cast<uint32_t>(
            integer(revision, m.at("revision_number"), 0, static_cast<int64_t>(std::numeric_limits<uint32_t>::max())));
    }
    out.identity.hardware_version = nullable_version(m, "hardware_version");
    out.identity.software_version = nullable_version(m, "software_version");
    out.identity.pdo_mapping_digest = nullable_digest(m, "pdo_mapping_digest");
    m.finish();
    return out;
}

/// The four claim keys, wherever they sit (a record's head, a bench's
/// `claims` block): shape only; the values are checked by inspect_drive_model
/// (a repeated slave is a reason there: judgment 5 for the record, 7 for the
/// bench, not a file error).
ApprovalClaims read_claims(Mapping& m) {
    ApprovalClaims out;
    out.config_digest = m.name("config_digest", kMaxRecordTextBytes);
    out.profile_digest = m.name("profile_digest", kMaxRecordTextBytes);
    out.implementation_revision = m.name("implementation_revision", kMaxRecordTextBytes);
    if (out.implementation_revision.empty()) {
        reject(m.at("implementation_revision"),
               "is empty — it records the release the bench ran on (provenance, not judged): copy the "
               "`implementation_revision` value `plem drive check --json` printed before the bench");
    }
    const auto& slaves = m.list("slaves", kMaxAxes, "in a claim (one per declared slave)");
    for (size_t i = 0; i < slaves.size(); ++i) {
        out.slaves.push_back(read_slave_identity(Mapping(slaves[i], item(m.at("slaves"), i))));
    }
    return out;
}

/// Free text a bench may leave empty (notes, evidence).
std::string free_text(const YamlNode& node, const std::string& path) {
    return node.kind == Kind::SCALAR && node.scalar.empty() ? std::string() : text(node, path, 65536);
}

/// The bench evidence (schema §6.1: claims, conditions, items), wherever it
/// sits: the standalone file (after its schema line) or approval.record.bench.
BenchFile read_bench(Mapping& m) {
    BenchFile out;
    Mapping claims = m.mapping("claims");
    out.claims = read_claims(claims);
    claims.finish();
    Mapping conditions = m.mapping("conditions");
    out.conditions.host = conditions.name("host", kMaxRecordTextBytes);
    out.conditions.master = conditions.name("master", kMaxRecordTextBytes);
    out.conditions.date = conditions.name("date", kMaxRecordTextBytes);
    out.conditions.measured_by = conditions.name("measured_by", kMaxRecordTextBytes);
    out.conditions.notes = free_text(conditions.required("notes"), conditions.at("notes"));
    conditions.finish();
    const auto& items = m.list("items", 64, "in a bench");
    for (size_t i = 0; i < items.size(); ++i) {
        Mapping row(items[i], item(m.at("items"), i));
        BenchItem entry;
        entry.id = row.name("id", 64);
        entry.result = row.pick<std::string>("result", {{"PASS", "PASS"}, {"FAIL", "FAIL"}, {"NA", "NA"}});
        entry.evidence = free_text(row.required("evidence"), row.at("evidence"));
        row.finish();
        for (const BenchItem& seen : out.items) {
            if (seen.id == entry.id) {
                reject(row.at("id"), "('" + entry.id + "') is listed twice — one row per item");
            }
        }
        out.items.push_back(std::move(entry));
    }
    return out;
}

/// approval.record of a file claiming approved (schema §6): what it binds to,
/// then the bench evidence it was made from.
RecordedApproval read_record(Mapping m) {
    RecordedApproval out;
    out.claims = read_claims(m);
    Mapping bench = m.mapping("bench");
    out.bench = read_bench(bench);
    bench.finish();
    m.finish();
    return out;
}

/// The `approval` block (schema §6): no key = unapproved; unapproved carries
/// a plain `record: null`, approved carries the record the bench's approve step wrote.
std::optional<RecordedApproval> read_approval(Mapping& file) {
    using models::Approval;
    const YamlNode* block = file.optional("approval");
    if (block == nullptr) {
        return std::nullopt;
    }
    Mapping approval(*block, file.at("approval"));
    const auto status =
        approval.pick<Approval>("status", {{"unapproved", Approval::UNAPPROVED}, {"approved", Approval::APPROVED}});
    const YamlNode& record = approval.required("record");
    std::optional<RecordedApproval> out;
    if (status == Approval::UNAPPROVED) {
        if (!is_null(record)) {
            reject(approval.at("record"),
                   "is " + got(record) +
                       " while approval.status is unapproved — an unapproved file carries no record: write "
                       "`record: null` (the plain word); the approved record is written by the bench "
                       "(`plem drive run <file>` on a terminal offers it)");
        }
    } else {
        if (is_null(record)) {
            reject(approval.at("record"),
                   "is null while approval.status is approved — an approved claim needs the record the bench "
                   "wrote (`plem drive run <file>` on a terminal offers it); a file not yet approved says "
                   "`status: unapproved`");
        }
        out = read_record(Mapping(record, approval.at("record")));
    }
    approval.finish();
    return out;
}

template <size_t... I>
DriveModelSpecAny empty_spec(size_t axis_count, std::index_sequence<I...>) {
    DriveModelSpecAny spec;
    (void)((axis_count == I + 1 && (spec.emplace<I>(), true)) || ...);
    return spec;
}

}  // namespace

core::Result<FileProjection> project(const YamlNode& root, std::string_view origin) {
    using Result = core::Result<FileProjection>;
    try {
        FileProjection out;
        Mapping file(root, "");
        const YamlNode& schema = file.required("schema");
        if (schema.kind != Kind::SCALAR || schema.scalar != kDriveModelSchema) {
            reject("schema", "is " + got(schema) + " — this loader reads " + std::string(kDriveModelSchema));
        }

        Mapping model = file.mapping("model");
        const std::string model_name = model.name("name", kMaxModelNameBytes);
        if (model_name.find_first_not_of(' ') == std::string::npos) {
            reject(model.at("name"), "is blank — name the model: it identifies this drive model in every log line");
        }
        model.finish();

        Mapping bus = file.mapping("bus");
        // The period in ns (ETG 0x1C32:02 Cycle time); every consumer counts in
        // us, so a value off the 1 us grid is refused, never rounded.
        const int64_t cycle_time_ns =
            bus.integer_in("cycle_time_ns", 0, int64_t{std::numeric_limits<uint32_t>::max()} * 1000);
        if (cycle_time_ns % 1000 != 0) {
            reject(bus.at("cycle_time_ns"), "(" + std::to_string(cycle_time_ns) +
                                                ") is not a whole number of microseconds — the control loop runs "
                                                "on a 1 us grid; write a multiple of 1000");
        }
        const auto cycle_us = static_cast<uint32_t>(cycle_time_ns / 1000);
        // Value names borrowed from ETG 0x1C32:01 Sync mode (PLEM writes no
        // 0x1C32:01; 0x1C32:02 only for a slave declaring dc.write_cycle_time).
        // dc_sync0 turns on the slave's DC (AssignActivate, SYNC0); drive_default turns
        // nothing on, so the drive's own setting decides.
        const bool dc_enable = bus.pick<bool>("sync_mode", {{"drive_default", false}, {"dc_sync0", true}});
        // The bus's ROS name: one ROS name token (plem drive run starts it as /<name>).
        if (const YamlNode* name = bus.optional("name")) {
            std::string value = text(*name, bus.at("name"), kMaxBusNameBytes);
            const auto word = [](char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '_'; };
            if (value.empty() || std::isdigit(static_cast<unsigned char>(value[0])) ||
                !std::all_of(value.begin(), value.end(), word)) {
                reject(bus.at("name"), "('" + value +
                                           "') is not a ROS name: letters, digits and _, not starting with a digit "
                                           "(e.g. a6b_pair); the bus runs as /<name>");
            }
            out.bus_name = std::move(value);
        }
        bus.finish();

        Mapping fault_reset = file.mapping("fault_reset");
        using fieldbus::ethercat::FaultResetPolicy;
        const auto policy = fault_reset.pick<FaultResetPolicy>(
            "policy", {{"none", FaultResetPolicy::NONE}, {"bounded_auto_reset", FaultResetPolicy::BOUNDED_AUTO_RESET}});
        // The pair is a switch and its block (schema §1); the value gate keeps
        // the same rule for a C++ caller (validate_fault_reset).
        std::optional<fieldbus::ethercat::FaultResetPacing> pacing;
        if (const YamlNode* declared = fault_reset.optional("pacing")) {
            if (policy == FaultResetPolicy::NONE) {
                reject(fault_reset.at("pacing"),
                       "is declared while fault_reset.policy is none — drop pacing, or set policy: "
                       "bounded_auto_reset");
            }
            Mapping p = block(*declared, fault_reset.at("pacing"));
            pacing =
                fieldbus::ethercat::FaultResetPacing{p.uint<uint32_t>("interval_ms"), p.uint<uint32_t>("max_attempts")};
            p.finish();
        } else if (policy == FaultResetPolicy::BOUNDED_AUTO_RESET) {
            reject(fault_reset.at("pacing"),
                   "is missing — fault_reset.policy: bounded_auto_reset paces its resets; declare pacing: { "
                   "interval_ms, max_attempts }, or set policy: none");
        }
        const auto pulse_ms =
            static_cast<uint32_t>(fault_reset.integer_in("pulse_ms", 0, fieldbus::ethercat::kMaxFaultResetPulseMs));
        fault_reset.finish();

        // No key = the core's built-in table, the key = the drive's own rows.
        // One table for every axis.
        const YamlNode* rows = file.optional(kProfileKey);
        const auto profile = rows == nullptr ? fieldbus::kStandardCia402Profile : read_profile(*rows);
        out.profile_source = rows == nullptr ? ProfileSource::BUILT_IN : ProfileSource::INLINE;

        // The claim is read here (shape), checked in inspect_drive_model (values).
        out.record = read_approval(file);

        // Axis count first: an empty `axes` is refused before any slave is
        // written, so every slave hosts an axis and slave_count <= N holds.
        const auto& slaves = file.list("slaves", std::numeric_limits<size_t>::max(), "");
        if (slaves.empty()) {
            reject("slaves", "is empty — declare the drive slaves on this bus and their axes");
        }
        size_t axis_count = 0;
        size_t slave_count = 0;
        std::vector<std::vector<int32_t>> positions;  // per slave entry
        for (size_t s = 0; s < slaves.size(); ++s) {
            Mapping slave(slaves[s], item("slaves", s));
            const auto& axes = slave.list("axes", std::numeric_limits<size_t>::max(), "");
            if (axes.empty()) {
                reject(slave.at("axes"),
                       "is empty — a drive slave hosts at least one axis; declare them, or drop the slave");
            }
            positions.push_back(ring_positions(slave));
            axis_count += axes.size() * positions.back().size();
            slave_count += positions.back().size();
        }
        if (axis_count > kMaxAxes) {
            reject("slaves", "declare " + std::to_string(axis_count) + " axes — a drive bus carries 1.." +
                                 std::to_string(kMaxAxes) + " (the drive layer's joint masks are 8 bits)");
        }

        DriveModelSpecAny spec = empty_spec(axis_count, std::make_index_sequence<kMaxAxes>{});
        std::visit(
            [&](auto& s) {
                size_t axis = 0;
                size_t k = 0;  // the slave index: one per ring position
                for (size_t e = 0; e < slaves.size(); ++e) {
                    const DriveCopy drives{.copies = positions[e].size()};
                    for (DriveCopy drive = drives; drive.copy < drive.copies; ++drive.copy, ++k) {
                        Mapping slave(slaves[e], item("slaves", e));
                        (void)slave.required("ring_position");  // read by ring_positions
                        s.slaves[k].ring_position = positions[e][drive.copy];
                        const PdoPolicy policy =
                            read_slave(slave, dc_enable, s.slaves[k], s.startup_sdo[k], s.stored_sdo[k]);
                        const SlaveMappingView view{policy, s.slaves[k].pdo_mapping};
                        const auto& axes = slave.list("axes", std::numeric_limits<size_t>::max(), "");
                        const std::string listed =
                            drive.copies > 1 ? item(slave.at("ring_position"), drive.copy) : std::string();
                        out.slave_places.push_back({item("slaves", e), listed});
                        for (size_t a = 0; a < axes.size(); ++a, ++axis) {
                            out.axis_places.push_back({item(slave.at("axes"), a), listed});
                            read_axis(Mapping(axes[a], item(slave.at("axes"), a)), k, drive, view, s.axes[axis],
                                      s.safety[axis], s.axis_names[axis]);
                            s.axes[axis].cia402_profile = profile;
                        }
                        if (policy == PdoPolicy::GENERATED) {
                            s.slaves[k].pdo_mapping = generated_layout(s, k, axis, slave.at("pdo_mapping"));
                        }
                        slave.finish();
                    }
                }
                s.slave_count = slave_count;
                s.cycle_time_us = cycle_us;
                s.dc_enable = dc_enable;
                s.fault_reset = policy;
                s.fault_reset_pacing = pacing;
                s.fault_reset_pulse_ms = pulse_ms;
                s.approval = models::Approval::UNAPPROVED;  // until the record is honored
                std::memcpy(s.model_name.data(), model_name.data(), model_name.size());
            },
            spec);
        file.finish();
        out.spec = spec;
        return Result::ok(std::move(out));
    } catch (const FileError& e) {
        return Result::error(std::string(origin) + ": " + e.what());
    }
}

std::string describe_mapping(std::string_view origin, const DriveModelSpecAny& any, ProfileSource source) {
    return std::visit(
        [&](const auto& spec) {
            std::string line = "drive model file " + std::string(origin) + ": model '" + spec.model_name.data() +
                               "', cia402_profile " + (source == ProfileSource::INLINE ? "inline (" : "built-in (") +
                               std::to_string(spec.axes[0].cia402_profile.transition_count) + " rows)";
            size_t within = 0;
            for (size_t i = 0; i < spec.axes.size(); ++i) {
                within = i > 0 && spec.axes[i].slave_index == spec.axes[i - 1].slave_index ? within + 1 : 0;
                line += (i == 0 ? "; axes[" : ", axes[") + std::to_string(i) + "] = slaves[" +
                        std::to_string(spec.axes[i].slave_index) + "].axes[" + std::to_string(within) + "] (" +
                        spec.axis_names[i].data() + ")";
            }
            return line;
        },
        any);
}

core::Result<BenchFile> parse_bench(std::string_view yaml, std::string_view origin) {
    using Result = core::Result<BenchFile>;
    const auto tree = parse_tree(yaml);
    if (!tree) {
        return Result::error(std::string(origin) + ": " + tree.message());
    }
    try {
        Mapping file(tree.value(), "");
        const YamlNode& schema = file.required("schema");
        if (schema.kind != Kind::SCALAR || schema.scalar != kBenchSchema) {
            reject("schema", "is " + got(schema) + " — a bench file says " + std::string(kBenchSchema));
        }
        BenchFile out = read_bench(file);
        file.finish();
        return Result::ok(std::move(out));
    } catch (const FileError& e) {
        return Result::error(std::string(origin) + ": " + e.what());
    }
}

}  // namespace plem::drive::detail
