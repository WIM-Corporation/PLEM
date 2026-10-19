// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file approve.cpp
/// @brief `approve` (the bench's approval step): the approval record, the bench evidence embedded,
/// written into the drive model file atomically; see drive_model_tool.hpp.

#include "tool/drive_model_tool.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include <fmt/format.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "loader/inspection.hpp"
#include "loader/reader.hpp"

namespace plem::drive::tool {

using detail::ApprovalClaims;
using detail::BenchConditions;
using detail::BenchFile;
using detail::BenchItem;
using detail::DriveModelInspection;
using detail::FileProjection;
using detail::inspect_drive_model;
using detail::kMaxDriveModelFileBytes;
using detail::parse_bench;
using detail::parse_tree;
using detail::project;
using detail::read_text_file;
using detail::SlaveIdentityRecord;
namespace {

using models::Approval;

/// Where approve writes the record: the top-level `approval:` line and the
/// indented lines under it, blank lines between them included (the whole
/// block, whatever it claims; a line of only spaces or tabs is blank), or (no such line) the end of the file. `nl` is
/// the file's line ending, `indent` the block's (two spaces when it has none).
struct RecordSpot {
    size_t begin = 0;
    size_t end = 0;  // past the block's last character, its line ending excluded
    bool append = false;
    std::string nl = "\n";
    std::string indent = "  ";
};

RecordSpot record_spot(const std::string& text) {
    RecordSpot spot;
    if (const size_t lf = text.find('\n'); lf != std::string::npos && lf > 0 && text[lf - 1] == '\r') {
        spot.nl = "\r\n";
    }
    size_t begin = text.rfind("approval:", 0) == 0 ? 0 : text.find("\napproval:");
    if (begin == std::string::npos) {
        spot.begin = spot.end = text.size();
        spot.append = true;
        return spot;
    }
    if (text[begin] == '\n') {
        ++begin;
    }
    spot.begin = begin;
    size_t eol = text.find('\n', begin);
    size_t last = eol;  // the line ending of the block's last indented line
    bool first = true;
    while (eol != std::string::npos && eol + 1 < text.size()) {
        const size_t start = eol + 1;
        const size_t next = text.find('\n', start);
        const size_t content = text.find_first_not_of(" \t\r", start);
        if (content == std::string::npos || content >= next) {
            eol = next;  // a blank (or whitespace-only) line: inside only if an indented line follows
        } else if (text[start] == ' ' || text[start] == '\t') {
            if (first) {
                spot.indent = text.substr(start, text.find_first_not_of(" \t", start) - start);
                first = false;
            }
            last = eol = next;
        } else {
            break;
        }
    }
    spot.end = last == std::string::npos ? text.size() : last;
    if (spot.end > spot.begin && text[spot.end - 1] == '\r') {
        --spot.end;
    }
    return spot;
}

/// `text` with `block` (no final line ending) at `spot`.
std::string with_block(const std::string& text, const RecordSpot& spot, const std::string& block) {
    std::string put = block;
    if (spot.append) {
        put = (text.empty() || text.back() == '\n' ? "" : spot.nl) + block + spot.nl;
    }
    return std::string(text).replace(spot.begin, spot.end - spot.begin, put);
}

/// A YAML double-quoted scalar that reads back as `text`. The loader's text
/// has no C0 control or DEL (it refuses them); what else a double-quoted
/// scalar cannot hold raw is escaped: `"`, `\`, the C1 range U+0080–U+009F
/// (not printable YAML; U+0085 is also a line break), the line and
/// paragraph separators U+2028/U+2029 (line breaks a reader folds to a space)
/// and the noncharacters U+FFFE/U+FFFF (not printable YAML).
std::string yaml_quoted(std::string_view text) {
    std::string out = "\"";
    for (size_t i = 0; i < text.size(); ++i) {
        const auto c = static_cast<unsigned char>(text[i]);
        const auto next = [&](size_t k) { return i + k < text.size() ? static_cast<unsigned char>(text[i + k]) : 0; };
        if (c == '"' || c == '\\') {
            out += '\\';
            out += static_cast<char>(c);
        } else if (c == 0xc2 && next(1) >= 0x80 && next(1) <= 0x9f) {
            out += fmt::format("\\x{:02x}", next(1));
            i += 1;
        } else if (c == 0xe2 && next(1) == 0x80 && (next(2) == 0xa8 || next(2) == 0xa9)) {
            out += next(2) == 0xa8 ? "\\L" : "\\P";
            i += 2;
        } else if (c == 0xef && next(1) == 0xbf && (next(2) == 0xbe || next(2) == 0xbf)) {
            out += next(2) == 0xbe ? "\\uFFFE" : "\\uFFFF";
            i += 2;
        } else {
            out += static_cast<char>(c);
        }
    }
    return out + "\"";
}

/// `0x` + 8 hex digits, how the master and `ethercat slaves -v` print an SII
/// revision number (the loader reads decimal or `0x`).
std::string hex(uint32_t value) {
    return fmt::format("0x{:08x}", value);
}

// NOLINTBEGIN(performance-inefficient-string-concatenation): runs once per approval, and one
// expression per YAML line reads as the line it writes.
/// One claims block (a record's head or its bench's `claims`), `at` deep.
std::string claims_lines(const std::string& nl, const std::string& at, const std::string& step,
                         const ApprovalClaims& claims) {
    std::string block = at + "config_digest: " + yaml_quoted(claims.config_digest) + nl;
    block += at + "profile_digest: " + yaml_quoted(claims.profile_digest) + nl;
    block += at + "implementation_revision: " + yaml_quoted(claims.implementation_revision) + nl;
    block += at + "slaves:" + (claims.slaves.empty() ? " []" : "") + nl;
    for (const SlaveIdentityRecord& s : claims.slaves) {
        // A fixed-array text field (version or digest): its text, or `null`.
        const auto text = [](const auto& v) {
            return v ? yaml_quoted(std::string_view(v->data(), strnlen(v->data(), v->size()))) : std::string("null");
        };
        block += at + step + "- { slave: " + std::to_string(s.slave) +
                 ", serial_number: " + (s.identity.serial_number ? std::to_string(*s.identity.serial_number) : "null") +
                 ", revision_number: " + (s.identity.revision_number ? hex(*s.identity.revision_number) : "null") +
                 ", hardware_version: " + text(s.identity.hardware_version) +
                 ", software_version: " + text(s.identity.software_version) +
                 ", pdo_mapping_digest: " + text(s.identity.pdo_mapping_digest) + " }" + nl;
    }
    return block;
}

/// approval.record in the shape the loader reads (schema §6), `indent` deep,
/// `nl` the file's line ending: the bench's claims as the record's head, then
/// the whole bench evidence (claims, every condition, every item).
std::string approved_block(const std::string& nl, const std::string& indent, const BenchFile& bench) {
    const std::string i2 = indent + indent;
    const std::string i3 = i2 + indent;
    const std::string i4 = i3 + indent;
    const BenchConditions& c = bench.conditions;
    std::string block = "approval:" + nl + indent + "status: approved" + nl + indent + "record:" + nl;
    block += claims_lines(nl, i2, indent, bench.claims);
    block += i2 + "bench:" + nl;
    block += i3 + "claims:" + nl + claims_lines(nl, i4, indent, bench.claims);
    block += i3 + "conditions: { host: " + yaml_quoted(c.host) + ", master: " + yaml_quoted(c.master) +
             ", date: " + yaml_quoted(c.date) + ", measured_by: " + yaml_quoted(c.measured_by) +
             ", notes: " + yaml_quoted(c.notes) + " }" + nl;
    block += i3 + "items:";
    for (const BenchItem& item : bench.items) {
        block += nl + i3 + indent + "- { id: " + yaml_quoted(item.id) + ", result: " + item.result +
                 ", evidence: " + yaml_quoted(item.evidence) + " }";
    }
    return block + (bench.items.empty() ? " []" : "");  // no final line ending: with_block's
}
// NOLINTEND(performance-inefficient-string-concatenation)

constexpr std::string_view kApproveUsage =
    "usage: drive_model_tool approve <model.yaml> <bench.yaml> — the bench's approval step (`plem drive run` offers "
    "the bench), not a command to run by hand\n";

}  // namespace

/// Why approve refuses this file before it looks at a bench; empty = it
/// would not. One check: `check` prints it as approve_blocker (before a
/// bench releases any brake), approve as its refusal. The file has loaded,
/// so it has at most one approval key; when record_spot misses that block (a
/// quoted key, a flow mapping), the trial rewrite holds the key twice and the
/// parse refuses it.
std::string approve_blocker(const std::string& text, std::string_view origin) {
    const RecordSpot spot = record_spot(text);
    const std::string trial =
        with_block(text, spot,
                   "approval:" + spot.nl + spot.indent + "status: unapproved" + spot.nl + spot.indent + "record: null");
    const auto tree = parse_tree(trial);
    const auto projected = tree ? project(tree.value(), origin) : core::Result<FileProjection>::error(tree.message());
    if (projected) {
        return {};
    }
    return "approve cannot find its approval block to rewrite — write it as an `approval:` line at the start of "
           "a line with its keys indented under it, or leave the key out (" +
           projected.message() + ")";
}

namespace {
/// `path` (a symlink: its target) replaced by `text` through a temporary file
/// in the same directory, with the file's permission bits, then the directory
/// synced; ok = done, else an error saying why not (the file is unchanged and
/// no temporary file is left). Refused when the file is not the caller's or
/// not writable by it (a replacement would change its owner or lift a
/// read-only mark).
core::Result<void> replace_file(const std::string& path, const std::string& text) {
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path target = fs::canonical(path, ec);
    struct stat now {};
    if (ec || ::stat(target.c_str(), &now) != 0) {
        return core::Result<void>::error(ec ? ec.message() : std::strerror(errno));
    }
    if (now.st_uid != ::geteuid()) {
        return core::Result<void>::error("it is owned by uid " + std::to_string(now.st_uid) + ", not by you (uid " +
                                         std::to_string(::geteuid()) +
                                         ") — replacing it would make it yours; approve it as its owner");
    }
    if (::access(target.c_str(), W_OK) != 0) {
        return core::Result<void>::error(
            "it is not writable by you — replacing it would lift its read-only mark; make it writable (chmod "
            "u+w) to approve it");
    }
    std::string temp = (target.parent_path() / ("." + target.filename().string() + ".XXXXXX")).string();
    const int fd = ::mkstemp(temp.data());
    if (fd < 0) {
        return core::Result<void>::error(std::strerror(errno));
    }
    int error = 0;
    for (size_t done = 0; done < text.size() && error == 0;) {
        const ssize_t n = ::write(fd, text.data() + done, text.size() - done);
        if (n > 0) {
            done += static_cast<size_t>(n);
        } else if (n == 0 || errno != EINTR) {
            error = n == 0 ? EIO : errno;
        }
    }
    if (error == 0 && (::fchmod(fd, now.st_mode & 07777) != 0 || ::fsync(fd) != 0)) {
        error = errno;
    }
    if (::close(fd) != 0 && error == 0) {
        error = errno;
    }
    if (error == 0 && ::rename(temp.c_str(), target.c_str()) != 0) {
        error = errno;
    }
    if (error != 0) {
        ::unlink(temp.c_str());
        return core::Result<void>::error(std::strerror(error));
    }
    // The rename itself survives a power cut once the directory is synced (best effort: the file is in place).
    if (const int dir = ::open(target.parent_path().c_str(), O_RDONLY | O_DIRECTORY); dir >= 0) {
        (void)::fsync(dir);
        ::close(dir);
    }
    return core::Result<void>::ok();
}
}  // namespace

int tool_approve(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
    if (wants_help(args)) {
        out << kApproveUsage;
        return kToolOk;
    }
    if (args.size() != 2) {
        err << kApproveUsage;
        return kToolUsage;
    }
    const std::string& model_path = args[0];
    const std::string& bench_path = args[1];
    // 1. The model file, as the bus would read it, any claim (a
    //    re-bench replaces an approved block).
    const auto text = read_text_file(model_path, "drive model file");
    if (!text) {
        err << text.message() << "\n";
        return kToolRefused;
    }
    const auto inspected = inspect_drive_model(text.value(), model_path);
    if (!inspected) {
        err << inspected.message() << "\n";
        return kToolRefused;
    }
    const DriveModelInspection& in = inspected.value();
    if (const std::string blocker = approve_blocker(text.value(), model_path); !blocker.empty()) {
        err << "drive_model_tool approve: " << model_path << ": " << blocker << "\n";
        return kToolRefused;
    }

    // 2. The bench file. Whether it claims this software's digests of this
    //    file as it is now (judgments 2-3; the record's head is the bench's
    //    claims; a file edited during the bench is refused) and reports every
    //    required item PASS (judgment 8) is the loader's check in step 4,
    //    not repeated here.
    const auto bench_text = read_text_file(bench_path, "bench file");
    if (!bench_text) {
        err << bench_text.message() << "\n";
        return kToolRefused;
    }
    const auto bench = parse_bench(bench_text.value(), bench_path);
    if (!bench) {
        err << bench.message() << "\n";
        return kToolRefused;
    }
    const BenchFile& b = bench.value();

    // 3. The record: the bench's claims as its head, the bench evidence whole.
    const RecordSpot spot = record_spot(text.value());
    const std::string approved = with_block(text.value(), spot, approved_block(spot.nl, spot.indent, b));

    // 4. Checked the way the node will read it before anything is written.
    //    It refuses a bench claiming other digests (judgments 2-3), a failed
    //    or missing required item (8), a bench whose slaves do not cover the
    //    file's or list one twice (5), and (checked first) an approval the
    //    loader's size cap would refuse (two inputs each under
    //    the cap can sum over it; inspect_drive_model reads text, not a file).
    if (approved.size() > kMaxDriveModelFileBytes) {
        err << "drive_model_tool approve: the approved file would be " << approved.size() << " bytes, over the "
            << kMaxDriveModelFileBytes << " a drive model file may hold — shorten the bench's notes and evidence\n";
        return kToolRefused;
    }
    const auto back = inspect_drive_model(approved, model_path);
    if (!back || back.value().effective != Approval::APPROVED) {
        err << "drive_model_tool approve: the record this bench yields is not honored for " << model_path << " — "
            << (back ? (back.value().reasons.empty() ? std::string("(no reason)") : back.value().reasons.front())
                     : back.message())
            << "\n";
        return kToolRefused;
    }

    // 5. Into the file itself, atomically: a temporary file next to it (its
    //    permission bits), then a rename; the file is either as it was or
    //    approved, never torn. A symlink's resolved target is replaced; the
    //    symlink remains. A file the caller does not own or cannot write is
    //    refused.
    if (const core::Result<void> why = replace_file(model_path, approved); !why) {
        err << "drive_model_tool approve: cannot write " << model_path << " (" << why.message()
            << ") — the file is unchanged\n";
        return kToolRefused;
    }
    out << "approved: " << model_path << "\n";
    out << "config_digest: " << in.digests.config << "\n";
    out << "profile_digest: " << in.digests.profile << "\n";
    out << "implementation_revision: " << b.claims.implementation_revision << "\n";
    out << "identity_scope: " << name_of(back.value().identity_scope) << "\n";
    return kToolOk;
}

}  // namespace plem::drive::tool
