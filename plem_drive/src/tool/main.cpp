// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file main.cpp
/// @brief `drive_model_tool`: the drive model file tool behind `plem drive
/// check`, the bench's approve step and run's probe (`drive_model_tool <verb>`).
/// Dispatch only; the verbs live in
/// tool/drive_model_tool.hpp.
///
/// stdout is the machine-read contract and nothing else. The core logger's
/// sink is stdout too (the loader logs its mapping line through it at debug
/// level, and the bus logs while probe runs), so
/// main hands the process's fd 1 to stderr first and writes the contract to
/// the original fd once the verb returns.

#include <cerrno>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <unistd.h>

#include "tool/drive_model_tool.hpp"

namespace {

using namespace plem::drive::tool;

int dispatch(int argc, char** argv, std::ostream& out, std::ostream& err) {
    if (argc < 2) {
        err << tool_usage();
        return kToolUsage;
    }
    const std::string verb = argv[1];
    const std::vector<std::string> args(argv + 2, argv + argc);
    if (verb == "check") {
        return tool_check(args, out, err);
    }
    if (verb == "approve") {
        return tool_approve(args, out, err);
    }
    if (verb == "probe") {
        return tool_probe(args, out, err);
    }
    if (verb == "--help" || verb == "-h" || verb == "help") {
        out << tool_usage();
        return kToolOk;
    }
    err << "plem drive: unknown verb '" << verb << "'\n" << tool_usage();
    return kToolUsage;
}

}  // namespace

int main(int argc, char** argv) {
    const int contract_fd = dup(STDOUT_FILENO);
    if (contract_fd < 0 || dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
        const int dup_error = errno;  // the stream inserts below may clobber errno
        std::cerr << "plem drive: cannot keep the logger off stdout (" << std::strerror(dup_error) << ")\n";
        return kToolRefused;
    }
    std::ostringstream contract;
    const int rc = dispatch(argc, argv, contract, std::cerr);
    const std::string text = contract.str();
    for (size_t done = 0; done < text.size();) {
        const ssize_t n = write(contract_fd, text.data() + done, text.size() - done);
        if (n <= 0) {
            std::cerr << "plem drive: cannot write to stdout\n";
            return kToolRefused;
        }
        done += static_cast<size_t>(n);
    }
    return rc;
}
