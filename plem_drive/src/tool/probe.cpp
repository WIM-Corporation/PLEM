// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file probe.cpp
/// @brief `probe` (run's check before it starts an approved file): the bus's own start-up check
/// of the drives on a master, nothing configured; see drive_model_tool.hpp.

#include "tool/drive_model_tool.hpp"

#include <charconv>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "bus_from_spec.hpp"

namespace plem::drive::tool {

using detail::prepare_drive_bus;

namespace {

constexpr std::string_view kProbeUsage =
    "usage: drive_model_tool probe <model.yaml> --master N — the check `plem drive run` makes of the drives before "
    "it starts an approved file, not a command to run by hand\n";

}  // namespace

int tool_probe(const std::vector<std::string>& args, std::ostream& /*out*/, std::ostream& err) {
    std::optional<std::string> file;
    std::optional<unsigned int> master;
    for (size_t i = 0; i < args.size(); ++i) {
        if (args[i] == "--master") {
            unsigned int index = 0;
            const std::string value = i + 1 < args.size() ? args[++i] : "";
            const char* end = value.data() + value.size();
            // Every character a digit and the value in range (an empty one is invalid_argument).
            const auto [ptr, ec] = std::from_chars(value.data(), end, index);
            if (ec != std::errc() || ptr != end) {
                err << kProbeUsage;
                return kToolUsage;
            }
            master = index;
        } else if (!file) {
            file = args[i];
        } else {
            err << kProbeUsage;
            return kToolUsage;
        }
    }
    if (!file || !master) {
        err << kProbeUsage;
        return kToolUsage;
    }
    const auto loaded = load_drive_model_file(*file);
    if (!loaded) {
        err << loaded.message() << "\n";
        return kToolRefused;
    }
    return std::visit(
        [&](const auto& spec) {
            try {
                auto bus = std::make_shared<fieldbus::ethercat::EtherCATBus>(*master);
                if (const core::Result<void> prepared = prepare_drive_bus(spec, bus.get()); !prepared) {
                    err << prepared.message() << "\n";
                    return kToolRefused;
                }
                bus->set_probe(true);
                if (const core::Result<void> probed = bus->initialize(); !probed) {
                    err << probed.message() << "\n";
                    return bus->identity_rebenchable() ? kToolRebench : kToolRefused;
                }
                return kToolOk;
            } catch (const std::exception& e) {
                err << e.what() << "\n";
                return kToolRefused;
            }
        },
        loaded.value());
}

}  // namespace plem::drive::tool
