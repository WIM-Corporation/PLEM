// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/core/logging/logger.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>

#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace plem::core {

// Async logger constants.
// queue_size: power-of-two ring buffer depth. At worst-case 1kHz logging this
//   provides ~8 seconds of buffering before the oldest lines drop. In RT production,
//   hot loops must use PLEM_EMIT instead of logger, so this buffer only needs
//   to absorb non-RT burst traffic (startup logs, error cascades).
// worker_threads: one worker; stdout I/O is serialized anyway.
constexpr const char* kLoggerName = "plem";
constexpr size_t kAsyncQueueSize = 8192;
constexpr size_t kAsyncWorkerThreads = 1;

namespace {

/// PLEM_LOG_LEVEL, a spdlog level name (also `warn`, `err`); unset or empty is
/// info. Any other value is said once on stderr (the logger is not built yet)
/// and leaves info (`plem drive run` warns about an unknown value first).
spdlog::level::level_enum level_from_env() {
    const char* env = std::getenv("PLEM_LOG_LEVEL");
    if (env == nullptr || *env == '\0') {
        return spdlog::level::info;
    }
    const spdlog::level::level_enum parsed = spdlog::level::from_str(env);
    if (parsed != spdlog::level::off || std::string_view(env) == "off") {  // from_str: unknown is off too
        return parsed;
    }
    std::string names;
    for (int l = spdlog::level::trace; l < spdlog::level::n_levels; ++l) {
        names += (names.empty() ? "" : ", ") +
                 std::string(spdlog::level::to_string_view(static_cast<spdlog::level::level_enum>(l)).data());
    }
    std::fprintf(stderr, "PLEM_LOG_LEVEL='%s' is not a level (%s, warn, err); logging at info\n", env, names.c_str());
    return spdlog::level::info;
}

}  // namespace

Logger::Logger() : pool_(std::make_shared<spdlog::details::thread_pool>(kAsyncQueueSize, kAsyncWorkerThreads)) {
    logger_ =
        std::make_shared<spdlog::async_logger>(kLoggerName, std::make_shared<spdlog::sinks::stdout_color_sink_mt>(),
                                               pool_, spdlog::async_overflow_policy::overrun_oldest);
    logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

    logger_->set_level(level_from_env());
    logger_->flush_on(spdlog::level::err);
    spdlog::register_logger(logger_);
}

Logger::~Logger() {
    // Drain this logger only: flush and leave the registry (a reloaded library
    // registers the name again). The members then release logger_ before pool_,
    // whose destructor joins the worker after the queue empties.
    logger_->flush();
    spdlog::drop(kLoggerName);
    // Lines dropped on a full queue are otherwise invisible; stdout may be the
    // reason they dropped, so report on stderr.
    if (const size_t dropped = pool_->overrun_counter(); dropped > 0) {
        std::fprintf(stderr, "plem: %zu log lines dropped (log queue full)\n", dropped);
    }
}

Logger& Logger::instance() {
    static Logger instance_;
    return instance_;
}

}  // namespace plem::core
