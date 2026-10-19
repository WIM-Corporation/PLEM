// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <spdlog/spdlog.h>
#include <memory>

namespace plem::core {

// Process-wide stdout logger of the plem packages. Asynchronous: a call formats
// the message and enqueues it (it may allocate and takes the queue mutex); one
// worker thread writes stdout. When the queue is full the oldest message is
// dropped, so a caller never waits on stdout; the number dropped is written to
// stderr when the logger is destroyed. RT hot loops use PLEM_EMIT, not
// this logger.
//
// It owns its worker thread and registers one logger named "plem"; spdlog's
// global thread pool and every other logger in the process are left alone.
// Do not log after static destruction has begun (a thread still running at exit).
class PLEM_CORE_PUBLIC Logger {
  public:
    static Logger& instance();

    template <typename... Args>
    void info(fmt::format_string<Args...> fmt, Args&&... args) {
        logger_->info(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void warn(fmt::format_string<Args...> fmt, Args&&... args) {
        logger_->warn(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void error(fmt::format_string<Args...> fmt, Args&&... args) {
        logger_->error(fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void debug(fmt::format_string<Args...> fmt, Args&&... args) {
        logger_->debug(fmt, std::forward<Args>(args)...);
    }

  private:
    Logger();
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    std::shared_ptr<spdlog::details::thread_pool> pool_;  // outlives logger_ (declared first)
    std::shared_ptr<spdlog::logger> logger_;
};

inline Logger& logger = Logger::instance();

}  // namespace plem::core
