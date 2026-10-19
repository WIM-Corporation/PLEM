// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cstdlib>

#include <plem/core/logging/logger.hpp>

#include <spdlog/async.h>
#include <spdlog/sinks/null_sink.h>
#include <spdlog/spdlog.h>

// The plem logger is loaded into processes it does not own (a ros2_control
// plugin, a host application). It must leave that process's spdlog state alone:
// the global registry names and the global async thread pool.
TEST(Logger, LeavesTheHostProcessSpdlogStateAlone) {
    auto app = std::make_shared<spdlog::logger>("console", std::make_shared<spdlog::sinks::null_sink_mt>());
    spdlog::register_logger(app);

    ASSERT_NO_THROW(plem::core::Logger::instance().info("logger test"));

    EXPECT_EQ(spdlog::get("console"), app);
    EXPECT_EQ(spdlog::thread_pool(), nullptr);
}

// A level PLEM_LOG_LEVEL does not name is said once on stderr, and the logger
// stays at info. A fresh process: the logger reads the variable when built.
TEST(LoggerDeathTest, AnUnknownLevelIsSaidOnceAndLeavesInfo) {
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    ASSERT_EQ(setenv("PLEM_LOG_LEVEL", "verbose", 1), 0);
    EXPECT_EXIT(
        (plem::core::Logger::instance(), std::exit(static_cast<int>(spdlog::get("plem")->level()))),
        ::testing::ExitedWithCode(static_cast<int>(spdlog::level::info)),
        "PLEM_LOG_LEVEL='verbose' is not a level \\(trace, debug, info, warning, error, critical, off, warn, err\\); "
        "logging at info");
    ASSERT_EQ(unsetenv("PLEM_LOG_LEVEL"), 0);
}
