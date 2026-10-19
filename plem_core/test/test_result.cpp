// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <plem/core/result.hpp>

#include <stdexcept>
#include <string>
#include <utility>

using namespace plem::core;

// Result<T> with Value Type Tests

class ResultWithValueTest : public ::testing::Test {};

TEST_F(ResultWithValueTest, OkCreatesSuccessResult) {
    auto result = Result<int>::ok(42);

    EXPECT_TRUE(result.is_ok());
    EXPECT_EQ(result.value(), 42);
}

TEST_F(ResultWithValueTest, ErrorCreatesFailureResult) {
    auto result = Result<int>::error("Value must be positive");

    EXPECT_FALSE(result.is_ok());
    EXPECT_EQ(result.message(), "Value must be positive");
}

TEST_F(ResultWithValueTest, ExplicitBoolConversionWorks) {
    auto ok_result = Result<int>::ok(42);
    auto error_result = Result<int>::error("error");

    EXPECT_TRUE(static_cast<bool>(ok_result));
    EXPECT_FALSE(static_cast<bool>(error_result));
}

TEST_F(ResultWithValueTest, ValueAccessWorks) {
    auto result = Result<std::string>::ok("hello");

    const auto& const_ref = result.value();
    EXPECT_EQ(const_ref, "hello");

    auto moved = std::move(result).value();
    EXPECT_EQ(moved, "hello");
}

// Result<void> Specialization Tests

class ResultVoidTest : public ::testing::Test {};

TEST_F(ResultVoidTest, OkCreatesSuccessResult) {
    auto result = Result<void>::ok();

    EXPECT_TRUE(result.is_ok());
}

TEST_F(ResultVoidTest, ErrorCreatesFailureResult) {
    auto result = Result<void>::error("Device not found");

    EXPECT_FALSE(result.is_ok());
    EXPECT_EQ(result.message(), "Device not found");
}

TEST_F(ResultVoidTest, ExplicitBoolConversionWorks) {
    auto ok_result = Result<void>::ok();
    auto error_result = Result<void>::error("error");

    EXPECT_TRUE(static_cast<bool>(ok_result));
    EXPECT_FALSE(static_cast<bool>(error_result));
}

// value() on an error result throws in every build (dev builds are
// RelWithDebInfo, i.e. NDEBUG, where an assert would read a default value),
// naming the error it was holding. An error built with an empty message is
// still a failure and reads as one: the message becomes "(no reason given)".

TEST(ResultMisuseTest, ValueOnAnErrorResultThrowsNamingTheError) {
    auto result = Result<int>::error("boom");
    try {
        (void)result.value();
        FAIL() << "value() on an error result must throw";
    } catch (const std::logic_error& e) {
        EXPECT_STREQ(e.what(), "Result::value() called on an error result: boom");
    }
}

TEST(ResultMisuseTest, MovedValueOnAnErrorResultThrowsNamingTheError) {
    try {
        (void)Result<int>::error("boom").value();
        FAIL() << "value() && on an error result must throw";
    } catch (const std::logic_error& e) {
        EXPECT_STREQ(e.what(), "Result::value() called on an error result: boom");
    }
}

TEST(ResultMisuseTest, AnErrorWithoutAMessageIsStillAFailureThatSaysSo) {
    const auto with_value = Result<int>::error("");
    EXPECT_FALSE(with_value.is_ok());
    EXPECT_EQ(with_value.message(), "(no reason given)");
    const auto without_value = Result<void>::error("");
    EXPECT_FALSE(without_value.is_ok());
    EXPECT_EQ(without_value.message(), "(no reason given)");
}
