// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace plem::core {

namespace detail {
/// Tag type to disambiguate error constructor
struct ErrorTag {};

/// What error("") stores, so an empty message still reads as a failure in a log
/// or a message() == "" check. It remains a failure and does not abort.
inline constexpr const char* kNoReasonGiven = "(no reason given)";
}  // namespace detail

/**
 * @brief Result type for consistent error handling
 *
 * Provides a type-safe way to return either a value or an error from functions.
 * Simplified design: success/failure with message, no error codes.
 *
 * Usage with value type:
 * @code
 * Result<int> divide(int a, int b) {
 *     if (b == 0) {
 *         return Result<int>::error("Division by zero");
 *     }
 *     return Result<int>::ok(a / b);
 * }
 *
 * // Check and use
 * if (auto result = divide(10, 2)) {
 *     std::cout << "Result: " << result.value() << std::endl;
 * }
 * @endcode
 *
 * @tparam T The value type on success; must be default-constructible (an error
 *           Result value-initializes it). Use void for operations that don't
 *           return a value.
 */
template <typename T = void>
class [[nodiscard]] Result {
  public:
    /**
     * @brief Create a successful result with a value
     * @param value The success value
     * @return Result containing the value
     */
    [[nodiscard]] static Result ok(T value) {
        return Result(std::move(value));
    }

    /**
     * @brief Create a failed result
     * @param message Human-readable error message; an empty one is stored as
     *        "(no reason given)".
     * @return Result containing the error
     */
    [[nodiscard]] static Result error(std::string message) {
        if (message.empty()) {
            message = detail::kNoReasonGiven;
        }
        return Result(detail::ErrorTag{}, std::move(message));
    }

    /**
     * @brief Check if the result represents success
     * @return true if successful, false if error
     */
    [[nodiscard]] bool is_ok() const noexcept {
        return success_;
    }

    /**
     * @brief Explicit bool conversion for use in if statements
     * @return true if successful, false if error
     */
    [[nodiscard]] explicit operator bool() const noexcept {
        return success_;
    }

    /**
     * @brief Access the contained value (const reference)
     * @return Const reference to the value
     * @throws std::logic_error on an error result ("Result::value() called on an
     *         error result: <message>"), in every build; callers check is_ok() first
     */
    [[nodiscard]] const T& value() const& {
        if (!success_) {
            throw std::logic_error("Result::value() called on an error result: " + message_);
        }
        return value_;
    }

    /**
     * @brief Access the contained value (rvalue reference for moving)
     * @return Rvalue reference to the value
     * @throws std::logic_error on an error result ("Result::value() called on an
     *         error result: <message>"), in every build; callers check is_ok() first
     */
    [[nodiscard]] T&& value() && {
        if (!success_) {
            throw std::logic_error("Result::value() called on an error result: " + message_);
        }
        return std::move(value_);
    }

    /**
     * @brief Get the error message
     * @return The error message (empty if successful)
     */
    [[nodiscard]] const std::string& message() const noexcept {
        return message_;
    }

  private:
    // Success constructor
    explicit Result(T value) : success_(true), message_(), value_(std::move(value)) {}

    // Error constructor (uses tag to disambiguate from success when T is std::string)
    Result(detail::ErrorTag, std::string message) : success_(false), message_(std::move(message)), value_() {}

    bool success_ = false;
    std::string message_;
    T value_;
};

/**
 * @brief Specialization of Result for void (no value)
 *
 * Used for operations that only need to report success/failure without
 * returning a value.
 */
template <>
class [[nodiscard]] Result<void> {
  public:
    /**
     * @brief Create a successful result
     * @return Successful Result<void>
     */
    [[nodiscard]] static Result ok() {
        return Result();
    }

    /**
     * @brief Create a failed result
     * @param message Human-readable error message; an empty one is stored as
     *        "(no reason given)".
     * @return Result containing the error
     */
    [[nodiscard]] static Result error(std::string message) {
        if (message.empty()) {
            message = detail::kNoReasonGiven;
        }
        return Result(std::move(message));
    }

    /**
     * @brief Check if the result represents success
     * @return true if successful, false if error
     */
    [[nodiscard]] bool is_ok() const noexcept {
        return success_;
    }

    /**
     * @brief Explicit bool conversion for use in if statements
     * @return true if successful, false if error
     */
    [[nodiscard]] explicit operator bool() const noexcept {
        return success_;
    }

    /**
     * @brief Get the error message
     * @return The error message (empty if successful)
     */
    [[nodiscard]] const std::string& message() const noexcept {
        return message_;
    }

  private:
    // Success constructor
    Result() : success_(true), message_() {}

    // Error constructor
    explicit Result(std::string message) : success_(false), message_(std::move(message)) {}

    bool success_ = false;
    std::string message_;
};

}  // namespace plem::core
