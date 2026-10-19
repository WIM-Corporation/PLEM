# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(plem_core)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(spdlog REQUIRED)

# libsodium backs crypto/sha256.cpp.
find_package(PkgConfig REQUIRED)
pkg_check_modules(SODIUM REQUIRED libsodium)

set(SOURCE_FILES
  src/crypto/sha256.cpp
  src/logging/logger.cpp
  src/monitoring/monitoring_context.cpp
  src/threading/rt_memory.cpp
  src/threading/rt_thread_config.cpp
  src/timing/periodic_timer_rt.cpp
)

add_library(${PROJECT_NAME} SHARED
    ${SOURCE_FILES}
)

# Hide all symbols by default; headers opt in with PLEM_CORE_PUBLIC
# (include/plem/core/visibility_control.hpp). PRIVATE: the flags stay on this
# target and do not reach consumers through ament_export_targets.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE
    -fvisibility=hidden
    -fvisibility-inlines-hidden)
endif()

# spdlog is in the public headers (logging/logger.hpp); libsodium is not.
target_link_libraries(${PROJECT_NAME}
  PUBLIC spdlog::spdlog
  PRIVATE ${SODIUM_LIBRARIES}
)

# The installed headers are C++20; the exported target says so to its consumers.
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${SODIUM_INCLUDE_DIRS}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(spdlog)
ament_export_include_directories(include/${PROJECT_NAME})

if(BUILD_TESTING)
  # Test-support headers (SeqlockDataTestPeer): this package's seqlock tests
  # include them from the source tree; installed beside the library's headers
  # (include/${PROJECT_NAME}) only in a BUILD_TESTING build, for other
  # packages' tests; a release build installs no test seam.
  install(DIRECTORY test/include/ DESTINATION include/${PROJECT_NAME})

  find_package(ament_cmake_gtest REQUIRED)

  # SHA-256 hex utility
  ament_add_gtest(${PROJECT_NAME}_test_sha256 test/test_sha256.cpp)
  target_link_libraries(${PROJECT_NAME}_test_sha256 ${PROJECT_NAME})

  # Logger leaves the host process's spdlog state alone
  ament_add_gtest(${PROJECT_NAME}_test_logger test/test_logger.cpp)
  target_link_libraries(${PROJECT_NAME}_test_logger ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_units test/test_units.cpp)
  target_link_libraries(${PROJECT_NAME}_test_units ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_periodic_timer_rt test/test_periodic_timer_rt.cpp)
  target_link_libraries(${PROJECT_NAME}_test_periodic_timer_rt ${PROJECT_NAME} pthread)

  # time -> cycle-count derivation
  ament_add_gtest(${PROJECT_NAME}_test_cycle_conversion test/test_cycle_conversion.cpp)
  target_link_libraries(${PROJECT_NAME}_test_cycle_conversion ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_internal_protective_stop_decision test/test_internal_protective_stop_decision.cpp)
  target_link_libraries(${PROJECT_NAME}_test_internal_protective_stop_decision ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_rt_thread_config test/test_rt_thread_config.cpp)
  target_link_libraries(${PROJECT_NAME}_test_rt_thread_config ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_result test/test_result.cpp)
  target_link_libraries(${PROJECT_NAME}_test_result ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_event test/test_event.cpp)
  target_link_libraries(${PROJECT_NAME}_test_event ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_rt_queue test/test_rt_queue.cpp)
  target_link_libraries(${PROJECT_NAME}_test_rt_queue ${PROJECT_NAME} pthread)

  # the global-context API (monitoring.hpp)
  ament_add_gtest(${PROJECT_NAME}_test_monitoring test/test_monitoring.cpp)
  target_link_libraries(${PROJECT_NAME}_test_monitoring ${PROJECT_NAME} pthread)

  ament_add_gtest(${PROJECT_NAME}_test_seqlock_data test/test_seqlock_data.cpp)
  target_include_directories(${PROJECT_NAME}_test_seqlock_data PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test/include)
  target_link_libraries(${PROJECT_NAME}_test_seqlock_data ${PROJECT_NAME} pthread)

  # Opt-in race check of the same suite: -DPLEM_CORE_TSAN_TESTS=ON in an isolated --build-base.
  # -O0 because GCC's TSan misses the payload-copy race once memcpy is inlined (-O1 and up).
  option(PLEM_CORE_TSAN_TESTS "Also build test_seqlock_data under ThreadSanitizer at -O0" OFF)
  if(PLEM_CORE_TSAN_TESTS)
    ament_add_gtest(${PROJECT_NAME}_test_seqlock_data_tsan test/test_seqlock_data.cpp)
    target_compile_options(${PROJECT_NAME}_test_seqlock_data_tsan PRIVATE -fsanitize=thread -O0 -g)
    target_link_options(${PROJECT_NAME}_test_seqlock_data_tsan PRIVATE -fsanitize=thread)
    # Headers only: linking the library starts spdlog's uninstrumented thread pool (TSan noise).
    target_include_directories(${PROJECT_NAME}_test_seqlock_data_tsan PRIVATE include test/include)
    target_link_libraries(${PROJECT_NAME}_test_seqlock_data_tsan pthread)
  endif()

  ament_add_gtest(${PROJECT_NAME}_test_stream_traits test/test_stream_traits.cpp)
  target_link_libraries(${PROJECT_NAME}_test_stream_traits ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_motion_limit test/test_motion_limit.cpp)
  target_link_libraries(${PROJECT_NAME}_test_motion_limit ${PROJECT_NAME})

  # per-instance isolation + thread-local context
  ament_add_gtest(${PROJECT_NAME}_test_monitoring_context test/test_monitoring_context.cpp)
  target_link_libraries(${PROJECT_NAME}_test_monitoring_context ${PROJECT_NAME} pthread)

  ament_add_gtest(${PROJECT_NAME}_test_tri_state test/test_tri_state.cpp)
  target_link_libraries(${PROJECT_NAME}_test_tri_state ${PROJECT_NAME})
endif()

ament_package()
