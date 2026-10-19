# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(plem_fieldbus)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(plem_core REQUIRED)
find_package(fmt REQUIRED)

# The IgH EtherCAT master is required: a missing one stops configure here, not
# later at `#include <ecrt.h>`. Search order (find_* defaults): CMAKE_PREFIX_PATH,
# then IgH's default prefix /opt/etherlab (HINTS), then the system prefixes
# (/usr/local, /usr, ...). The shared library only (NAMES is a file name, so no
# .a): libethercat is LGPL and plem links it dynamically.
find_path(ETHERCAT_INCLUDE_DIR ecrt.h HINTS /opt/etherlab/include)
find_library(ETHERCAT_LIB NAMES libethercat.so HINTS /opt/etherlab/lib)
if(NOT ETHERCAT_INCLUDE_DIR OR NOT ETHERCAT_LIB)
  message(FATAL_ERROR "plem_fieldbus needs the IgH EtherCAT master (ecrt.h and the shared libethercat.so — the "
    "unversioned development link), which was not found — install IgH to /opt/etherlab (or put its prefix on "
    "CMAKE_PREFIX_PATH)")
endif()
# A cached or -D ETHERCAT_LIB skips the search above, so check the value itself.
# Each refusal below drops the found values from the cache first, so the next
# configure of the same build directory searches again and the fix it names
# takes effect.
set(_igh_include "${ETHERCAT_INCLUDE_DIR}")
set(_igh_lib "${ETHERCAT_LIB}")
if(NOT _igh_lib MATCHES "\\.so$")
  unset(ETHERCAT_INCLUDE_DIR CACHE)
  unset(ETHERCAT_LIB CACHE)
  message(FATAL_ERROR "ETHERCAT_LIB is ${_igh_lib}, not the shared libethercat.so — libethercat is LGPL and plem "
    "links it dynamically only. Point ETHERCAT_LIB at the shared library or leave it unset (the value was dropped "
    "from the CMake cache, so the next configure searches again)")
endif()
# Header and library from one IgH install: the library must sit under the header's
# prefix's lib/ (lib64/ and lib/<multiarch>/ too); /usr and /usr/local are two installs.
get_filename_component(_igh_prefix "${_igh_include}" DIRECTORY)
string(FIND "${_igh_lib}" "${_igh_prefix}/lib" _igh_lib_pos)
if(NOT _igh_lib_pos EQUAL 0)
  unset(ETHERCAT_INCLUDE_DIR CACHE)
  unset(ETHERCAT_LIB CACHE)
  message(FATAL_ERROR "ecrt.h (${_igh_include}) and libethercat.so (${_igh_lib}) come from different IgH "
    "prefixes — put the one IgH prefix you want first on CMAKE_PREFIX_PATH")
endif()
get_filename_component(ETHERCAT_LIB_DIR "${ETHERCAT_LIB}" DIRECTORY)

# The EtherCAT bus and its devices, compiled into the library and, against
# the fake master, into the fake-master test targets.
set(PLEM_FIELDBUS_BUS_SOURCES
  src/ethercat/bus.cpp
  src/ethercat/configuration_validation.cpp
  src/ethercat/pdo_layout_validation.cpp
  src/ethercat/pdo_scan.cpp
  src/ethercat/device_base.cpp
  src/ethercat/servo/device.cpp
  src/ethercat/servo/pdo_handler.cpp
  src/ethercat/state_machine.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${PLEM_FIELDBUS_BUS_SOURCES}
  src/ethercat/controller.cpp
)
# The installed headers are C++20; the exported target says so to its consumers.
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${ETHERCAT_INCLUDE_DIR}
)
# Package-private headers (the .hpp files under src/ethercat/): the library's
# own TUs include them as "ethercat/...", same spelling as the test targets'
# src/ include seam. Never installed, and PLEM_FIELDBUS_INTERNAL
# (internal_visibility.hpp) keeps what they emit out of the .so's export table.
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(${PROJECT_NAME}
  ${ETHERCAT_LIB}
  plem_core::plem_core
  fmt::fmt
)
# INSTALL_RPATH lists explicit entries only; USE_LINK_PATH would also bake in the
# workspace staging absolute paths at link time (dead paths on the device), leaking
# build paths into the release .so. plem dependencies resolve on the device through
# ld.so.conf / the overlay env.
#
# Inline code from the package-private headers (src/ethercat/; see
# internal_visibility.hpp) is emitted at the optimizer's discretion, so hiding
# inline code keeps it out of the export table whatever the optimizer emits.
# Consumers compile their own copies of inline code; GCC keeps function-local
# statics default.
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "${ETHERCAT_LIB_DIR}"
  VISIBILITY_INLINES_HIDDEN ON
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
ament_export_dependencies(plem_core fmt)
ament_export_include_directories(include/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(spdlog REQUIRED)  # test_ethercat_bus_configuration_fake_master captures log lines

  # Test-support headers (fake master controls, probes, fixtures): header-only,
  # dev-only surface. Installed beside the library's headers
  # (include/${PROJECT_NAME}) so downstream package tests include
  # <plem/fieldbus/test/...> through the exported target. Installed only
  # when BUILD_TESTING is ON, which is ament_cmake's default; a build with
  # -DBUILD_TESTING=OFF (the release debs) installs no test double.
  install(DIRECTORY test/include/ DESTINATION include/${PROJECT_NAME})

  # CiA 402 profile declaration rules (header-only, ecrt-free)
  ament_add_gtest(${PROJECT_NAME}_test_cia402_profile test/test_cia402_profile.cpp)
  target_link_libraries(${PROJECT_NAME}_test_cia402_profile ${PROJECT_NAME})

  # Servo state machine test
  ament_add_gtest(${PROJECT_NAME}_test_state_machine test/test_state_machine.cpp)
  target_link_libraries(${PROJECT_NAME}_test_state_machine ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_test_state_machine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/include)  # <plem/fieldbus/test/cia402_drive_sim.hpp>

  # Servo conversions test
  ament_add_gtest(${PROJECT_NAME}_test_servo_conversions test/test_servo_conversions.cpp)
  target_link_libraries(${PROJECT_NAME}_test_servo_conversions ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_test_servo_conversions PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/include)  # <plem/fieldbus/test/single_axis_cst_layout.hpp>

  # Servo PDO handler test
  ament_add_gtest(${PROJECT_NAME}_test_servo_pdo_handler test/test_servo_pdo_handler.cpp)
  target_link_libraries(${PROJECT_NAME}_test_servo_pdo_handler ${PROJECT_NAME})

  # Servo PDO types test
  ament_add_gtest(${PROJECT_NAME}_test_servo_pdo_types test/test_servo_pdo_types.cpp)
  target_link_libraries(${PROJECT_NAME}_test_servo_pdo_types ${PROJECT_NAME})

  # Servo info / spec struct validation test
  ament_add_gtest(${PROJECT_NAME}_test_servo_info test/test_servo_info.cpp)
  target_link_libraries(${PROJECT_NAME}_test_servo_info ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_test_servo_info PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/include)  # <plem/fieldbus/test/single_axis_cst_layout.hpp>



  # EtherCAT controller test (test/include/ include root)
  ament_add_gtest(${PROJECT_NAME}_test_ethercat_controller test/test_ethercat_controller.cpp)
  target_include_directories(${PROJECT_NAME}_test_ethercat_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/include)
  target_link_libraries(${PROJECT_NAME}_test_ethercat_controller ${PROJECT_NAME})

  # Fake-master kit: the ecrt fake (FakeEcrt, <plem/fieldbus/test/fake_ecrt.hpp>)
  # as a static library, installed and exported only here (BUILD_TESTING) so
  # other packages' tests can run the bus on it. The fake only, not the bus or
  # controller sources: a consumer links the shared ${PROJECT_NAME} as usual
  # (a downstream package's tests do) and the executable's ecrt_* definitions
  # take precedence over libethercat's; a static copy of the bus would put a
  # second EtherCATBus next to the one in the shared library.
  add_library(${PROJECT_NAME}_fake_master STATIC test/fakes/fake_ecrt.cpp)
  target_include_directories(${PROJECT_NAME}_fake_master
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    PRIVATE ${ETHERCAT_INCLUDE_DIR})
  # plem_core: an unconfigured slave reports the synthetic drive identity
  # from single_axis_cst_layout.hpp, which includes configuration.hpp
  # (plem/core/units.hpp).
  target_link_libraries(${PROJECT_NAME}_fake_master PRIVATE fmt::fmt plem_core::plem_core)
  target_compile_features(${PROJECT_NAME}_fake_master PUBLIC cxx_std_20)
  # In a BUILD_TESTING build the exported fake joins plem_fieldbus's targets:
  # consumers link it by name from test targets only, never through
  # ament_target_dependencies on a non-test target.
  install(TARGETS ${PROJECT_NAME}_fake_master
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib)

  # Fake-master seam test (hermetic bring-up / DC contract pinning).
  # Compiles the bus/device sources directly and links the fake-master kit
  # instead of libethercat, so EtherCATBus::initialize() runs without
  # hardware and the exact ecrt call sequence is assertable (config_dc before
  # activate, send-interval derivation, DC-disabled adds no ecrt calls).
  # Deliberately not linked against ${PROJECT_NAME}: this target compiles
  # the bus itself (package-private src/ seam).
  ament_add_gtest(${PROJECT_NAME}_test_ethercat_bus_fake_master
    test/test_ethercat_bus_fake_master.cpp
    test/test_ethercat_bus_configuration_fake_master.cpp
    test/test_ethercat_bus_scanned_fake_master.cpp
    test/test_pdo_scan_fake_master.cpp
    test/test_servo_device.cpp
    test/test_ethercat_bus.cpp
    ${PLEM_FIELDBUS_BUS_SOURCES}
  )
  target_include_directories(${PROJECT_NAME}_test_ethercat_bus_fake_master PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${CMAKE_CURRENT_SOURCE_DIR}/test/include  # <plem/fieldbus/test/cia402_connection_fixture.hpp> (consumer half)
    ${CMAKE_CURRENT_SOURCE_DIR}/src      # package-private seam (servo/device.cpp reaches fault_reset_pulse.hpp)
    ${ETHERCAT_INCLUDE_DIR}
  )
  target_link_libraries(${PROJECT_NAME}_test_ethercat_bus_fake_master
    ${PROJECT_NAME}_fake_master
    plem_core::plem_core
    fmt::fmt
    spdlog::spdlog
  )

  # Controller RT-loop wiring pins on the fake master:
  # compiles the real run_loop against fake_ecrt so the cycle-level brake
  # contract is pinned at device level (ordered PLEM_EMIT trace). The src/
  # include dir exposes the package-private shell headers to this target
  # only (test seam, not an installed surface).
  ament_add_gtest(${PROJECT_NAME}_test_ethercat_controller_fake_master
    test/test_ethercat_controller_fake_master.cpp
    test/test_brake_golden_trace_fake_master.cpp
    test/test_drive_safety_fake_master.cpp
    test/test_io_device_hooks_fake_master.cpp
    ${PLEM_FIELDBUS_BUS_SOURCES}
    src/ethercat/controller.cpp
  )
  target_include_directories(${PROJECT_NAME}_test_ethercat_controller_fake_master PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test/include  # <plem/fieldbus/test/single_axis_cst_layout.hpp>
    ${ETHERCAT_INCLUDE_DIR}
  )
  target_link_libraries(${PROJECT_NAME}_test_ethercat_controller_fake_master
    ${PROJECT_NAME}_fake_master
    plem_core::plem_core
    fmt::fmt
  )

  # Package-private entities must stay out of the .so's export table
  # (PLEM_FIELDBUS_INTERNAL, src/ethercat/internal_visibility.hpp). The
  # behavior suites cannot see this: it is an ELF property of the linked
  # library, so the symbol table is the oracle. ament_add_test (not a bare
  # add_test) so the result lands in test_results/ with the other suites'.
  find_package(ament_cmake_test REQUIRED)
  # Without libethercat configure stops with the install instruction (IgH's
  # absence simulated by ignoring the directories this configure found it in,
  # wherever that is).
  ament_add_test(${PROJECT_NAME}_test_configure_without_igh
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/check_configure_without_igh.sh"
            "${CMAKE_CURRENT_SOURCE_DIR}" "${ETHERCAT_INCLUDE_DIR}" "${ETHERCAT_LIB_DIR}"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    SKIP_RETURN_CODE 77)  # a second IgH install: absence cannot be simulated
  ament_add_test(${PROJECT_NAME}_test_internal_symbols
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/check_internal_symbols.sh"
            "$<TARGET_FILE:${PROJECT_NAME}>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  # PDO layout semantic validation (pure, no ecrt dependency)
  ament_add_gtest(${PROJECT_NAME}_test_pdo_layout_validation test/test_pdo_layout_validation.cpp)
  target_link_libraries(${PROJECT_NAME}_test_pdo_layout_validation ${PROJECT_NAME})

  # Watchdog logic test (header-only, no EtherCAT dependency)
  ament_add_gtest(${PROJECT_NAME}_test_watchdog test/test_watchdog.cpp)
  target_link_libraries(${PROJECT_NAME}_test_watchdog ${PROJECT_NAME})

  # Brake decision truth table (header-only pure logic)
  ament_add_gtest(${PROJECT_NAME}_test_brake_logic test/test_brake_logic.cpp)
  target_link_libraries(${PROJECT_NAME}_test_brake_logic ${PROJECT_NAME})

  # Brake cycle-step shell ordering (package-private header;
  # the src/ include dir is the deliberate test-only seam)
  ament_add_gtest(${PROJECT_NAME}_test_brake_cycle_step test/test_brake_cycle_step.cpp)
  target_include_directories(${PROJECT_NAME}_test_brake_cycle_step PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(${PROJECT_NAME}_test_brake_cycle_step ${PROJECT_NAME})

  # Sequence-step decision table -- the package-private pure per-joint
  # decision, same src/ include seam as brake_cycle_step.
  ament_add_gtest(${PROJECT_NAME}_test_sequence_step test/test_sequence_step.cpp)
  target_include_directories(${PROJECT_NAME}_test_sequence_step PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(${PROJECT_NAME}_test_sequence_step ${PROJECT_NAME})

  # Drive-tier torque gate truth table (installed pure header)
  ament_add_gtest(${PROJECT_NAME}_test_drive_safety test/test_drive_safety.cpp)
  target_link_libraries(${PROJECT_NAME}_test_drive_safety ${PROJECT_NAME})

  # Edge-triggered RT events (package-private header, same src/ seam)
  ament_add_gtest(${PROJECT_NAME}_test_rt_event_emitter test/test_rt_event_emitter.cpp)
  target_include_directories(${PROJECT_NAME}_test_rt_event_emitter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(${PROJECT_NAME}_test_rt_event_emitter ${PROJECT_NAME})

  # Fault-reset pulse truth table (package-private header,
  # same src/ test-only seam; header-only pure logic, no EtherCAT dependency)
  ament_add_gtest(${PROJECT_NAME}_test_fault_reset_pulse test/test_fault_reset_pulse.cpp)
  target_include_directories(${PROJECT_NAME}_test_fault_reset_pulse PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(${PROJECT_NAME}_test_fault_reset_pulse ${PROJECT_NAME})
endif()

ament_package()
