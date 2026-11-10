# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(plem_drive)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(plem_core REQUIRED)
find_package(plem_fieldbus REQUIRED)
find_package(libyaml_vendor REQUIRED)
find_package(fmt REQUIRED)
find_package(nlohmann_json REQUIRED)  # plem drive check prints JSON (header-only, not exported)
# The ROS layer
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(plem_drive_msgs REQUIRED)
find_package(plem_rt_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

# The drive library: the drive model file loader and DriveHost<N>. It links no
# ROS library (a process without ROS runs drives with it); the ROS layer is
# plem_drive_ros below.
add_library(${PROJECT_NAME} SHARED
  src/bus_setup.cpp
  src/drive_host.cpp
  src/loader/yaml_tree.cpp
  src/loader/digest.cpp
  src/loader/reader.cpp
  src/loader/approval.cpp
  src/loader/inspection.cpp
  src/models/validation.cpp
  src/models/pointer_free_fields.cpp
)

# The installed headers are C++20; the exported target says so to its consumers.
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# Headers under src/ are not installed: they sit beside their .cpp and are
# included by their path under src/ ("drive_host_rules.hpp", "loader/…",
# "models/…", "ros/…", "tool/…"). The tests reach them through the same src/
# include directory.
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# The version `plem drive check` prints and the bench records as
# implementation_revision (schema §6, provenance only). Baked in from
# package.xml <version>; ament_package() parses it only at the end, too late
# for the compile definition below. See platform_version in
# src/loader/digest.hpp.
ament_package_xml()
set(PLEM_PLATFORM_VERSION "${${PROJECT_NAME}_VERSION}")
if(NOT PLEM_PLATFORM_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
  message(FATAL_ERROR "package.xml <version> holds '${PLEM_PLATFORM_VERSION}' — expected MAJOR.MINOR.PATCH")
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE PLEM_PLATFORM_VERSION="${PLEM_PLATFORM_VERSION}")

target_link_libraries(${PROJECT_NAME}
  plem_core::plem_core
  plem_fieldbus::plem_fieldbus
  fmt::fmt
)

# libyaml_vendor brings the `yaml` target the drive model file reader parses
# with (<yaml.h> stays in src/loader/yaml_tree.cpp).
ament_target_dependencies(${PROJECT_NAME}
  libyaml_vendor
)

# Every header under include/ is installed, and only those: <plem/drive/…>
# (namespace plem::drive) and <plem/drive/ros/…> (plem::drive::ros, for another
# ROS hardware plugin that reports through the same service and topics).
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# drive_model_tool: the drive model file tool behind `plem drive check`, the
# bench's approve step and run's probe. Its verbs are a static library so the
# gtest below drives them as functions; the executable is dispatch only.
# Installed under lib/plem_drive/ (ament's per-package executable dir, outside
# PATH; the `plem-drive` command execs it). No ROS. check and approve read and
# write files only; probe opens the EtherCAT master and checks the drives (it
# writes no PDO, SDO or DC setting and does not activate the master).
add_library(drive_model_tool_lib STATIC src/tool/check.cpp src/tool/approve.cpp src/tool/probe.cpp)
target_include_directories(drive_model_tool_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(drive_model_tool_lib PUBLIC ${PROJECT_NAME})
target_link_libraries(drive_model_tool_lib PRIVATE nlohmann_json::nlohmann_json)
add_executable(drive_model_tool src/tool/main.cpp)
target_link_libraries(drive_model_tool drive_model_tool_lib)
install(TARGETS drive_model_tool DESTINATION lib/${PROJECT_NAME})


# The ROS layer (src/ros/, namespace plem::drive::ros): the ros2_control
# hardware plugin and the stream controller (`plem drive run`), and the
# helpers another hardware plugin links, whose headers are installed as
# <plem/drive/ros/…> (control command service, rt_raw batching, to_rt_event).
# Registers no plugin; the registrations are plem_drive_plugins below.
add_library(plem_drive_ros SHARED
  src/ros/control_command_service.cpp
  src/ros/drive_bus.cpp
  src/ros/drive_system_interface.cpp
  src/ros/stream_controller.cpp
  src/ros/rt_raw_batcher.cpp
  src/ros/rt_raw_relay.cpp)
target_compile_features(plem_drive_ros PUBLIC cxx_std_20)
target_include_directories(plem_drive_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(plem_drive_ros PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
ament_target_dependencies(plem_drive_ros rclcpp rclcpp_lifecycle plem_drive_msgs plem_rt_msgs std_msgs
  hardware_interface controller_interface pluginlib realtime_tools)
target_link_libraries(plem_drive_ros
  plem_core::plem_core
  plem_fieldbus::plem_fieldbus
  ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} plem_drive_ros
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

# The pluginlib registrations (src/ros/plugins.cpp), the library the plugin
# descriptions name. Loaded by pluginlib, never linked: not exported.
add_library(plem_drive_plugins SHARED src/ros/plugins.cpp)
target_include_directories(plem_drive_plugins PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(plem_drive_plugins plem_drive_ros)
ament_target_dependencies(plem_drive_plugins pluginlib)
install(TARGETS plem_drive_plugins LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(hardware_interface hardware_plugins.xml)
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

# `ros2 run plem_drive rt_raw_relay`: rt_raw one message per sample, for
# plotting every cycle (PlotJuggler). Normal scheduling.
add_executable(rt_raw_relay src/ros/rt_raw_relay_main.cpp)
target_include_directories(rt_raw_relay PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(rt_raw_relay plem_drive_ros)
install(TARGETS rt_raw_relay DESTINATION lib/${PROJECT_NAME})

# `plem.drive` (import plem.drive) and the `plem drive <verb>` command behind
# it (plem/drive/cli/: check execs drive_model_tool; run starts the bus, offers
# the bench for a file not yet approved, and prints the bus timing with
# --monitor). `plem` is a PEP 420 namespace package: there is no
# plem/__init__.py, so another package can add plem.<name> beside it, and
# ament_python_install_package (which wants that file) cannot install it. The
# directory goes in as is, and the hook below puts its destination on
# PYTHONPATH when the install is sourced (what that macro does first). The
# command goes in bin/, which the same setup puts on PATH.
# The hook is ament_cmake_python's own step, not a documented command: if a
# ROS release renames it, stop here rather than install a plem.drive that
# Python cannot find.
if(NOT COMMAND _ament_cmake_python_register_environment_hook)
  message(FATAL_ERROR "ament_cmake_python has no _ament_cmake_python_register_environment_hook: "
    "plem.drive would install outside PYTHONPATH. Register the install directory on PYTHONPATH "
    "the way this ament_cmake_python's ament_python_install_package does.")
endif()
_ament_cmake_python_register_environment_hook()
install(DIRECTORY plem DESTINATION ${PYTHON_INSTALL_DIR} PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/plem-drive DESTINATION bin)
# The example drive model file a user starts from, the README that says which
# example to start from, and the two-axis A6B example, a mirrored pair
# (hardware runs: docs/hardware-evidence.md): its drive model file, a sync demo
# and a PlotJuggler layout with the pair's sync tab (`plem drive run --plot`
# builds its own layout from the bus's axes).
install(FILES examples/example_drive.yaml examples/README.md DESTINATION share/${PROJECT_NAME}/examples)
install(DIRECTORY examples/a6b_pair DESTINATION share/${PROJECT_NAME}/examples)
# The agent skill that writes a drive model file with a source for every value
# (getting-started section 1), into the install prefix's skill folder,
# share/skills/<name> (link it into an agent's skill directory).
install(DIRECTORY skills/ DESTINATION share/skills)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The tests include <plem/fieldbus/test/…>, installed by plem_fieldbus
  # only when it was built with BUILD_TESTING=ON.

  # The drive model file's strict-YAML reader: structure kept, every
  # syntax-level refusal of the schema's §2
  ament_add_gtest(${PROJECT_NAME}_test_yaml_tree test/test_yaml_tree.cpp)
  target_include_directories(${PROJECT_NAME}_test_yaml_tree PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(${PROJECT_NAME}_test_yaml_tree ${PROJECT_NAME})

  # The drive model file: the example golden, the table-driven key walk,
  # the scalar grammar and the file-only rules.
  ament_add_gtest(${PROJECT_NAME}_test_drive_model_file test/test_drive_model_file.cpp)
  target_include_directories(${PROJECT_NAME}_test_drive_model_file PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_compile_definitions(${PROJECT_NAME}_test_drive_model_file PRIVATE
    DRIVE_EXAMPLE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/test/data/drive_synthetic.yaml")
  target_link_libraries(${PROJECT_NAME}_test_drive_model_file ${PROJECT_NAME})

  # drive_model_tool check / approve over fixtures (an unapproved original + a bench
  # file in a temp dir): the stdout contract, every refusal, in-place
  # replacement with comments preserved.
  ament_add_gtest(${PROJECT_NAME}_test_drive_model_tool test/test_drive_model_tool.cpp)
  target_include_directories(${PROJECT_NAME}_test_drive_model_tool PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_compile_definitions(${PROJECT_NAME}_test_drive_model_tool PRIVATE
    DRIVE_EXAMPLE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/test/data/drive_synthetic.yaml"
    CHECK_GOLDEN_JSON="${CMAKE_CURRENT_SOURCE_DIR}/test/data/check_synthetic.golden.json"
    CHECK_SUMMARY_GOLDEN="${CMAKE_CURRENT_SOURCE_DIR}/test/data/check_synthetic.golden.txt"
    PLEM_DRIVE_EXE="$<TARGET_FILE:drive_model_tool>")
  target_link_libraries(${PROJECT_NAME}_test_drive_model_tool drive_model_tool_lib nlohmann_json::nlohmann_json)
  add_dependencies(${PROJECT_NAME}_test_drive_model_tool drive_model_tool)

  # The version source: a standalone copy of this package configures and
  # bakes in package.xml's <version>.
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(${PROJECT_NAME}_test_configure_without_packaging
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/check_configure_without_packaging.sh"
            "${CMAKE_CURRENT_SOURCE_DIR}"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  # Drive-tier host: value gate, carry, enable decision, create refusal
  # (pure pins; the loop is proven on the fieldbus fake-master target)
  ament_add_gtest(${PROJECT_NAME}_test_drive_host test/test_drive_host.cpp)
  target_include_directories(${PROJECT_NAME}_test_drive_host PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(${PROJECT_NAME}_test_drive_host ${PROJECT_NAME})

  # drive_model_tool probe: the bus's start-up check of an approved file's
  # drives, on the fake master linked into the executable.
  ament_add_gtest(${PROJECT_NAME}_test_drive_probe test/test_drive_probe.cpp)
  target_include_directories(${PROJECT_NAME}_test_drive_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_compile_definitions(${PROJECT_NAME}_test_drive_probe PRIVATE
    DRIVE_EXAMPLE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/test/data/drive_synthetic.yaml")
  target_link_libraries(${PROJECT_NAME}_test_drive_probe
    drive_model_tool_lib
    plem_fieldbus::plem_fieldbus_fake_master
    ${CMAKE_DL_LIBS})

  # DriveHost's own methods on the shared plem_fieldbus with the fake master
  # linked into the executable (its ecrt_* definitions win over libethercat's).
  ament_add_gtest(${PROJECT_NAME}_test_drive_host_lifecycle test/test_drive_host_lifecycle.cpp)
  target_include_directories(${PROJECT_NAME}_test_drive_host_lifecycle PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(${PROJECT_NAME}_test_drive_host_lifecycle
    ${PROJECT_NAME}
    plem_fieldbus::plem_fieldbus_fake_master
    ${CMAKE_DL_LIBS})  # dlsym: SetUp checks the fake owns the global ecrt_* scope

  # Two tests use the ros2_control API where Jazzy and Humble differ: a command
  # handle's set_value returns a bool (or nothing), its lock is get_mutex(),
  # HardwareInfo names the plugin hardware_plugin_name (or hardware_class_type),
  # and a controller's init() takes the URDF and update rate (or not). Probe the
  # found ros2_control for that API rather than trust a version number;
  # PLEM_ROS2_CONTROL_JAZZY=1 selects it in those tests.
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_LIBRARIES controller_interface::controller_interface
    hardware_interface::hardware_interface rclcpp_lifecycle::rclcpp_lifecycle)
  check_cxx_source_compiles([=[
#include <mutex>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <utility>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
int main() {
  hardware_interface::HardwareInfo info;
  info.hardware_plugin_name = "probe";
  double value = 0.0;
  hardware_interface::CommandInterface handle("joint", "velocity", &value);
  std::shared_lock<std::shared_mutex> lock(handle.get_mutex());
  static_assert(std::is_same_v<decltype(handle.set_value(0.0)), bool>);
  static_assert(std::is_same_v<decltype(std::declval<controller_interface::ControllerInterface&>().init(
                                   std::string(), std::string(), 0u, std::string(), rclcpp::NodeOptions())),
                               controller_interface::return_type>);
  return 0;
}
]=] PLEM_ROS2_CONTROL_JAZZY_API)
  unset(CMAKE_REQUIRED_LIBRARIES)

  ament_add_gtest(${PROJECT_NAME}_test_control_command_service test/ros/test_control_command_service.cpp)
  target_link_libraries(${PROJECT_NAME}_test_control_command_service plem_drive_ros plem_core::plem_core)
  ament_target_dependencies(${PROJECT_NAME}_test_control_command_service rclcpp plem_rt_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_drive_bus test/ros/test_drive_bus.cpp)
  target_include_directories(${PROJECT_NAME}_test_drive_bus PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(${PROJECT_NAME}_test_drive_bus plem_drive_ros plem_core::plem_core)
  ament_target_dependencies(${PROJECT_NAME}_test_drive_bus rclcpp plem_drive_msgs plem_rt_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_drive_system_interface test/ros/test_drive_system_interface.cpp)
  target_include_directories(${PROJECT_NAME}_test_drive_system_interface PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_compile_definitions(${PROJECT_NAME}_test_drive_system_interface PRIVATE
    DRIVE_EXAMPLE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/test/data/drive_synthetic.yaml")
  target_link_libraries(${PROJECT_NAME}_test_drive_system_interface plem_drive_ros plem_core::plem_core)
  ament_target_dependencies(${PROJECT_NAME}_test_drive_system_interface rclcpp hardware_interface)
  if(PLEM_ROS2_CONTROL_JAZZY_API)
    target_compile_definitions(${PROJECT_NAME}_test_drive_system_interface PRIVATE PLEM_ROS2_CONTROL_JAZZY=1)
  endif()

  ament_add_gtest(${PROJECT_NAME}_test_stream_controller test/ros/test_stream_controller.cpp)
  target_include_directories(${PROJECT_NAME}_test_stream_controller PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(${PROJECT_NAME}_test_stream_controller plem_drive_ros)
  ament_target_dependencies(${PROJECT_NAME}_test_stream_controller rclcpp controller_interface hardware_interface std_msgs)
  if(PLEM_ROS2_CONTROL_JAZZY_API)
    target_compile_definitions(${PROJECT_NAME}_test_stream_controller PRIVATE PLEM_ROS2_CONTROL_JAZZY=1)
  endif()

  ament_add_gtest(${PROJECT_NAME}_test_rt_raw_batcher test/ros/test_rt_raw_batcher.cpp)
  target_link_libraries(${PROJECT_NAME}_test_rt_raw_batcher plem_drive_ros plem_core::plem_core)
  ament_target_dependencies(${PROJECT_NAME}_test_rt_raw_batcher rclcpp plem_rt_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_rt_raw_relay test/ros/test_rt_raw_relay.cpp)
  target_include_directories(${PROJECT_NAME}_test_rt_raw_relay PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(${PROJECT_NAME}_test_rt_raw_relay plem_drive_ros)
  ament_target_dependencies(${PROJECT_NAME}_test_rt_raw_relay rclcpp plem_rt_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_drive_host_adapter test/ros/test_drive_host_adapter.cpp)
  target_include_directories(${PROJECT_NAME}_test_drive_host_adapter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(${PROJECT_NAME}_test_drive_host_adapter plem_drive_ros
    plem_core::plem_core plem_fieldbus::plem_fieldbus ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_test_drive_host_adapter PRIVATE
    PLEM_DRIVE_EXAMPLE_YAML="${CMAKE_CURRENT_SOURCE_DIR}/examples/example_drive.yaml")

  # The bench runner's rclpy-free parts and its bench file, through this build's
  # drive_model_tool approve/check.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_test_bench test/python/test_bench.py
    ENV PLEM_DRIVE_BIN=$<TARGET_FILE:drive_model_tool>
        PLEM_DRIVE_EXAMPLE_YAML=${CMAKE_CURRENT_SOURCE_DIR}/examples/example_drive.yaml
        PLEM_DRIVE_BENCH_SCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/plem/drive/cli/bench.py)
  # The dispatcher: its help, usage codes and which verb goes where (check execs
  # the installed drive_model_tool).
  ament_add_pytest_test(${PROJECT_NAME}_test_cli test/python/test_cli.py
    ENV PLEM_DRIVE_BIN=${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/drive_model_tool
        PLEM_DRIVE_EXAMPLE_YAML=${CMAKE_CURRENT_SOURCE_DIR}/examples/example_drive.yaml)
  # plem drive run: what it generates from check, its lanes, host warnings, refusals.
  ament_add_pytest_test(${PROJECT_NAME}_test_run test/python/test_run.py
    ENV PLEM_DRIVE_BIN=$<TARGET_FILE:drive_model_tool>
        PLEM_DRIVE_EXAMPLE_YAML=${CMAKE_CURRENT_SOURCE_DIR}/test/data/drive_synthetic.yaml)
  # plem.drive's motion profiles: worked examples, phase sync, limits, validation.
  ament_add_pytest_test(${PROJECT_NAME}_test_profile test/python/test_profile.py)
  # ...and against Ruckig (test only): thousands of random moves through a tiny
  # C++ reference that runs Ruckig offline. Never skipped here: the env is set.
  find_package(ruckig REQUIRED)
  add_executable(ruckig_reference test/ruckig_reference.cpp)
  target_link_libraries(ruckig_reference ruckig::ruckig)
  ament_add_pytest_test(${PROJECT_NAME}_test_profile_ruckig test/python/test_profile_ruckig.py
    TIMEOUT 120
    ENV PLEM_RUCKIG_REFERENCE=$<TARGET_FILE:ruckig_reference>)
  # plem.drive.cia402 against the C++ statusword decoder, all 65536 words.
  add_executable(statusword_reference test/statusword_reference.cpp)
  target_link_libraries(statusword_reference plem_fieldbus::plem_fieldbus)
  ament_add_pytest_test(${PROJECT_NAME}_test_cia402 test/python/test_cia402.py
    ENV PLEM_STATUSWORD_REFERENCE=$<TARGET_FILE:statusword_reference>)
  # SIGINT to the controller manager: the bus's stop, then its end. A test-only
  # plugin (recording host, no bus) found through a prefix in the build tree.
  set(_test_prefix ${CMAKE_CURRENT_BINARY_DIR}/test_prefix)
  add_library(plem_drive_test_plugins SHARED test/ros/plugins/recording_drive_system_interface.cpp)
  target_include_directories(plem_drive_test_plugins PRIVATE src test)
  target_link_libraries(plem_drive_test_plugins plem_drive_ros)
  set_target_properties(plem_drive_test_plugins PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${_test_prefix}/lib)
  file(WRITE ${_test_prefix}/share/ament_index/resource_index/packages/plem_drive_test "")
  file(WRITE ${_test_prefix}/share/ament_index/resource_index/hardware_interface__pluginlib__plugin/plem_drive_test
    "share/plem_drive_test/plugins.xml")
  # pluginlib names the package from the package.xml beside the plugin description.
  file(WRITE ${_test_prefix}/share/plem_drive_test/package.xml
    "<package format=\"3\"><name>plem_drive_test</name></package>\n")
  file(WRITE ${_test_prefix}/share/plem_drive_test/plugins.xml
"<library path=\"plem_drive_test_plugins\">
  <class name=\"plem_drive/test/RecordingDriveSystemInterface\"
         type=\"plem::drive::ros::test::RecordingDriveSystemInterface\"
         base_class_type=\"hardware_interface::SystemInterface\"/>
</library>
")
  # pluginlib creates both plugins of this package after the recording host
  # (a plugin that links plem_drive_ros) is loaded.
  ament_add_gtest(${PROJECT_NAME}_test_plugin_load test/ros/test_plugin_load.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${_test_prefix})
  ament_target_dependencies(${PROJECT_NAME}_test_plugin_load pluginlib hardware_interface controller_interface)
  add_dependencies(${PROJECT_NAME}_test_plugin_load plem_drive_test_plugins)
  find_package(controller_manager REQUIRED)
  get_filename_component(_cm_prefix "${controller_manager_DIR}/../../.." ABSOLUTE)
  ament_add_pytest_test(${PROJECT_NAME}_test_drive_sigint test/python/test_drive_sigint.py
    TIMEOUT 90
    ENV PLEM_DRIVE_TEST_PREFIX=${_test_prefix}
        PLEM_ROS2_CONTROL_NODE=${_cm_prefix}/lib/controller_manager/ros2_control_node)
  # The whole run chain without a bus: generated description and parameters,
  # the controller manager over the simulated host, the spawned lanes, plem.drive.
  ament_add_pytest_test(${PROJECT_NAME}_test_drive_run_e2e test/python/test_drive_run_e2e.py
    TIMEOUT 180
    ENV PLEM_DRIVE_TEST_PREFIX=${_test_prefix}
        PLEM_ROS2_CONTROL_NODE=${_cm_prefix}/lib/controller_manager/ros2_control_node)
  # The installed plem.drive, found by a fresh interpreter outside this tree.
  ament_add_pytest_test(${PROJECT_NAME}_test_installed_import test/python/test_installed_import.py
    ENV PLEM_DRIVE_PREFIX=${CMAKE_INSTALL_PREFIX})
  # plem.drive.Bus against a fake bus graph (rclpy, in-process).
  ament_add_pytest_test(${PROJECT_NAME}_test_bus test/python/test_bus.py TIMEOUT 120)
  # The monitor's ROS-free fold (sequence loss, DC sentinel, percentile bins).
  ament_add_pytest_test(${PROJECT_NAME}_test_monitor test/python/test_monitor.py)
  ament_add_pytest_test(${PROJECT_NAME}_test_fleet test/python/test_fleet.py)
  ament_add_pytest_test(${PROJECT_NAME}_test_fleet_session test/python/test_fleet_session.py)
  # The A6B example passes the loader, the value gate and the bus's startup
  # check of drive_model_tool (exit 0 = the approval it claims holds). Its
  # values stay unproven here.
  ament_add_test(${PROJECT_NAME}_test_a6b_example_check GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND $<TARGET_FILE:drive_model_tool> check
            ${CMAKE_CURRENT_SOURCE_DIR}/examples/a6b_pair/drive.yaml)

endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# The drive library's link is plain (transitive), so a consumer links `yaml`
# too; the export makes it find the vendored target instead of a bare -lyaml
# that the system libyaml would satisfy. The rest is the ROS layer's.
ament_export_dependencies(plem_core plem_fieldbus libyaml_vendor fmt rclcpp rclcpp_lifecycle plem_drive_msgs
  plem_rt_msgs std_msgs hardware_interface controller_interface pluginlib realtime_tools)

ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
