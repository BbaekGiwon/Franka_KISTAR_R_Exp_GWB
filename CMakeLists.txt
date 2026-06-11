cmake_minimum_required(VERSION 3.22)
project(shm_test 
VERSION 0.1.0 
LANGUAGES C CXX)

set(SOEM_INCLUDE_INSTALL_DIR include/soem)
set(SOEM_LIB_INSTALL_DIR lib)


if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  message(STATUS "SOEM: Build test")
  set(KISTAR_BUILD_TESTS TRUE)
else()
  message(STATUS "SOEM: not building tests when built as dependency")
  set(KISTAR_BUILD_TESTS FALSE)
endif()

execute_process(COMMAND uname -r OUTPUT_VARIABLE UNAME_RES OUTPUT_STRIP_TRAILING_WHITESPACE)
message("-- kernel version : " ${UNAME_RES})

set(OS "linux")

# Linux Setting
set(OS_LIBS pthread rt)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
if(EXISTS "/opt/openrobots")
  list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")
endif()
find_package(HDF5 REQUIRED COMPONENTS C CXX)

message(STATUS "OS is ${OS}")

file(GLOB SOEM_SOURCES soem_lib/soem/*.c)
file(GLOB OSAL_SOURCES soem_lib/osal/${OS}/*.c)
file(GLOB OSHW_SOURCES soem_lib/oshw/${OS}/*.c)

file(GLOB SOEM_HEADERS soem_lib/soem/*.h)
file(GLOB OSAL_HEADERS soem_lib/osal/osal.h soem_lib/osal/${OS}/*.h)
file(GLOB OSHW_HEADERS soem_lib/oshw/${OS}/*.h)
file(GLOB KISTAR_HEADERS inc/*.h)

add_library(soem STATIC
  ${SOEM_SOURCES}
  ${OSAL_SOURCES}
  ${OSHW_SOURCES}
  ${OSHW_EXTRA_SOURCES})
target_link_libraries(soem ${OS_LIBS})


target_include_directories(soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem_lib/soem>
  $<INSTALL_INTERFACE:include/soem>)

target_include_directories(soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem_lib/osal>
  $<INSTALL_INTERFACE:include/soem>)

target_include_directories(soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem_lib/osal/${OS}>
  $<INSTALL_INTERFACE:include/soem>)

target_include_directories(soem
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/soem_lib/oshw/${OS}>
  $<INSTALL_INTERFACE:include/soem>
  )

message(STATUS "LIB_DIR: ${SOEM_LIB_INSTALL_DIR}")

include_directories(
  inc
)

add_library(kistar_hand_base STATIC
  src/Ethercat_setting.cpp
  src/fk_solver.cpp
  src/hdf5_logger.cpp
  src/shm.cpp
  src/shm_print.cpp)
target_link_libraries(kistar_hand_base PUBLIC soem ${HDF5_CXX_LIBRARIES} ${HDF5_LIBRARIES})
target_include_directories(kistar_hand_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inc>
  $<INSTALL_INTERFACE:include>)
target_include_directories(kistar_hand_base PRIVATE ${HDF5_INCLUDE_DIRS})
target_compile_definitions(kistar_hand_base PRIVATE
  KISTAR_PROJECT_ROOT=\"${CMAKE_CURRENT_LIST_DIR}\")

set(KISTAR_FRANKA_ENABLED FALSE)
set(KISTAR_FRANKA_STATUS_MSG "[KISTAR] Franka package NOT found: Franka module disabled")

# 시스템 libfranka는 보통 Pinocchio 3.4.x에 빌드됨. 3.4가 없으면 번들 libfranka를 자동 사용.
option(USE_BUNDLED_LIBFRANKA "Force use bundled libfranka instead of system" OFF)

find_package(Franka QUIET)
set(KISTAR_USE_SYSTEM_FRANKA FALSE)
if(Franka_FOUND AND NOT USE_BUNDLED_LIBFRANKA)
  find_package(pinocchio QUIET)
  # 시스템 libfranka는 libpinocchio.so.3.4.0을 요구함 → 3.4.x일 때만 시스템 Franka 사용
  if(pinocchio_FOUND AND pinocchio_VERSION VERSION_GREATER_EQUAL "3.4" AND pinocchio_VERSION VERSION_LESS "3.5")
    set(KISTAR_USE_SYSTEM_FRANKA TRUE)
  else()
    if(pinocchio_FOUND)
      message(STATUS "System Franka needs Pinocchio 3.4.x; found ${pinocchio_VERSION} -> using bundled libfranka")
    else()
      message(STATUS "System Franka needs Pinocchio at link time; not found -> using bundled libfranka")
    endif()
  endif()
endif()

if(KISTAR_USE_SYSTEM_FRANKA)
  find_package(Eigen3 REQUIRED)
  set(KISTAR_FRANKA_LINK_PINOCCHIO TRUE)
  set(KISTAR_FRANKA_ENABLED TRUE)
  set(KISTAR_FRANKA_TARGET Franka::Franka)
  set(KISTAR_FRANKA_STATUS_MSG "[KISTAR] Franka package FOUND (system): Franka module enabled")
  message(STATUS "Franka module enabled from system package")
elseif(EXISTS ${CMAKE_CURRENT_LIST_DIR}/libfranka/CMakeLists.txt)
  message(STATUS "Using bundled libfranka (system Franka needs Pinocchio 3.4.x)...")
  find_package(Eigen3 QUIET)
  find_package(Poco QUIET COMPONENTS Net Foundation)
  find_package(pinocchio QUIET)
  if(Eigen3_FOUND AND Poco_FOUND AND pinocchio_FOUND)
    set(BUILD_DOCUMENTATION OFF CACHE BOOL "Disable bundled libfranka docs" FORCE)
    set(BUILD_EXAMPLES OFF CACHE BOOL "Disable bundled libfranka examples" FORCE)
    set(BUILD_TESTS OFF CACHE BOOL "Disable bundled libfranka tests" FORCE)
    set(BUILD_TESTS OFF)
    add_subdirectory(libfranka EXCLUDE_FROM_ALL)
    if(TARGET franka)
      set(KISTAR_FRANKA_ENABLED TRUE)
      set(KISTAR_FRANKA_TARGET franka)  # 번들 빌드 타깃만 사용 (시스템 Franka::Franka와 충돌 방지)
      set(KISTAR_FRANKA_STATUS_MSG "[KISTAR] Franka package FOUND (bundled): Franka module enabled")
      message(STATUS "Franka module enabled from bundled libfranka")
    else()
      message(STATUS "Bundled libfranka added, but Franka::Franka target is missing")
    endif()
  else()
    set(KISTAR_FRANKA_STATUS_MSG "[KISTAR] Bundled libfranka skipped: missing deps (Eigen3/Poco/pinocchio)")
    message(STATUS "Bundled libfranka skipped. Missing one of: Eigen3, Poco(Net/Foundation), pinocchio")
  endif()
else()
  message(STATUS "Franka package not found and bundled libfranka directory is missing")
endif()

if(KISTAR_FRANKA_ENABLED)
  target_sources(kistar_hand_base PRIVATE
    src/franka_right_arm.cpp
    libfranka/examples/examples_common.cpp)
  if(NOT TARGET Eigen3::Eigen3)
    add_library(Eigen3::Eigen3 INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen3 PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIRS}")
  endif()
  set(KISTAR_FRANKA_LIBS ${KISTAR_FRANKA_TARGET} Eigen3::Eigen3)
  if(KISTAR_FRANKA_LINK_PINOCCHIO AND TARGET pinocchio::pinocchio)
    list(APPEND KISTAR_FRANKA_LIBS pinocchio::pinocchio)
  endif()
  target_link_libraries(kistar_hand_base PUBLIC ${KISTAR_FRANKA_LIBS})
  target_include_directories(kistar_hand_base PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/libfranka/examples)
  target_compile_definitions(kistar_hand_base PUBLIC KISTAR_HAS_FRANKA=1)
endif()

add_custom_target(print_franka_status ALL
  COMMAND ${CMAKE_COMMAND} -E echo "${KISTAR_FRANKA_STATUS_MSG}"
  COMMENT "Printing Franka package detection status"
  VERBATIM)


install(FILES
  ${SOEM_HEADERS}
  ${OSAL_HEADERS}
  ${OSHW_HEADERS}
  DESTINATION ${SOEM_INCLUDE_INSTALL_DIR})

install(FILES
  ${KISTAR_HEADERS}
  DESTINATION include)

if(KISTAR_BUILD_TESTS)
  add_subdirectory(test)
endif()
