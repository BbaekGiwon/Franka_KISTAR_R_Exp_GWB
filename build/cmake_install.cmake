# Install script for directory: /home/prime/Franka_KISTAR_R_Exp_GWB_V1.0

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/soem" TYPE FILE FILES
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/soem/ethercat.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/soem/ethercatbase.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/soem/ethercatcoe.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/soem/ethercatconfig.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/soem/ethercatconfiglist.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/soem/ethercatdc.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/soem/ethercateoe.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/soem/ethercatfoe.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/soem/ethercatmain.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/soem/ethercatprint.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/soem/ethercatsoe.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/soem/ethercattype.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/osal/linux/osal_defs.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/osal/osal.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/oshw/linux/nicdrv.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/soem_lib/oshw/linux/oshw.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/inc/Ethercat_setting.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/inc/fk_solver.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/inc/franka_right_arm.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/inc/hdf5_logger.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/inc/shm.h"
    "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/inc/shm_print.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/build/test/cmake_install.cmake")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/build/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/prime/Franka_KISTAR_R_Exp_GWB_V1.0/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
