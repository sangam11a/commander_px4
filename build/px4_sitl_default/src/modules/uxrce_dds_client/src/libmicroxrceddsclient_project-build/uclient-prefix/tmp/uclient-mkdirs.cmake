# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/sangam/PX4-Autopilot/src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client"
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build"
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix"
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix/tmp"
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix/src/uclient-stamp"
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix/src"
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix/src/uclient-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix/src/uclient-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/uclient-prefix/src/uclient-stamp${cfgdir}") # cfgdir has leading slash
endif()
