# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr"
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build"
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/temp_install"
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/tmp"
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp"
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src"
  "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/sangam/PX4-Autopilot/build/px4_sitl_default/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp${cfgdir}") # cfgdir has leading slash
endif()
