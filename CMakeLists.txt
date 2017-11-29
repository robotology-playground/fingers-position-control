# Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

cmake_minimum_required(VERSION 3.1)
project(FingersPositionControl LANGUAGES CXX VERSION 1.0.0.0)

include(GNUInstallDirs)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(YarpInstallationHelpers)
include(ICUBcontribOptions)
include(ICUBcontribHelpers)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
option(BUILD_TESTING "Create tests using CMake" OFF)
option(ENABLE_RPATH "Enable RPATH for this library" ON)

if(BUILD_TESTING)
    enable_testing()
endif()

mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                          DEPENDS ENABLE_RPATH
                          USE_LINK_PATH)

if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
      message(STATUS "Setting build type to 'Release' as none was specified.")
      set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
  endif()
endif()

icubcontrib_set_default_prefix()

add_subdirectory(src)
add_subdirectory(app)

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME} VARS_PREFIX ${PROJECT_NAME}
                                            VERSION ${${PROJECT_NAME}_VERSION}
                                            COMPATIBILITY AnyNewerVersion
                                            TARGETS_PROPERTY ${PROJECT_NAME}_TARGETS
                                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)
include(AddUninstallTarget)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()
