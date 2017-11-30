# Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

cmake_minimum_required(VERSION 3.1)
project(FingersPositionControl_test-package)

find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(FingersPositionControl REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} FingersPositionControl::FingersPositionControl)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

icubcontrib_add_uninstall_target()
