# Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
