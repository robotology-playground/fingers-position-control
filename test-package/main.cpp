/******************************************************************************
*                                                                            *
* Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
* All Rights Reserved.                                                       *
*                                                                            *
******************************************************************************/

/**
* @authors: Ugo Pattacini <ugo.pattacini@iit.it>
*/

#include <cstdlib>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Value.h>
#include <yarp/os/ResourceFinder.h>
#include "FingersPositionControl/HandController.h"

int main(int argc, char *argv[]) {
  yarp::os::Network yarp;
  if (!yarp.checkNetwork()) {
    yError()<<"Unable to find YARP network";
    return EXIT_FAILURE;
  }

  yarp::os::ResourceFinder rf;
  rf.setDefault("hand", "right");
  rf.setDefault("file", "config.ini");
  rf.configure(argc, argv);

  yInfo()<<"Declaring the controller";
  fingersPositionControl::HandController ctrl;
  ctrl.set("hand", yarp::os::Value(rf.find("hand").asString()));
  ctrl.set("FingersPositionControl", rf.find("file").asString());

  yInfo()<<"Opening the controller";
  if (ctrl.open()) {
    yInfo()<<"Closing the hand";
    if (ctrl.closeHand()) {
      yInfo()<<"Opening the hand";
      if (ctrl.openHand(true)) {
        yInfo()<<"Closing the controller";
        if (ctrl.close()) {
          return EXIT_SUCCESS;
        }
      }
    }
  }

  return EXIT_FAILURE;
}
