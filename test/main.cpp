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
#include "TactileControl/HandController.h"

int main() {
  yarp::os::Network yarp;
  if (!yarp.checkNetwork()) {
    yError()<<"Unable to find YARP network";
    return EXIT_FAILURE;
  }

  yInfo()<<"Declaring the controller";
  tactileControl::HandController ctrl;
  ctrl.set("hand", yarp::os::Value("right"));
  ctrl.set("FingersPositionControl", "config_sim.ini");

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
