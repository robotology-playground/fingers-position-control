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

#include "TactileControl/HandController.h"

int main() {
  tactileControl::HandController ctrl;

  ctrl.set("hand", "right");
  ctrl.set("FingersPositionControl", "config_sim.ini");

  if (ctrl.open()) {
    if (ctrl.closeHand()) {
      if (ctrl.openHand(true)) {
        if (ctrl.close()) {
          return EXIT_SUCCESS;
        }
      }
    }
  }

  return EXIT_FAILURE;
}
