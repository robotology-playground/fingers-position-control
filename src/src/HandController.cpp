/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <utility>
#include <fstream>

#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include <iCub/perception/models.h>
#include "TactileControl/HandController.h"

using namespace std;
using namespace yarp::os;
using namespace iCub::perception;
using namespace iCub::action;
using namespace tactileControl;

namespace tactileControl {

class ActionPrimitivesHandOnly : public ActionPrimitives {
 protected:
  bool disableArmWaving()override {
    return false;
  }

  void run()override {
    const double t = Time::now();

    if (!handMoveDone) {
      // check whether all the remaining active joints have come
      // to a complete stop
      handMoveDone = isHandSeqEnded();
      if (handMoveDone) {
        if (!handSeqTerminator)
          if (execPendingHandSequences())     // here handMoveDone may switch false again
            motionStartEvent.signal();
      }
    }

    latchArmMoveDone = armMoveDone;
    latchHandMoveDone = handMoveDone;

    if (latchHandMoveDone && (t - latchTimerWait > waitTmo)) {
      // execute action-end callback
      if (actionClb != nullptr) {
        actionClb->exec();
        actionClb = nullptr;
      }

      if (execQueuedAction())
        motionStartEvent.signal();
      else
        motionDoneEvent.signal();
    }
  }

 public:
  ActionPrimitivesHandOnly() : ActionPrimitives() {
  }

  ActionPrimitivesHandOnly(yarp::os::Property& opt) : ActionPrimitives(opt) {
  }

  virtual ~ActionPrimitivesHandOnly() {
  }

  bool open(yarp::os::Property& opt)override {
    if (configured)
      return true;

    if (!opt.check("local"))
      return false;

    robot = opt.check("robot", Value("icub")).asString();
    local = opt.find("local").asString();
    part = opt.check("part", Value("right")).asString();
    verbose = opt.check("verbosity", Value("off")).asString() == "on" ? true : false;
    int period = opt.check("thread_period", Value(50)).asInt();

    if (!configGraspModel(opt)) {
      close();
      return false;
    }

    configHandSeq(opt);

    Property optPolyHand("(device remote_controlboard)");
    optPolyHand.put("remote", ("/" + robot + "/" + part).c_str());
    optPolyHand.put("local", ("/" + local + "/" + part + "/position").c_str());
    if (!polyHand.open(optPolyHand)) {
      close();
      return false;
    }

    polyHand.view(modCtrl);
    polyHand.view(encCtrl);
    polyHand.view(posCtrl);

    jHandMin = 7;
    posCtrl->getAxes(&jHandMax);

    for (int j = jHandMin; j < jHandMax; j++) {
      fingersJnts.push_back(j);
      fingersJntsSet.insert(j);
    }

    fingers2JntsMap.insert(pair<int, int>(0, 8));
    fingers2JntsMap.insert(pair<int, int>(0, 9));
    fingers2JntsMap.insert(pair<int, int>(0, 10));
    fingers2JntsMap.insert(pair<int, int>(1, 11));
    fingers2JntsMap.insert(pair<int, int>(1, 12));
    fingers2JntsMap.insert(pair<int, int>(2, 13));
    fingers2JntsMap.insert(pair<int, int>(2, 14));
    fingers2JntsMap.insert(pair<int, int>(3, 15));
    fingers2JntsMap.insert(pair<int, int>(4, 15));

    Time::turboBoost();
    setRate(period);
    start();

    return configured = true;
  }

  void close()override {
    if (closed)
      return;

    if (isRunning())
      stop();

    if (polyHand.isValid()) {
      stopControl();
      polyHand.close();
    }

    delete graspModel;
    graspModel = nullptr;
    actionsQueue.clear();
    closed = true;
  }

  bool stopControl()override {
    if (configured) {
      suspend();
      clearActionsQueue();
      posCtrl->stop(fingersJnts.size(), fingersJnts.getFirst());
      handMoveDone = latchHandMoveDone = true;
      resume();
      return true;
    } else
      return false;
  }

  bool checkActionsDone(bool& f, const bool sync)override {
    if (configured) {
      if (sync && checkEnabled) {
        motionDoneEvent.reset();
        motionDoneEvent.wait();
      }

      f = latchHandMoveDone;
      return true;
    } else
      return false;
  }

  bool checkActionOnGoing(bool& f, const bool sync)override {
    if (configured) {
      if (sync && checkEnabled) {
        motionStartEvent.reset();
        motionStartEvent.wait();
      }

      f = !latchHandMoveDone;
      return true;
    } else
      return false;
  }
};

}

HandController::HandController() : action(nullptr),
    hand("left"),
    context("FingersPositionControl"),
    file("config.ini") {
}

HandController::~HandController() {
  delete action;
}

bool HandController::isOpen() const {
  if (action != nullptr)
    return action->isValid();
  else
    return false;
}

bool HandController::open() {
  ResourceFinder rf;
  rf.setDefaultContext(context);
  rf.configure(0, nullptr);

  Property config;
  string fileName(hand + "_" + file);
  config.fromConfigFile(rf.findFileByName(fileName));

  string grasp_model_file=rf.findFileByName(config.find("grasp_model_file").asString());
  string hand_sequences_file=rf.findFileByName(config.find("hand_sequences_file").asString());

  Property configRelocated=config;
  configRelocated.unput("grasp_model_file");
  configRelocated.unput("hand_sequences_file");
  configRelocated.put("grasp_model_file",grasp_model_file);
  configRelocated.put("hand_sequences_file",hand_sequences_file);

  action = new tactileControl::ActionPrimitivesHandOnly(configRelocated);
  if (action->isValid()) {
    Model* model;
    action->getGraspModel(model);
    if (model != nullptr) {
      if (!model->isCalibrated()) {
        Property prop;
        prop.put("finger", "all");
        model->calibrate(prop);

        ofstream fout;
        fout.open((rf.getHomeContextPath() + "/" + fileName).c_str());
        model->toStream(fout);
        fout.close();

        return true;
      }
    }
  }

  delete action;
  action = nullptr;
  return false;
}

bool HandController::close() {
  if (isOpen()) {
    delete action;
    action = nullptr;
    return true;
  } else
    return false;
}

bool HandController::set(const string& context, const string& file) {
  this->context=context;
  this->file=file;
  return true;
}

bool HandController::set(const string& key, const Value& value) {
  if ((key == "hand") && value.isString()) {
    string hand = value.asString();
    if ((hand == "left") || (hand == "right")) {
      this->hand = hand;
      return true;
    }
  }

  return false;
}

bool HandController::closeHand(const bool wait) {
  if (isOpen()) {
    if (!action->pushAction("close_hand"))
      return false;
    if (wait) {
      bool f;
      action->checkActionsDone(f, true);
    }
    return true;
  } else
    return false;
}

bool HandController::isHandClose() {
  if (isOpen()) {
    bool f;
    action->checkActionsDone(f);
    return f;
  } else
    return false;
}

bool HandController::openHand(const bool fullyOpen, const bool wait) {
  if (isOpen()) {
    if (!action->pushAction(fullyOpen ? "open_hand_full" : "open_hand"))
      return false;
    if (wait) {
      bool f;
      action->checkActionsDone(f, true);
    }
    return true;
  } else
    return false;
}

bool HandController::isHandOpen() {
  return isHandClose();
}
