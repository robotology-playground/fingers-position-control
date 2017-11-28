/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */
 
#ifndef LIB_FINGERS_POSITION_CONTROL_H
#define LIB_FINGERS_POSITION_CONTROL_H

#include <string>
#include <yarp/os/Value.h>
#include <iCub/action/actionPrimitives.h>

namespace tactileControl {

class HandController
{
	iCub::action::ActionPrimitives *action;
	std::string hand;

	std::string context;
	std::string file;

	bool isOpen() const;

public:
	HandController();
	virtual ~HandController();

	virtual bool open();
	virtual bool close();

	virtual bool set(const std::string &context, const std::string &file);
	virtual bool set(const std::string &key, const yarp::os::Value &value);

	virtual bool closeHand(const bool wait = true);
	virtual bool isHandClose();

	virtual bool openHand(const bool fullyOpen, const bool wait = true);
	virtual bool isHandOpen();
};

}

#endif
