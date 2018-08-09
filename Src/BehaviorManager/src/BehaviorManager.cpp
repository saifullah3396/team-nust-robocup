/**
 * @file BehaviorManager/src/BehaviorManager.cpp
 *
 * This file implements the class BehaviorManager and BManagerException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "BehaviorManager/include/Behavior.h"
#include "BehaviorManager/include/BehaviorRequest.h"
#include "BehaviorManager/include/BehaviorManager.h"

BManagerException::BManagerException(
  BehaviorManager* behaviorManager,
  const string& message,
  const bool& bSysMsg,
  const BManagerExceptionType& type) throw () :
  TNRSException(message, bSysMsg),
  name(behaviorManager->getName()),
  type(type)
{
}

void BehaviorManager::manageRequest(
  const BehaviorRequestPtr& req)
{
  manageRequest(currBehavior, req);
}

void BehaviorManager::manageRequest(
  BehaviorPtr& bPtr,
  const BehaviorRequestPtr& req)
{
  try {
    if (!req->getReqConfig()) {
      throw BManagerException(
        this,
        "Requested behavior has no configuration associated with it.",
        false,
        EXC_BEHAVIOR_SETUP_FAILED);
    }
    auto reqConfig = req->getReqConfig();
    //! Requested behavior configuration is valid or not
    if (!reqConfig->isValid()) {
      req->setReceived(true);
      return;
    }
    //! Behavior already exists
    if (bPtr) {
      //if (name == "PlanningBehavior")
      //  cout << "Behavior already running" << endl;
      //! Check if the requested behavior is the same as the behavior to
      //! be managed
      if (reqConfig->id == bPtr->getBehaviorConfig()->id &&
          reqConfig->type == bPtr->getBehaviorConfig()->type)
      {
        //if (name == "PlanningBehavior")
        //  cout << "Requested is same as previous" << endl;
        //! Reinitiate the current behavior with new requested
        //! configuration
        bPtr->reinitiate(reqConfig);
        req->setAccepted(true);
        //lastBehaviorAccepted.id = req->id;
        //lastBehaviorAccepted.state =
        //  boost::make_shared < BehaviorState > (bId, this->behavior->getState());
        //updateBehaviorState();
      }
    } else {
      //! Set up the current behavior with the requested configuration
      if(setupBehavior(bPtr, reqConfig)) {
        req->setAccepted(true);
      }
    }
  } catch (BManagerException& e) {
    cout << e.what();
  }
  req->setReceived(true);
  //lastBehaviorAccepted.id = req->id;
  //cout << "Last behavior id: " << lastBehaviorAccepted.id << endl;
  //lastBehaviorAccepted.state =
  //  boost::make_shared < BehaviorState > (bId, this->behavior->getState());
  //updateBehaviorState();
}

bool BehaviorManager::setupBehavior(
  BehaviorPtr& bPtr,
  const BehaviorConfigPtr& cfg) throw (BManagerException)
{
  try {
  //! Make a behavior based on the abstract function call
    if (!makeBehavior(bPtr, cfg)) {
      throw BManagerException(
        this,
        "Requested behavior is undefined.",
        false,
        EXC_BEHAVIOR_SETUP_FAILED);
    }
  } catch (BManagerException& e) {
    cout << e.what();
    return false;
  }
  bPtr->setupExternalConfig();
  //bPtr->setBehaviorConfig(reqConfig);
  return true;
}

void
BehaviorManager::update()
{
  //! Kill the behavior if requested
  if (killRequested) {
    if (currBehavior) {
      currBehavior->kill();
    }
    killRequested = false;
  }
  
  //! If a current behavior exists
  if (currBehavior) {
	//! Update the behavior
    updateBehavior(currBehavior);
    ////lastBehaviorAccepted.state->finished = true;
    ////updateBehaviorState();
  }
}

void BehaviorManager::updateBehavior(BehaviorPtr& bPtr)
{
  //! Check if top behavior time limit is reached and kill it along
  //! with all its childs if true
  bPtr->checkBehaviorTimeLimit();

  //! Get pointer to child
  auto& child = bPtr->getChild();

  //! Setup a new child if a new child request is found
  //! This call does nothing if there is no request
  if (!child) {
    auto& childReq = bPtr->getChildRequest();
    if (childReq) {
      manageRequest(child, bPtr->getChildRequest());
      if (child) {
        bPtr->setLastChildCfg(bPtr->getChildRequest()->getReqConfig());
      }
      childReq.reset();
    }
  }
	
  //! If a child exists now
  if (child) {
    //! Update the child
    updateBehavior(child);
  } else {
    //! Initiate if not, update otherwise
    bPtr->manage();
    
    //! If behavior is initiated but still not valid for update
    if (bPtr->isInitiated() && !bPtr->isRunning()) {
      bPtr.reset(); //! Behavior is finished so delete it
    }
    
    setBehaviorInfo(bPtr);
  }
}

void BehaviorManager::setBehaviorInfo(const BehaviorPtr& bPtr)
{
  if (bPtr) {
    behaviorInfo->setInitiated(bPtr->isInitiated());
    behaviorInfo->setRunning(bPtr->isRunning());
    behaviorInfo->setConfig(bPtr->getBehaviorConfig());
  } else {
    behaviorInfo->setInitiated(false);
    behaviorInfo->setRunning(false);
    behaviorInfo->resetConfig();
  }
}
