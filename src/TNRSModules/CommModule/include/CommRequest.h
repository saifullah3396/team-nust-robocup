/**
 * @file CommModule/include/CommRequest.h
 *
 * This file defines the class CommRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "CommModule/include/CommMessage.h"

using namespace cv;

/**
 * Types of request valid for CommModule
 * 
 * @enum CommRequestIds
 */ 
enum class CommRequestIds {
  SEND_MSG_REQUEST,
  SEND_IMAGE_REQUEST
};

/**
 * @class CommRequest
 * @brief A module request that can be handled by CommModule
 */ 
class CommRequest : public ModuleRequest 
{
public:
  /**
   * Constructor
   * 
   * @param id: Id of the control request
   */ 
  CommRequest(const CommRequestIds& id) :
    ModuleRequest((unsigned)TNSPLModules::COMM, (unsigned)id)
  {
  }
};
typedef boost::shared_ptr<CommRequest> CommRequestPtr;

/**
 * @class SendMsgRequest
 * @brief A request that holds a message to be sent over the network
 */ 
struct SendMsgRequest : public CommRequest
{
  /**
   * Constructor
   * 
   * @param cMsg: The comm message associated with this request
   */ 
  SendMsgRequest(const CommMessage& cMsg) :
    CommRequest(CommRequestIds::SEND_MSG_REQUEST),
    cMsg(cMsg)
  {
  }

  CommMessage cMsg;
};
typedef boost::shared_ptr<SendMsgRequest> SendMsgRequestPtr;

/**
 * @class SendImageRequest
 * @brief A request that holds an image to be sent over the network
 */ 
struct SendImageRequest : public CommRequest
{
  /**
   * Constructor
   */ 
  SendImageRequest(const Mat& image) :
    CommRequest(CommRequestIds::SEND_IMAGE_REQUEST),
    image(image)
  {
  }

  Mat image;
};
typedef boost::shared_ptr<SendImageRequest> SendImageRequestPtr;


