/**
 * @file Utils/include/JsonUtils.h
 *
 * This file defines the class JsonUtils
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#ifndef _JSON_UTILS_H_
#define _JSON_UTILS_H_

#include <iostream>
#include <jsoncpp/json/json.h>
#include "Eigen/Dense"

#define JSON_ASSIGN(logRoot, key, value) logRoot[key] = value;
#define JSON_APPEND(logRoot, key, value) logRoot[key].append(value);

using namespace Eigen;
using namespace std;

namespace Utils
{
  /**
   * @class JsonUtils
   * @brief Class that provides functions for handling Json objects
   */
  class JsonUtils
  {
  public:

    /**
     * @brief Class constructor.
     */
    JsonUtils()
    {
    }

    /**
     * @brief Class destructor.
     */
    ~JsonUtils()
    {
    }
    
    /**
     * Creates a json object from a eigen matrix
     * 
     * @param mat: Input matrix
     * 
     * @return Json::Value
     */ 
    template<typename Derived>
    static inline Json::Value MatrixToJson(const MatrixBase<Derived>& mat)
    {
      Json::Value jsonMat;
      for (int i = 0; i < mat.rows(); ++i) {
        jsonMat.append(Json::Value::null);
        for (int j = 0; j < mat.cols(); ++j) {
          jsonMat[i].append(mat(i ,j));
        }
      }
      return jsonMat;
    } 
  };
}
#endif //! _JSON_UTILS_H_
