/**
 * @file Utils/include/JsonLogger.h
 *
 * This file defines the class JsonLogger
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <fstream>
#include "Utils/include/JsonUtils.h"

#define JSON_ASSIGN(logRoot, key, value) logRoot[key] = value;
#define JSON_APPEND(logRoot, key, value) logRoot[key].append(value);

using namespace Eigen;
using namespace std;

namespace Utils
{
  /**
   * @class JsonLogger
   * @brief Class that provides a json based data logger
   */
  class JsonLogger
  {
  public:

    /**
     * @brief Class constructor.
     * 
     * @param path: Path to json file
     * @param root: Root of json object
     */
    JsonLogger(
      const string& path, const Json::Value& root = Json::Value()) :
      path(path), root(root)
    {
    }

    /**
     * @brief Class destructor.
     */
    ~JsonLogger()
    {
      save();
    }
    
    /**
     * @brief Dumps the json object into the file
     */ 
    void save(const bool& fast = false)
    {
      std::ofstream file;
      file.open(path);
      if (fast) {
        Json::FastWriter fastWriter;
        file << fastWriter.write(root);
      } else {
        Json::StyledWriter styledWriter;
        file << styledWriter.write(root);
      }
      file.close();
    }
    
    //! Getters
    const string& getPath() const { return path; }
    Json::Value& getRoot() { return root; }
    
    //! Setters
    void setRoot(const Json::Value& root) { this->root = root; }
    void setPath(const string& root) { this->path = path; }
  protected:  
    //! Behavior specific path to log file
    string path;
    
    //! Root json object which holds the logged data
    Json::Value root;
  };
}
typedef boost::shared_ptr<Utils::JsonLogger> JsonLoggerPtr;
