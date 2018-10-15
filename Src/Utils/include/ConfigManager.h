/**
 * @file Configuration/ConfigManager.h
 *
 * This file declares the class ConfigManager.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#ifndef _CONFIG_MANAGER_H_
#define _CONFIG_MANAGER_H_

#include <boost/exception/diagnostic_information.hpp> 
#include <boost/exception/all.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <string>
#include "Utils/include/DebugUtils.h"
#include "Utils/include/VariadicMacros.h"

using namespace std;

/**
 * @class ConfigManager
 * @brief Class for parsing the contents of a config file.
 */
class ConfigManager
{
public:
  /**
   * @brief Class constructor.
   */
  ConfigManager()
  {
  }

  /**
   * @brief Class destructor.
   */
  ~ConfigManager()
  {
  }

  /**
   * @brief Parses an .ini cfg file based on boost libraries.
   */
  void
  parseFile(const string& cfgFile);

  /**
   * @brief Returns the value of the given key.
   * @param key Name of the key
   * @return valueType Value of the key
   */
  template<typename valueType>
  valueType
	getValueOfKey(const string& key) const;

  //! Returns path to the directory of configuration files.
  static string
  getConfigDirPath()
  {
    return configDirPath;
  }

  //! Returns path to the directory of log  files.
  static string
  getLogsDirPath()
  {
    return logsDirPath;
  }
  
  ///! Returns Ppth to robot directory.
  static string
  getRobotDirPath()
  {
    return robotDirPath;
  }

  //! Sets the path to the directory of configuration/logs/robot files.
  static void
  setDirPaths(const string& robotDir)
  {
	#ifdef MODULE_IS_REMOTE
			robotDirPath = robotDir;
			logsDirPath = logsDirPath + robotDir;
			configDirPath = configDirPath + robotDir;
	#endif
  }
private:
  //! Path to the directory of configuration files.
  static string configDirPath;
  
  //! Path to the directory of log  files.
  static string logsDirPath;
  
  //! Path to robot directory.
  static string robotDirPath;

  //! The container that holds the parsed configuration parameters.
  boost::property_tree::ptree iniConfig;

  //! The config file path
  string cfgFile;
};
#endif //! _CONFIG_MANAGER_H_
