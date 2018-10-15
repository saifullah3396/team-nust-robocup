/**
 * @file Configuration/ConfigManager.h
 *
 * This file implements the class ConfigManager.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include "Utils/include/ConfigManager.h"

#ifndef MODULE_IS_REMOTE
string ConfigManager::configDirPath = GET_STRINGS("/home/nao/Config/");
string ConfigManager::logsDirPath = GET_STRINGS("/home/nao/Logs/");
string ConfigManager::robotDirPath = "";
#else
string ConfigManager::configDirPath = GET_STRINGS(ROOT_DIR "/../Config/");
string ConfigManager::logsDirPath = GET_STRINGS(ROOT_DIR "/../Logs/");
string ConfigManager::robotDirPath = "";
#endif

void
ConfigManager::parseFile(const string& cfgFile)
{
  try {
    this->cfgFile = cfgFile;
    boost::property_tree::ini_parser::read_ini(cfgFile, iniConfig);
  } catch (boost::exception &e) {
    ERROR("Error while reading file: " << cfgFile);
  }
}

template<typename valueType>
valueType
ConfigManager::getValueOfKey(const string& key) const
{
	valueType var = valueType();
	try {
	var = iniConfig.get < valueType > (key);
	} catch (boost::exception &e) {
	ERROR(
		"Error while reading the key: " << key << " from file: " << cfgFile);
	}
	return var;
}

template bool ConfigManager::getValueOfKey<bool>(const string& key) const;
template int ConfigManager::getValueOfKey<int>(const string& key) const;
template unsigned ConfigManager::getValueOfKey<unsigned>(const string& key) const;
template float ConfigManager::getValueOfKey<float>(const string& key) const;
template double ConfigManager::getValueOfKey<double>(const string& key) const;
template string ConfigManager::getValueOfKey<string>(const string& key) const;
