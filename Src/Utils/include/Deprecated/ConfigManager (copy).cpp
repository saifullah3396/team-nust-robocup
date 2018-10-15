/**
 * @file Configuration/ConfigManager.h
 *
 * This file implements the class ConfigManager.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include "ConfigManager.h"

namespace Configuration
{

  ConfigManager::ConfigManager()
  {
#ifndef MODULE_IS_REMOTE
    configDirPath = GET_STRINGS("/home/nao/Config/");
#else
    configDirPath = GET_STRINGS(ROOT_DIR "/../Config/");
#endif
  }

  ConfigManager::~ConfigManager()
  {
  }

  void
  ConfigManager::parseFile()
  {
    try {
      ifstream file;
      file.open(config.c_str());
      if (!file) throw("Configuration file not found.\n");
      string str;
      size_t lineNo = 0;
      while (getline(file, str)) {
        lineNo++;
        string temp = str;
        if (temp.empty()) continue;
        removeComment(temp);
        if (strEmpty(temp)) continue;
        if (strIsComment(temp)) continue;
        parseLine(temp, lineNo);
      }
      file.close();
    } catch (exception &e) {
      cout << "Exception caught while parsing configuration file " << config << ": " << e.what() << endl;
    }
  }

  void
  ConfigManager::removeComment(string& str)
  {
    try {
      if (str.find(';') != str.npos) str.erase(str.find(';'), str.npos);
    } catch (exception &e) {
      cout << "Exception caught while removing a comment in " << "configuration file " << config << ": " << e.what() << endl;
    }
  }

  bool
  ConfigManager::strEmpty(const string& str) const
  {
    return (str.find_first_not_of(' ') == str.npos);
  }

  bool
  ConfigManager::strIsComment(const string& str) const
  {
    if (str.find('#') != str.npos) return true;
    else return false;
  }

  void
  ConfigManager::parseLine(const string& str, const size_t& lineNo)
  {
    try {
      if (str.find('=') == str.npos) {
        throw("Couldn't find separator on line: " + DataUtils::varToString(
          lineNo) + "in file: " + config + "\n");
      }
      if (!validPair(str)) {
        throw("Bad format for line: " + DataUtils::varToString(lineNo) + "in file: " + config + "\n");
      }
    } catch (exception &e) {
      cout << "Exception caught while parsing a line in " << "configuration file " << config << ": " << e.what() << endl;
    }
    extractContents(str);
  }

  bool
  ConfigManager::validPair(const string& str) const
  {
    try {
      string temp = str;
      temp.erase(0, temp.find_first_not_of("\t "));
      if (temp[0] == '=') return false;
      for (size_t i = temp.find('=') + 1; i < temp.length(); i++)
        if (temp[i] != ' ') return true;
      return false;
    } catch (exception &e) {
      cout << "Exception caught while checking a line validity in " << "configuration file " << config << ": " << e.what() << endl;
    }
  }

  void
  ConfigManager::extractContents(const string& str)
  {
    try {
      string temp = str;
      temp.erase(0, temp.find_first_not_of("\t "));
      size_t sepPos = temp.find('=');
      string key, value;
      extractKey(key, sepPos, temp);
      extractValue(value, sepPos, temp);
      if (!keyExists(key)) {
        contents.insert(pair<string, string>(key, value));
      } else {
        throw("Repetitive key names found: " + key + "in file:" + config + "\n");
      }
    } catch (exception &e) {
      cout << "Exception caught while extracting contents from " << "configuration file " << config << ": " << e.what() << endl;
    }
  }

  void
  ConfigManager::extractKey(string& key, const size_t& sepPos,
    const string& str) const
  {
    try {
      key = str.substr(0, sepPos);
      if (key.find('\t') != str.npos || key.find(' ') != str.npos) key.erase(
        key.find_first_of("\t "));
    } catch (exception &e) {
      cout << "Exception caught while extracting a key from " << "configuration file " << config << ": " << e.what() << endl;
    }
  }

  void
  ConfigManager::extractValue(string& value, const size_t& sepPos,
    const string& str) const
  {
    try {
      value = str.substr(sepPos + 1);
      value.erase(0, value.find_first_not_of("\t "));
      value.erase(value.find_last_not_of("\t ") + 1);
    } catch (exception &e) {
      cout << "Exception caught while extracting a key value from " << "configuration file " << config << ": " << e.what() << endl;
    }
  }

  bool
  ConfigManager::keyExists(const string& key) const
  {
    return contents.find(key) != contents.end();
  }

}
