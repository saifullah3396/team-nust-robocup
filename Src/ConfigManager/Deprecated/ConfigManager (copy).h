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

#include "Utils/BasicUtils.h"
#include "Utils/DataUtils.h"
#include "Utils/VariadicMacros.h"

using namespace Utils;

namespace Configuration
{
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
    ConfigManager();

    /**
     * @brief Class destructor.
     */
    ~ConfigManager();

    /**
     * @brief Setter for config file.
     * @param config Name of the config file to be opened
     */
    void
    setConfig(const string& config)
    {
      this->config = config;
    }

    /**
     * @brief Parses the config file and maps string name to string
     *   value of the content data.
     * @return void
     */
    void
    parseFile();

    /**
     * @brief Returns the value of the given key.
     * @param key Name of the key
     * @return valueType Value of the key
     */
    template<typename valueType>
      valueType
      getValueOfKey(const string& key) const
      {
        ASSERT(keyExists(key));
        valueType var;
        DataUtils::stringToVar(contents.find(key)->second, var);
        return var;
      }

    //! Returns path to the directory of configuration files.
    string
    getConfigDirPath()
    {
      return configDirPath;
    }

  private:
    /**
     * @brief Removes comments written ahead of ";" from a given string.
     * @param str Input string
     * @return void
     */
    inline void
    removeComment(string& str);

    /**
     * @brief Returns true if the string consists only of empty spaces.
     * @param str Input string
     * @return boolean
     */
    inline bool
    strEmpty(const string& str) const;

    /**
     * @brief Returns true if the string consists of a comment.
     * @param str Input string
     * @return boolean
     */
    inline bool
    strIsComment(const string& str) const;

    /**
     * @brief Parses a line read from the config file.
     * @param str The line string
     * @param lineNo The line number
     * @return void
     */
    inline void
    parseLine(const string& str, const size_t& lineNo);

    /**
     * @brief Returns true if a string consists of a valid key/value pair.
     * @param str Input string
     * @return boolean
     */
    inline bool
    validPair(const string& str) const;

    /**
     * @brief Extracts the key/value pair from a given string.
     * @param str Input string
     * @return void
     */
    inline void
    extractContents(const string& str);

    /**
     * @brief Extracts the key of a key/value pair from a given string
     *   marked by string before "=" sign.
     * @param key Name of the key
     * @param sepPos End of substring
     * @param str Input string
     * @return void
     */
    inline void
    extractKey(string& key, const size_t& sepPos, const string& str) const;

    /**
     * @brief Extracts the value of a key/value pair from a given string
     *   marked by string after "=" sign.
     * @param value Value as string
     * @param sepPos End of the substring
     * @param str Input string
     * @return void
     */
    inline void
    extractValue(string& value, const size_t& sepPos, const string& str) const;

    /**
     * @brief Returns true if the given key exists in contents.
     * @param key Name of the key
     * @return boolean
     */
    bool
    keyExists(const string& key) const;

    //! Map of stringified key/value pairs extracted from the config file.
    map<string, string> contents;

    //! Name of the config file.
    string config;

    //! Path to the directory of configuration files.
    string configDirPath;
  };

}
#endif //! _CONFIG_MANAGER_H_
