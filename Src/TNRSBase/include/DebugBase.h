/**
 * @file TNRSBase/include/DebugBase.h
 *
 * This file implements the class DebugBase.
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 09 Nov 2017  
 */

#pragma once

#include "Utils/include/DataUtils.h"
#include "Utils/include/Variable.h"

/**
 * \def DVAR(type, var)
 *
 * This macro gets the access to the local debug variable.
 *
 * @param type: Type of the variable.
 * @param var: Index of the variable.
 */
#define DVAR(type, var, value) static_cast<Variable<type>*>(variables[var])->getAccess()

/**
 * \def DVAR(type, var)
 *
 * This macro sets the value of the local debug variable.
 *
 * @param type: Type of the variable.
 * @param var: Index of the variable.
 */
#define SET_DVAR(type, var, value) static_cast<Variable<type>*>(variables[var])->setValue(value)

/**
 * \def DVAR(type, var)
 *
 * This macro returns the value of the local debug variable.
 *
 * @param type: Type of the variable.
 * @param var: Index of the variable.
 */
#define GET_DVAR(type, var) static_cast<Variable<type>*>(variables[var])->getValue()

/**
 * \def DEFINE_VARIABLE_2(type, name, value)
 * This macro takes the inputs type, name, value sends them to 
 * DEFINE_VARIABLE()
 *
 * @param type: Variable type
 * @param name: Variable name
 * @param type: Variable value
 */
#define DEFINE_VARIABLE_2(type, name, value) \
  DEFINE_VARIABLE(type, name, value)

/**
 * \def DEFINE_VARIABLE_1(TYPE_VAR)
 * This macro takes a single input variable (type, name, value), removes
 * paranthesis and sends it to DEFINE_VARIABLE_1()
 *
 * @param TYPE_VAR: (type, name, value) in paranthesis.
 */
#define DEFINE_VARIABLE_1(TYPE_VAR) \
  DEFINE_VARIABLE_2( \
    M_GET_ELEM(0,UNPAREN(TYPE_VAR)), \
    M_GET_ELEM(1,UNPAREN(TYPE_VAR)), \
    M_GET_ELEM(2,UNPAREN(TYPE_VAR)) \
  )

/**
 * \def INIT_DEBUG_BASE_(...)
 * 
 * Wrapper for macro INIT_DEBUG_BASE_()
 * 
 * @param ... : (type, name), (type, name), ... (typelast, namelast), .
 */
#define INIT_DEBUG_BASE(...) INIT_DEBUG_BASE_(__VA_ARGS__)

#define PUSH_TO_MAP_2(name) varMap.insert(pair<string, DebugVar>(#name, name));
#define PUSH_TO_MAP_1(name) PUSH_TO_MAP_2(name)
#define PUSH_TO_MAP(TYPE_VAR) \
  PUSH_TO_MAP_1( \
    M_GET_ELEM(1,UNPAREN(TYPE_VAR)) \
  )

#define VALUE_TO_VAR_2(type, name) \
	if(varName == #name) { \
		Variable<type>* var =  static_cast<Variable<type>* >(variables[varMap[#name]]); \
		type varValue = var->getValue(); \
		DataUtils::stringToVar(varValueStr, varValue); \
		var->setValue(varValue); \
		return true; \
	}

#define VALUE_TO_VAR_1(type, name) \
	VALUE_TO_VAR_2(type, name)

#define VALUE_TO_VAR(TYPE_VAR) \
  VALUE_TO_VAR_1( \
	M_GET_ELEM(0,UNPAREN(TYPE_VAR)), \
	M_GET_ELEM(1,UNPAREN(TYPE_VAR)) \
)

/**
 * \def INIT_DEBUG_BASE_(...)
 * 
 * This macro initiates the debug base class with given debug variables.
 * 
 * @param ... : (type, name), (type, name), ... (typelast, namelast), .
 */
#define INIT_DEBUG_BASE_(...) \
protected: \
    /** \
     * Enumerator for all the debug variables of the class \
     * \
     * @enum DebugVar \
     */ \
	DEFINE_ENUM(DebugVar, 0, SEPARATE(__VA_ARGS__), numDebug); \
	void initDebugBase() { \
		variables.resize(numDebug); \
		FOR_EACH(DEFINE_VARIABLE_1, __VA_ARGS__) \
		FOR_EACH(PUSH_TO_MAP, __VA_ARGS__) \
	} \
	\
	bool updateDebugVar(string varName, string varValueStr) { \
		FOR_EACH(VALUE_TO_VAR, __VA_ARGS__) \
		return false; \
	}

/**
 * @class DebugBase
 * @brief A class that initiates a debug interface for the child class.
 */
class DebugBase
{
public:
  /**
   * Default constructor for this class.
   */
  DebugBase(const string& name, DebugBase* ptr);

  /**
   * Default destructor for this class.
   */
  virtual
  ~DebugBase()
  {
    for (auto it = variables.begin(); it != variables.end(); ++it) {
      delete *it;
    }
  }
  ;

  /**
   * Initiates the debug base with given debug variables. 
   * Use macro INIT_DEBUG_BASE in child class for definition.
   */
  virtual void
  initDebugBase() = 0;

  /**
   * Updates the given debug variable with value provided as string.
   * Use macro INIT_DEBUG_BASE in child class for definition.
   */
  virtual bool
  updateDebugVar(string varName, string varValueStr) = 0;

  /**
   * This function processes the incoming message, finds which class
   * it corresponds to and assigns input values to the debugInfo 
   * variables of the class.
   * 
   * @param msg: Input message recieved from the debugger.
   */
  static string
  processDebugMsg(const string& msg)
  {
    string inStr;
    inStr.assign(msg);
    inStr.erase(remove_if(inStr.begin(), inStr.end(), ::isspace), inStr.end());
    vector < string > parts = DataUtils::splitString(inStr, ':');
    if (parts.size() < 2) return string(
      "Debug response: Invalid message sytax.");
    string className = parts[0];
    if (classMap.find(className) == classMap.end()) return string(
      "Debug response: Specified class does not exist in debug layer.");
    DebugBase* classPtr;
    pthread_mutex_lock(&classMapAccessMutex);
    classPtr = classMap[className];
    pthread_mutex_unlock(&classMapAccessMutex);
    if (parts.size() == 2) {
      if (parts[1][0] == '{') {
        string tmp = parts[1].substr(1, parts[1].size() - 2);
        vector < string > values;
        string value = string("");
        bool start = false;
        for (auto it = tmp.begin(), end = tmp.end(); it != tmp.end(); ++it) {
          if (*it == '{') start = true;
          if (*it == '}') start = false;
          value += *it;
          if (!start) {
            if (*it == ',') {
              values.push_back(value.substr(0, value.size() - 1));
              value.clear();
            } else if (next(it) == tmp.end()) {
              values.push_back(value);
              value.clear();
            }
          }
        }
        if (classPtr->varMap.size() != values.size()) {
          return string(
            "Debug response: Invalid number of variable inputs for specified class.");
        }
        for (int i = 0; i < classPtr->variables.size(); ++i) {
          if (!classPtr->updateDebugVar(
            classPtr->variables[i]->getVariableName(),
            values[i])) return string(
            "Debug response: Invalid value specified for variable: " + classPtr->variables[i]->getVariableName());
        }
        return string("Success");
      } else {
        return string(
          "Debug response: Missing trailing '{' for class variable values.");
      }
    } else if (parts.size() == 3) {
      if (classPtr->varMap.find(parts[1]) == classPtr->varMap.end()) {
        string ret = "Debug response: Variable ";
        ret += parts[2];
        ret += " does not exist in class: ";
        ret += className;
        return ret;
      }
      if (!classPtr->updateDebugVar(parts[1], parts[2])) {
        string("Debug response: Invalid value specified for variable: ") + parts[1];
      }
      return string("Success");
    }
  }

  /**
   * Returns the memory variable by matching its name.
   *
   * @param string name: Name of the variable to be returned.
   * @return Pointer to ThreadSafeVariable.
   */
  ThreadSafeVariable*
  findVariableFromName(const string& name)
  {
    for (int i = 0; i < variables.size(); i++) {
      if (variables[i]->getVariableName() == name) return variables[i];
    }
    return NULL;
  }
protected:
  //! Vector of pointers to threadsafe memory variables.
  vector<ThreadSafeVariable*> variables;

  //! String to enum map for variable names.
  map<string, unsigned> varMap;
private:
  //! Updates the class map with class name and its pointer.
  void
  updateClassMap(const string& name, DebugBase* ptr);

  //! Name to pointer map for all the childs of this class.
  static map<string, DebugBase*> classMap;

  //! Image queue thread-safe access mutex.
  static pthread_mutex_t classMapAccessMutex;
};
