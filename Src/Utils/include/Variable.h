/**
 * @file Utils/include/Variable.h
 *
 * This file declares the class Variable.
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

#include "Utils/include/ThreadSafeVariable.h"
#include "Utils/include/DataUtils.h"

using namespace Utils;

/**
 * Macro that defines a new variable of given type, name and value in
 * the SharedMemory.
 *
 * @param type: Type of the variable.
 * @param name: Name of the variable.
 * @param value: Value of the variable.
 */
#define DEFINE_VARIABLE(type, name, value) \
{ \
  Variable<type> * obj = \
    new Variable<type>(#name, value); \
  variables[name] = obj; \
}

/**
 * Macro that declares a new variable of given type and name in the
 * SharedMemory.
 *
 * @param type: Type of the variable.
 * @param name: Name of the variable.
 */
#define DECLARE_VARIABLE(type, name) \
{ \
  Variable<type> * obj = \
    new Variable<type>(#name); \
  variables[name] = obj; \
}

/**
 * @class Variable
 * @brief A class for defining templated variables.
 *
 * Defines a templated class for templated variables.
 */
template<typename T>
  class Variable : public ThreadSafeVariable
  {
  public:

    /**
     * Defines a threadsafe memory variable with template type T with
     * initial value.
     *
     * @param string variableName: Name of the variable.
     * @param T value: The value of the variable.
     */
    Variable(string variableName, T value) :
      ThreadSafeVariable(variableName)
    {
      this->value = value;
    }

    /**
     * Defines a threadsafe memory variable with template type T.
     *
     * @param string variableName: Name of the variable.
     */
    Variable(string variableName) :
      ThreadSafeVariable(variableName)
    {
      value = *(new T);
    }

    /**
     * Default destructor for this class.
     */
    ~Variable()
    {
    }

    /**
     * Returns the value of the variable in a threadsafe manner.
     *
     * @return T
     */
    T
    getValue();

    /**
     * Tries to return the value of the variable
     * without blocking the calling thread.
     *
     * @return T
     */
    T
    getValueTry();

    /**
     * Sets the value of the variable.
     *
     * @param T value: Value to set to the variable in a threadsafe
     *   manner.
     */
    void
    setValue(const T& value);

    /**
     * Tries to set the value of the variable
     * without blocking the setter thread.
     *
     * @param T value: Value to set to the variable.
     */
    void
    setValueTry(const T& value);

    /**
     * Increment operator for the template type T.
     *
     * @return T
     */
    T
    increment();

    /**
     * Decrement operator for the template type T.
     *
     * @return T
     */
    T
    decrement();

    /**
     * Provides threadsafe access to the variable.
     *
     * @return T&
     */
    T&
    getAccess();

    /**
     * Unlocks the variable mutex.
     */
    void
    unlockMutex();

    /**
     * Sets the value of a variable by dereferncing the input pointer.
     *
     * @param void* ptrvalue: Input pointer variable.
     */
    void
    setValueFromPtr(void* ptrvalue)
    {
      pthread_mutex_lock(&accessMutex);
      this->value = *(static_cast<T*>(ptrvalue));
      pthread_mutex_unlock(&accessMutex);
    }

    /**
     * Gets the value of a variable into the output pointer.
     *
     * @param void* ptrvalue: Output pointer variable.
     */
    void
    getValueIntoPtr(void * ptrvalue)
    {
      pthread_mutex_lock(&accessMutex);
      *static_cast<T*>(ptrvalue) = value;
      pthread_mutex_unlock(&accessMutex);
    }

    /**
     * Gets the variable value as string in the output string.
     *
     * @param string& out: Output string reference.
     */

    void
    getString(string& out)
    {
      DataUtils::getString(out, value);
    }

    /**
     * Returns true if the input string matches with the variable string.
     *
     * @param in: Input string for match.
     * @return bool
     */
    //bool matchString(const string& in) {
//	string varString = "";
    //  DataUtils::getString(varString, value);
    //  if(varString == in)
//		return true;
//	else  
//		return false;
    // }
  protected:
    //! Value of the variable.
    T value;

    //! Check whether the variable is an array.
    bool isArray;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

template<typename T>
  T
  Variable<T>::getValue()
  {
    pthread_mutex_lock(&accessMutex);
    T Tempvalue = value;
    pthread_mutex_unlock(&accessMutex);
    return Tempvalue;
  }

template<typename T>
  T
  Variable<T>::getValueTry()
  {
    pthread_mutex_trylock(&accessMutex);
    T Tempvalue = value;
    pthread_mutex_unlock(&accessMutex);
    return Tempvalue;
  }

template<typename T>
  void
  Variable<T>::setValue(const T& value)
  {
    pthread_mutex_lock(&accessMutex);
    this->value = value;
    pthread_mutex_unlock(&accessMutex);
  }

template<typename T>
  void
  Variable<T>::setValueTry(const T& value)
  {
    pthread_mutex_trylock(&accessMutex);
    this->value = value;
    pthread_mutex_unlock(&accessMutex);
  }

template<typename T>
  T&
  Variable<T>::getAccess()
  {
    pthread_mutex_lock(&accessMutex);
    return value;
  }

template<typename T>
  void
  Variable<T>::unlockMutex()
  {
    pthread_mutex_unlock(&accessMutex);
  }
