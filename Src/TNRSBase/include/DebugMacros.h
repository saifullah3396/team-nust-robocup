/**
 * @file TNRSBase/include/DebugMacros.h
 *
 * This file defines the macros for debugging symbols.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 09 Nov 2017  
 */

#include "Utils/include/VariadicMacros.h"

/**
 * \def ADD_SYMBOL(...)
 * 
 * Adds the symbol of given name into debugMap.
 * 
 * @param name: Name of the debug symbol
 */
#define ADD_SYMBOL_(name) \
	debugMap.insert(pair<string, int>(#name, 0));

/**
 * \def DEBUG_SYMBOLS(...)
 * 
 * Wrapper for DEBUG_SYMBOLS_()
 * 
 * @param ... : name, name, ... name, .
 */
#define DEBUG_SYMBOLS(...) DEBUG_SYMBOLS_(__VA_ARGS__)

/**
 * \def DEBUG_SYMBOLS_(...)
 * 
 * This macro defines the debug symbols based on input array of strings.
 * 
 * @param ... : name, name, ... name, .
 */
#define DEBUG_SYMBOLS_(...) { \
public: \
	FOR_EACH(ADD_SYMBOL_, __VA_ARGS__) \
}
