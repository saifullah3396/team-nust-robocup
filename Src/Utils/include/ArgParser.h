/**
 * @file Utils/include/ArgParser.h
 *
 * This file declares the class ArgParser.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018
 */

#pragma once

#include "Utils/include/Exceptions/ArgParseException.h"

/**
 * @class ArgParser
 * @brief A class for argument parsing.
 */
class ArgParser {
public:
	/**
	 * Gets the value of the option for given string
	 */
	static char* getCmdOption(
		char ** begin, char ** end, const string & option)
	{
		try {
			char ** itr = find(begin, end, option);
			if (itr != end && ++itr != end)
			{
				return *itr;
			} else {
				throw 
					ArgParseException(
						"No argument specified for option: " + option, 
						false, 
						EXC_NO_ARG_SPECIFIED
					);
			}
		} catch (ArgParseException& e) {
			cout << e.what();
		}
		return 0;
	}

	/**
	 * Checks if the given option exists
	 */
	static bool cmdOptionExists(
		char** begin, char** end, const string& option)
	{
		return find(begin, end, option) != end;
	}
};

