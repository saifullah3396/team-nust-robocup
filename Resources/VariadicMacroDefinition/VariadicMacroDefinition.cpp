/**
 * @file Utils/VariadicMacroGeneration/VariadicMacroDefinition.cpp
 *
 * This file writes the the definition of variadic macro in VariadicMacros.h
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Feb 2017  
 */

#include <fstream>
#include <iostream>
#include <map>
#include <string.h>
#include <sstream>
#include <typeinfo>

#define MAX_NUM_VARS 61

using namespace std;

int main()
{
	std::stringstream line;
	fstream file;
	file.open("Utils/VariadicMacros.h");
	file << "M_NARGS_(";
	int k = MAX_NUM_VARS-1;
	for (int i=0;i<MAX_NUM_VARS;++i)
		if(k==MAX_NUM_VARS-1)
			file << "_" << k-- << ",";
		else
			file << " _" << k-- << ",";
	file << " N, ...) N" << endl;
		
	for (int i=0;i<MAX_NUM_VARS;++i){
		line.str("");
		line << "#define M_GET_ELEM_";
		line << i;
		line << "(";
		for (int j=0;j<i+1;++j) {
			if(!j)
				line << "_";
			else
				line << " _";
			line << j;
			line << ",";
		}
		line << " ...) _";
		line << i;
		file << line.str() << endl;
	}
	return 0;
}
