#include <iostream>
#include "../include/Exceptions/ArgParseException.h"
#include "../include/EnumToString.h"

int main()
{
    try {
		throw ArgParseException("Invalid value for argument --robotName.", false, EXC_INVALID_ARG_VALUE);
	} catch (ArgParseException& e) {
		cout << e.what();
	}
}
