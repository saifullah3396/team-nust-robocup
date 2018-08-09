#include <iostream>
#include "../include/EnumToString.h"

DEFINE_ENUM_WITH_STRING_CONVERSIONS(Days, (Mon)(Tue)(Wed)(Thur)(Fri)(Sat)(Sun))
DEFINE_ENUM_WITH_STRING_CONVERSIONS(Months, (Jan)(Feb))

int main()
{
    Days d = Mon;
    std::cout << EnumToString(d) << " " << EnumToString(Tue) << std::endl;
}
