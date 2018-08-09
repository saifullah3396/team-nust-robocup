/**
 * @file Utils/include/AssertUtil.h
 *
 * This file defines the macro for handling assertions.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 07 Feb 2017
 */

#ifndef _DEBUG_UTILS_H_
#define _DEBUG_UTILS_H_

#include <assert.h>
#include <chrono>
#include <iostream>
#include <string>

using namespace std;

extern int USE_LOGGED_IMAGES;
extern int SAVE_IMAGES;
extern int PROJECT_GIELD;
extern int SIMULATION;
extern string ROBOT_NAME;

using namespace std;
using namespace std::chrono;

#define PRINT_1(x) cout << "[INFO]: " << x << endl;
#define PRINT(x) PRINT_1(x)

#define ERROR_1(x) cerr << "[ERROR]: " << x << endl;
#define ERROR(x) ERROR_1(x)

#define ASSERT(condition) \
{ \
  assert(condition); \
}

#define ASSERT_MSG(condition, msg) \
{ \
  assert(condition && #msg); \
}
#endif //!_DEBUG_UTILS_H_
