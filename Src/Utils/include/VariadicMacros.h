/**
 * @file Utils/include/VariadicMacros.h
 *
 * This file defines the macros for handling unknown number of macro
 * arguments.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 07 Feb 2017
 */

#pragma once

#include <stdio.h>

#define NOTHING_EXTRACT
#define EXTRACT(...) EXTRACT __VA_ARGS__
#define PASTE(x, ...) x##__VA_ARGS__
#define EVALUATING_PASTE(x, ...) PASTE(x, __VA_ARGS__)
#define UNPAREN(x) EVALUATING_PASTE(NOTHING_, EXTRACT x)

#define GET_STRINGS(x) x
#define TO_STRING(arg)  TO_STRING_(arg)
#define TO_STRING_(arg) #arg

#define M_NARGS(...) M_NARGS_(__VA_ARGS__, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define M_NARGS_(_60, _59, _58, _57, _56, _55, _54, _53, _52, _51, _50, _49, _48, _47, _46, _45, _44, _43, _42, _41, _40, _39, _38, _37, _36, _35, _34, _33, _32, _31, _30, _29, _28, _27, _26, _25, _24, _23, _22, _21, _20, _19, _18, _17, _16, _15, _14, _13, _12, _11, _10, _9, _8, _7, _6, _5, _4, _3, _2, _1, _0, N, ...) N
#define M_CONC(A, B) M_CONC_(A, B)
#define M_CONC_(A, B) A##B

#define M_GET_ELEM(N, ...) M_CONC(M_GET_ELEM_, N)(__VA_ARGS__)
#define M_GET_ELEM_0(_0, ...) _0
#define M_GET_ELEM_1(_0, _1, ...) _1
#define M_GET_ELEM_2(_0, _1, _2, ...) _2
#define M_GET_ELEM_3(_0, _1, _2, _3, ...) _3
#define M_GET_ELEM_4(_0, _1, _2, _3, _4, ...) _4
#define M_GET_ELEM_5(_0, _1, _2, _3, _4, _5, ...) _5
#define M_GET_ELEM_6(_0, _1, _2, _3, _4, _5, _6, ...) _6
#define M_GET_ELEM_7(_0, _1, _2, _3, _4, _5, _6, _7, ...) _7
#define M_GET_ELEM_8(_0, _1, _2, _3, _4, _5, _6, _7, _8, ...) _8
#define M_GET_ELEM_9(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, ...) _9
#define M_GET_ELEM_10(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, ...) _10
#define M_GET_ELEM_11(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, ...) _11
#define M_GET_ELEM_12(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, ...) _12
#define M_GET_ELEM_13(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, ...) _13
#define M_GET_ELEM_14(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, ...) _14
#define M_GET_ELEM_15(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, ...) _15
#define M_GET_ELEM_16(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, ...) _16
#define M_GET_ELEM_17(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, ...) _17
#define M_GET_ELEM_18(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, ...) _18
#define M_GET_ELEM_19(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, ...) _19
#define M_GET_ELEM_20(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, ...) _20
#define M_GET_ELEM_21(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, ...) _21
#define M_GET_ELEM_22(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, ...) _22
#define M_GET_ELEM_23(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, ...) _23
#define M_GET_ELEM_24(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, ...) _24
#define M_GET_ELEM_25(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, ...) _25
#define M_GET_ELEM_26(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, ...) _26
#define M_GET_ELEM_27(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, ...) _27
#define M_GET_ELEM_28(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, ...) _28
#define M_GET_ELEM_29(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, ...) _29
#define M_GET_ELEM_30(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, ...) _30
#define M_GET_ELEM_31(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, ...) _31
#define M_GET_ELEM_32(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, ...) _32
#define M_GET_ELEM_33(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, ...) _33
#define M_GET_ELEM_34(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, ...) _34
#define M_GET_ELEM_35(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, ...) _35
#define M_GET_ELEM_36(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, ...) _36
#define M_GET_ELEM_37(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, ...) _37
#define M_GET_ELEM_38(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, ...) _38
#define M_GET_ELEM_39(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, ...) _39
#define M_GET_ELEM_40(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, ...) _40
#define M_GET_ELEM_41(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, ...) _41
#define M_GET_ELEM_42(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, ...) _42
#define M_GET_ELEM_43(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, ...) _43
#define M_GET_ELEM_44(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, ...) _44
#define M_GET_ELEM_45(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, ...) _45
#define M_GET_ELEM_46(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, ...) _46
#define M_GET_ELEM_47(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, ...) _47
#define M_GET_ELEM_48(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, ...) _48
#define M_GET_ELEM_49(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, ...) _49
#define M_GET_ELEM_50(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, ...) _50
#define M_GET_ELEM_51(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, ...) _51
#define M_GET_ELEM_52(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, _52, ...) _52
#define M_GET_ELEM_53(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, _52, _53, ...) _53
#define M_GET_ELEM_54(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, _52, _53, _54, ...) _54
#define M_GET_ELEM_55(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, _52, _53, _54, _55, ...) _55
#define M_GET_ELEM_56(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, _52, _53, _54, _55, _56, ...) _56
#define M_GET_ELEM_57(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, _52, _53, _54, _55, _56, _57, ...) _57
#define M_GET_ELEM_58(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, _52, _53, _54, _55, _56, _57, _58, ...) _58
#define M_GET_ELEM_59(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, _52, _53, _54, _55, _56, _57, _58, _59, ...) _59
#define M_GET_ELEM_60(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, _52, _53, _54, _55, _56, _57, _58, _59, _60, ...) _60
#define M_GET_ELEM_AFTER(N, ...) M_CONC(M_GET_ELEM_AFTER_, N)(__VA_ARGS__)
#define M_GET_ELEM_AFTER_0(_0, ...) __VA_ARGS__

#define FOR_EACH_1(what, x, ...) what(x)
#define FOR_EACH_2(what, x, ...) what(x); FOR_EACH_1(what, __VA_ARGS__)
#define FOR_EACH_3(what, x, ...) what(x); FOR_EACH_2(what, __VA_ARGS__)
#define FOR_EACH_4(what, x, ...) what(x); FOR_EACH_3(what, __VA_ARGS__)
#define FOR_EACH_5(what, x, ...) what(x); FOR_EACH_4(what, __VA_ARGS__)
#define FOR_EACH_6(what, x, ...) what(x); FOR_EACH_5(what, __VA_ARGS__)
#define FOR_EACH_7(what, x, ...) what(x); FOR_EACH_6(what, __VA_ARGS__)
#define FOR_EACH_8(what, x, ...) what(x); FOR_EACH_7(what, __VA_ARGS__)
#define FOR_EACH_9(what, x, ...) what(x); FOR_EACH_8(what, __VA_ARGS__)
#define FOR_EACH_10(what, x, ...) what(x); FOR_EACH_9(what, __VA_ARGS__)
#define FOR_EACH_11(what, x, ...) what(x); FOR_EACH_10(what, __VA_ARGS__)
#define FOR_EACH_12(what, x, ...) what(x); FOR_EACH_11(what, __VA_ARGS__)
#define FOR_EACH_13(what, x, ...) what(x); FOR_EACH_12(what, __VA_ARGS__)
#define FOR_EACH_14(what, x, ...) what(x); FOR_EACH_13(what, __VA_ARGS__)
#define FOR_EACH_15(what, x, ...) what(x); FOR_EACH_14(what, __VA_ARGS__)
#define FOR_EACH_16(what, x, ...) what(x); FOR_EACH_15(what, __VA_ARGS__)
#define FOR_EACH_17(what, x, ...) what(x); FOR_EACH_16(what, __VA_ARGS__)
#define FOR_EACH_18(what, x, ...) what(x); FOR_EACH_17(what, __VA_ARGS__)
#define FOR_EACH_19(what, x, ...) what(x); FOR_EACH_18(what, __VA_ARGS__)
#define FOR_EACH_20(what, x, ...) what(x); FOR_EACH_19(what, __VA_ARGS__)
#define FOR_EACH_21(what, x, ...) what(x); FOR_EACH_20(what, __VA_ARGS__)
#define FOR_EACH_22(what, x, ...) what(x); FOR_EACH_21(what, __VA_ARGS__)
#define FOR_EACH_23(what, x, ...) what(x); FOR_EACH_22(what, __VA_ARGS__)
#define FOR_EACH_24(what, x, ...) what(x); FOR_EACH_23(what, __VA_ARGS__)
#define FOR_EACH_25(what, x, ...) what(x); FOR_EACH_24(what, __VA_ARGS__)
#define FOR_EACH_26(what, x, ...) what(x); FOR_EACH_25(what, __VA_ARGS__)
#define FOR_EACH_27(what, x, ...) what(x); FOR_EACH_26(what, __VA_ARGS__)
#define FOR_EACH_28(what, x, ...) what(x); FOR_EACH_27(what, __VA_ARGS__)
#define FOR_EACH_29(what, x, ...) what(x); FOR_EACH_28(what, __VA_ARGS__)
#define FOR_EACH_30(what, x, ...) what(x); FOR_EACH_29(what, __VA_ARGS__)
#define FOR_EACH_31(what, x, ...) what(x); FOR_EACH_30(what, __VA_ARGS__)
#define FOR_EACH_32(what, x, ...) what(x); FOR_EACH_31(what, __VA_ARGS__)
#define FOR_EACH_33(what, x, ...) what(x); FOR_EACH_32(what, __VA_ARGS__)
#define FOR_EACH_34(what, x, ...) what(x); FOR_EACH_33(what, __VA_ARGS__)
#define FOR_EACH_35(what, x, ...) what(x); FOR_EACH_34(what, __VA_ARGS__)
#define FOR_EACH_36(what, x, ...) what(x); FOR_EACH_35(what, __VA_ARGS__)
#define FOR_EACH_37(what, x, ...) what(x); FOR_EACH_36(what, __VA_ARGS__)
#define FOR_EACH_38(what, x, ...) what(x); FOR_EACH_37(what, __VA_ARGS__)
#define FOR_EACH_39(what, x, ...) what(x); FOR_EACH_38(what, __VA_ARGS__)
#define FOR_EACH_40(what, x, ...) what(x); FOR_EACH_39(what, __VA_ARGS__)
#define FOR_EACH_41(what, x, ...) what(x); FOR_EACH_40(what, __VA_ARGS__)
#define FOR_EACH_42(what, x, ...) what(x); FOR_EACH_41(what, __VA_ARGS__)
#define FOR_EACH_43(what, x, ...) what(x); FOR_EACH_42(what, __VA_ARGS__)
#define FOR_EACH_44(what, x, ...) what(x); FOR_EACH_43(what, __VA_ARGS__)
#define FOR_EACH_45(what, x, ...) what(x); FOR_EACH_44(what, __VA_ARGS__)
#define FOR_EACH_46(what, x, ...) what(x); FOR_EACH_45(what, __VA_ARGS__)
#define FOR_EACH_47(what, x, ...) what(x); FOR_EACH_46(what, __VA_ARGS__)
#define FOR_EACH_48(what, x, ...) what(x); FOR_EACH_47(what, __VA_ARGS__)
#define FOR_EACH_49(what, x, ...) what(x); FOR_EACH_48(what, __VA_ARGS__)
#define FOR_EACH_50(what, x, ...) what(x); FOR_EACH_49(what, __VA_ARGS__)
#define FOR_EACH_51(what, x, ...) what(x); FOR_EACH_50(what, __VA_ARGS__)
#define FOR_EACH_52(what, x, ...) what(x); FOR_EACH_51(what, __VA_ARGS__)
#define FOR_EACH_53(what, x, ...) what(x); FOR_EACH_52(what, __VA_ARGS__)
#define FOR_EACH_54(what, x, ...) what(x); FOR_EACH_53(what, __VA_ARGS__)
#define FOR_EACH_55(what, x, ...) what(x); FOR_EACH_54(what, __VA_ARGS__)
#define FOR_EACH_56(what, x, ...) what(x); FOR_EACH_55(what, __VA_ARGS__)
#define FOR_EACH_57(what, x, ...) what(x); FOR_EACH_56(what, __VA_ARGS__)
#define FOR_EACH_58(what, x, ...) what(x); FOR_EACH_57(what, __VA_ARGS__)
#define FOR_EACH_59(what, x, ...) what(x); FOR_EACH_58(what, __VA_ARGS__)
#define FOR_EACH_60(what, x, ...) what(x); FOR_EACH_59(what, __VA_ARGS__)

#define FOR_EACH_IN_PLACE_1(what, x, ...) what(x)
#define FOR_EACH_IN_PLACE_2(what, x, ...) what(x), FOR_EACH_IN_PLACE_1(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_3(what, x, ...) what(x), FOR_EACH_IN_PLACE_2(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_4(what, x, ...) what(x), FOR_EACH_IN_PLACE_3(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_5(what, x, ...) what(x), FOR_EACH_IN_PLACE_4(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_6(what, x, ...) what(x), FOR_EACH_IN_PLACE_5(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_7(what, x, ...) what(x), FOR_EACH_IN_PLACE_6(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_8(what, x, ...) what(x), FOR_EACH_IN_PLACE_7(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_9(what, x, ...) what(x), FOR_EACH_IN_PLACE_8(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_10(what, x, ...) what(x), FOR_EACH_IN_PLACE_9(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_11(what, x, ...) what(x), FOR_EACH_IN_PLACE_10(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_12(what, x, ...) what(x), FOR_EACH_IN_PLACE_11(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_13(what, x, ...) what(x), FOR_EACH_IN_PLACE_12(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_14(what, x, ...) what(x), FOR_EACH_IN_PLACE_13(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_15(what, x, ...) what(x), FOR_EACH_IN_PLACE_14(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_16(what, x, ...) what(x), FOR_EACH_IN_PLACE_15(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_17(what, x, ...) what(x), FOR_EACH_IN_PLACE_16(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_18(what, x, ...) what(x), FOR_EACH_IN_PLACE_17(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_19(what, x, ...) what(x), FOR_EACH_IN_PLACE_18(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_20(what, x, ...) what(x), FOR_EACH_IN_PLACE_19(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_21(what, x, ...) what(x), FOR_EACH_IN_PLACE_20(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_22(what, x, ...) what(x), FOR_EACH_IN_PLACE_21(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_23(what, x, ...) what(x), FOR_EACH_IN_PLACE_22(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_24(what, x, ...) what(x), FOR_EACH_IN_PLACE_23(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_25(what, x, ...) what(x), FOR_EACH_IN_PLACE_24(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_26(what, x, ...) what(x), FOR_EACH_IN_PLACE_25(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_27(what, x, ...) what(x), FOR_EACH_IN_PLACE_26(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_28(what, x, ...) what(x), FOR_EACH_IN_PLACE_27(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_29(what, x, ...) what(x), FOR_EACH_IN_PLACE_28(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_30(what, x, ...) what(x), FOR_EACH_IN_PLACE_29(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_31(what, x, ...) what(x), FOR_EACH_IN_PLACE_30(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_32(what, x, ...) what(x), FOR_EACH_IN_PLACE_31(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_33(what, x, ...) what(x), FOR_EACH_IN_PLACE_32(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_34(what, x, ...) what(x), FOR_EACH_IN_PLACE_33(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_35(what, x, ...) what(x), FOR_EACH_IN_PLACE_34(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_36(what, x, ...) what(x), FOR_EACH_IN_PLACE_35(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_37(what, x, ...) what(x), FOR_EACH_IN_PLACE_36(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_38(what, x, ...) what(x), FOR_EACH_IN_PLACE_37(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_39(what, x, ...) what(x), FOR_EACH_IN_PLACE_38(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_40(what, x, ...) what(x), FOR_EACH_IN_PLACE_39(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_41(what, x, ...) what(x), FOR_EACH_IN_PLACE_40(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_42(what, x, ...) what(x), FOR_EACH_IN_PLACE_41(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_43(what, x, ...) what(x), FOR_EACH_IN_PLACE_42(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_44(what, x, ...) what(x), FOR_EACH_IN_PLACE_43(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_45(what, x, ...) what(x), FOR_EACH_IN_PLACE_44(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_46(what, x, ...) what(x), FOR_EACH_IN_PLACE_45(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_47(what, x, ...) what(x), FOR_EACH_IN_PLACE_46(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_48(what, x, ...) what(x), FOR_EACH_IN_PLACE_47(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_49(what, x, ...) what(x), FOR_EACH_IN_PLACE_48(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_50(what, x, ...) what(x), FOR_EACH_IN_PLACE_49(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_51(what, x, ...) what(x), FOR_EACH_IN_PLACE_50(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_52(what, x, ...) what(x), FOR_EACH_IN_PLACE_51(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_53(what, x, ...) what(x), FOR_EACH_IN_PLACE_52(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_54(what, x, ...) what(x), FOR_EACH_IN_PLACE_53(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_55(what, x, ...) what(x), FOR_EACH_IN_PLACE_54(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_56(what, x, ...) what(x), FOR_EACH_IN_PLACE_55(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_57(what, x, ...) what(x), FOR_EACH_IN_PLACE_56(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_58(what, x, ...) what(x), FOR_EACH_IN_PLACE_57(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_59(what, x, ...) what(x), FOR_EACH_IN_PLACE_58(what, __VA_ARGS__)
#define FOR_EACH_IN_PLACE_60(what, x, ...) what(x), FOR_EACH_IN_PLACE_59(what, __VA_ARGS__)

#define FOR_EACH_(N, what, x, ...) M_CONC(FOR_EACH_, N)(what, x, __VA_ARGS__)
#define FOR_EACH(what, x, ...) FOR_EACH_(M_NARGS(x, __VA_ARGS__), what, x, __VA_ARGS__)

#define FOR_EACH_IN_PLACE_(N, what, x, ...) M_CONC(FOR_EACH_IN_PLACE_, N)(what, x, __VA_ARGS__)
#define FOR_EACH_IN_PLACE(what, x, ...) FOR_EACH_IN_PLACE_(M_NARGS(x, __VA_ARGS__), what, x, __VA_ARGS__)

#define PRINT_STRING(s) printf("%s", s)
#define MAKESTR_PRINT(x) printf("%s", #x)

#define MAKE_STRING(str, ...) #str, MAKE_STRING1_(__VA_ARGS__)
#define MAKE_STRING1_(str, ...) #str, MAKE_STRING2_(__VA_ARGS__)
#define MAKE_STRING2_(str, ...) #str, MAKE_STRING3_(__VA_ARGS__)
#define MAKE_STRING3_(str, ...) #str, MAKE_STRING4_(__VA_ARGS__)
#define MAKE_STRING4_(str, ...) #str, MAKE_STRING5_(__VA_ARGS__)
#define MAKE_STRING5_(str, ...) #str, MAKE_STRING6_(__VA_ARGS__)
#define MAKE_STRING6_(str, ...) #str, MAKE_STRING7_(__VA_ARGS__)
#define MAKE_STRING7_(str, ...) #str, MAKE_STRING8_(__VA_ARGS__)
#define MAKE_STRING8_(str, ...) #str, MAKE_STRING9_(__VA_ARGS__)
#define MAKE_STRING9_(str, ...) #str, MAKE_STRING10_(__VA_ARGS__)
#define MAKE_STRING10_(str, ...) #str, MAKE_STRING11_(__VA_ARGS__)
#define MAKE_STRING11_(str, ...) #str, MAKE_STRING12_(__VA_ARGS__)
#define MAKE_STRING12_(str, ...) #str, MAKE_STRING13_(__VA_ARGS__)
#define MAKE_STRING13_(str, ...) #str, MAKE_STRING14_(__VA_ARGS__)
#define MAKE_STRING14_(str, ...) #str, MAKE_STRING15_(__VA_ARGS__)
#define MAKE_STRING15_(str, ...) #str, MAKE_STRING16_(__VA_ARGS__)
#define MAKE_STRING16_(str, ...) #str, MAKE_STRING17_(__VA_ARGS__)
#define MAKE_STRING17_(str, ...) #str, MAKE_STRING18_(__VA_ARGS__)
#define MAKE_STRING18_(str, ...) #str, MAKE_STRING19_(__VA_ARGS__)
#define MAKE_STRING19_(str, ...) #str, MAKE_STRING20_(__VA_ARGS__)
#define MAKE_STRING20_(str, ...) #str, MAKE_STRING21_(__VA_ARGS__)
#define MAKE_STRING21_(str, ...) #str, MAKE_STRING22_(__VA_ARGS__)
#define MAKE_STRING22_(str, ...) #str, MAKE_STRING23_(__VA_ARGS__)
#define MAKE_STRING23_(str, ...) #str, MAKE_STRING24_(__VA_ARGS__)
#define MAKE_STRING24_(str, ...) #str, MAKE_STRING25_(__VA_ARGS__)
#define MAKE_STRING25_(str, ...) #str, MAKE_STRING26_(__VA_ARGS__)
#define MAKE_STRING26_(str, ...) #str, MAKE_STRING27_(__VA_ARGS__)
#define MAKE_STRING27_(str, ...) #str, MAKE_STRING28_(__VA_ARGS__)
#define MAKE_STRING28_(str, ...) #str, MAKE_STRING29_(__VA_ARGS__)
#define MAKE_STRING29_(str, ...) #str, MAKE_STRING30_(__VA_ARGS__)
#define MAKE_STRING30_(str, ...) #str, MAKE_STRING31_(__VA_ARGS__)
#define MAKE_STRING31_(str, ...) #str, MAKE_STRING32_(__VA_ARGS__)
#define MAKE_STRING32_(str, ...) #str, MAKE_STRING33_(__VA_ARGS__)
#define MAKE_STRING33_(str, ...) #str, MAKE_STRING34_(__VA_ARGS__)
#define MAKE_STRING34_(str, ...) #str, MAKE_STRING35_(__VA_ARGS__)
#define MAKE_STRING35_(str, ...) #str, MAKE_STRING36_(__VA_ARGS__)
#define MAKE_STRING36_(str, ...) #str, MAKE_STRING37_(__VA_ARGS__)
#define MAKE_STRING37_(str, ...) #str, MAKE_STRING38_(__VA_ARGS__)
#define MAKE_STRING38_(str, ...) #str, MAKE_STRING39_(__VA_ARGS__)
#define MAKE_STRING39_(str, ...) #str, MAKE_STRING40_(__VA_ARGS__)
#define MAKE_STRING40_(str, ...) #str, MAKE_STRING41_(__VA_ARGS__)
#define MAKE_STRING41_(str, ...) #str, MAKE_STRING42_(__VA_ARGS__)
#define MAKE_STRING42_(str, ...) #str, MAKE_STRING43_(__VA_ARGS__)
#define MAKE_STRING43_(str, ...) #str, MAKE_STRING44_(__VA_ARGS__)
#define MAKE_STRING44_(str, ...) #str, MAKE_STRING45_(__VA_ARGS__)
#define MAKE_STRING45_(str, ...) #str, MAKE_STRING46_(__VA_ARGS__)
#define MAKE_STRING46_(str, ...) #str, MAKE_STRING47_(__VA_ARGS__)
#define MAKE_STRING47_(str, ...) #str, MAKE_STRING48_(__VA_ARGS__)
#define MAKE_STRING48_(str, ...) #str, MAKE_STRING49_(__VA_ARGS__)
#define MAKE_STRING49_(str, ...) #str, MAKE_STRING50_(__VA_ARGS__)
#define MAKE_STRING50_(str, ...) #str, MAKE_STRING51_(__VA_ARGS__)
#define MAKE_STRING51_(str, ...) #str, MAKE_STRING52_(__VA_ARGS__)
#define MAKE_STRING52_(str, ...) #str, MAKE_STRING53_(__VA_ARGS__)
#define MAKE_STRING53_(str, ...) #str, MAKE_STRING54_(__VA_ARGS__)
#define MAKE_STRING54_(str, ...) #str, MAKE_STRING55_(__VA_ARGS__)
#define MAKE_STRING55_(str, ...) #str, MAKE_STRING56_(__VA_ARGS__)
#define MAKE_STRING56_(str, ...) #str, MAKE_STRING57_(__VA_ARGS__)
#define MAKE_STRING57_(str, ...) #str, MAKE_STRING58_(__VA_ARGS__)
#define MAKE_STRING58_(str, ...) #str, MAKE_STRING59_(__VA_ARGS__)
#define MAKE_STRING59_(str, ...) #str, MAKE_STRING60_(__VA_ARGS__)
#define MAKE_STRING60_(str) #str

#define GET_NAME(x) M_GET_ELEM(1,UNPAREN(x))
#define SEPARATE_(...) FOR_EACH_IN_PLACE(GET_NAME, __VA_ARGS__)
#define SEPARATE(...) SEPARATE_(__VA_ARGS__)

#define GET_ENUM(offset, ...) M_GET_ELEM(0, __VA_ARGS__) = offset, M_GET_ELEM_AFTER(0, __VA_ARGS__)

#define DEFINE_ENUM(name, offset, ...) \
  enum name { GET_ENUM(offset, __VA_ARGS__) };
//const char *name##Strings[] = { MAKE_STRING(__VA_ARGS__, last) };

