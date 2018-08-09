#include <string>

using namespace std;

namespace GnuPlotEnv
{

  enum class GnuPlotStyle
:  unsigned int {
    lines = 0,
    points,
    linespoints,
    impulses,
    dots,
    steps,
    fsteps,
    histeps,
    boxes,
    filledcurves,
    histograms,
    labels,
    xerrorbars,
    xerrorlines,
    errorbars,
    errorlines,
    yerrorbars,
    yerrorlines,
    boxerrorbars,
    xyerrorbars,
    xyerrorlines,
    boxxyerrorbars,
    financebars,
    candlesticks,
    vectors,
    image,
    rgbimage,
    pm3d
  };

  enum class GnuPlotColor
:  unsigned int {
    white, //!#ffffff = 255 255 255
    black,//!#000000 =   0   0   0
    darkGrey,//!#a0a0a0 = 160 160 160
    red,//!#ff0000 = 255   0   0
    webGreen,//!#00c000 =   0 192   0
    webBlue,//!#0080ff =   0 128 255
    darkMagenta,//!#c000ff = 192   0 255
    darkCyan,//!#00eeee =   0 238 238
    darkOrange,//!#c04000 = 192  64   0
    darkYellow,//!#c8c800 = 200 200   0
    royalblue,//!#4169e1 =  65 105 225
    goldenrod,//!#ffc020 = 255 192  32
    darkSpringGreen,//!#008040 =   0 128  64
    purple,//!#c080ff = 192 128 255
    steelblue,//!#306080 =  48  96 128
    darkRed,//!#8b0000 = 139   0   0
    darkChartreuse,//!#408000 =  64 128   0
    orchid,//!#ff80ff = 255 128 255
    aquamarine,//!#7fffd4 = 127 255 212
    brown,//!#a52a2a = 165  42  42
    yellow,//!#ffff00 = 255 255   0
    turquoise,//!#40e0d0 =  64 224 208
    grey0,//!#000000 =   0   0   0
    grey10,//!#1a1a1a =  26  26  26
    grey20,//!#333333 =  51  51  51
    grey30,//!#4d4d4d =  77  77  77
    grey40,//!#666666 = 102 102 102
    grey50,//!#7f7f7f = 127 127 127
    grey60,//!#999999 = 153 153 153
    grey70,//!#b3b3b3 = 179 179 179
    grey,//!#c0c0c0 = 192 192 192
    grey80,//!#cccccc = 204 204 204
    grey90,//!#e5e5e5 = 229 229 229
    grey100,//!#ffffff = 255 255 255
    lightRed,//!#f03232 = 240  50  50
    lightGreen,//!#90ee90 = 144 238 144
    lightBlue,//!#add8e6 = 173 216 230
    lightMagenta,//!#f055f0 = 240  85 240
    lightCyan,//!#e0ffff = 224 255 255
    lightGoldenrod,//!#eedd82 = 238 221 130
    lightPink,//!#ffb6c1 = 255 182 193
    lightTurquoise,//!#afeeee = 175 238 238
    gold,//!#ffd700 = 255 215   0
    green,//!#00ff00 =   0 255   0
    darkGreen,//!#006400 =   0 100   0
    springGreen,//!#00ff7f =   0 255 127
    forestGreen,//!#228b22 =  34 139  34
    seaGreen,//!#2e8b57 =  46 139  87
    blue,//!#0000ff =   0   0 255
    darkBlue,//!#00008b =   0   0 139
    midnightBlue,//!#191970 =  25  25 112
    navy,//!#000080 =   0   0 128
    mediumBlue,//!#0000cd =   0   0 205
    skyblue,//!#87ceeb = 135 206 235
    cyan,//!#00ffff =   0 255 255
    magenta,//!#ff00ff = 255   0 255
    darkTurquoise,//!#00ced1 =   0 206 209
    darkPink,//!#ff1493 = 255  20 147
    coral,//!#ff7f50 = 255 127  80
    lightCoral,//!#f08080 = 240 128 128
    orangeRed,//!#ff4500 = 255  69   0
    salmon,//!#fa8072 = 250 128 114
    darkSalmon,//!#e9967a = 233 150 122
    khaki,//!#f0e68c = 240 230 140
    darkKhaki,//!#bdb76b = 189 183 107
    darkGoldenrod,//!#b8860b = 184 134  11
    beige,//!#f5f5dc = 245 245 220
    olive,//!#a08020 = 160 128  32
    orange,//!#ffa500 = 255 165   0
    violet,//!#ee82ee = 238 130 238
    darkViolet,//!#9400d3 = 148   0 211
    plum,//!#dda0dd = 221 160 221
    darkPlum,//!#905040 = 144  80  64
    darkOlivegreen,//!#556b2f =  85 107  47
    orangered4,//!#801400 = 128  20   0
    brown4,//!#801414 = 128  20  20
    sienna4,//!#804014 = 128  64  20
    orchid4,//!#804080 = 128  64 128
    mediumpurple3,//!#8060c0 = 128  96 192
    slateblue1,//!#8060ff = 128  96 255
    yellow4,//!#808000 = 128 128   0
    sienna1,//!#ff8040 = 255 128  64
    tan1,//!#ffa040 = 255 160  64
    sandybrown,//!#ffa060 = 255 160  96
    lightSalmon,//!#ffa070 = 255 160 112
    pink,//!#ffc0c0 = 255 192 192
    khaki1,//!#ffff80 = 255 255 128
    lemonchiffon,//!#ffffc0 = 255 255 192
    bisque,//!#cdb79e = 205 183 158
    honeydew,//!#f0fff0 = 240 255 240
    slategrey,//!#a0b6cd = 160 182 205
    seagreen,//!#c1ffc1 = 193 255 193
    antiquewhite,//!#cdc0b0 = 205 192 176
    chartreuse,//!#7cff40 = 124 255  64
    greenyellow,//!#a0ff20 = 160 255  32
    gray,//!#bebebe = 190 190 190
    lightGray,//!#d3d3d3 = 211 211 211
    lightGrey,//!#d3d3d3 = 211 211 211
    darkGray,//!#a0a0a0 = 160 160 160
    slategray,//!#a0b6cd = 160 182 205
    gray0,//!#000000 =   0   0   0
    gray10,//!#1a1a1a =  26  26  26
    gray20,//!#333333 =  51  51  51
    gray30,//!#4d4d4d =  77  77  77
    gray40,//!#666666 = 102 102 102
    gray50,//!#7f7f7f = 127 127 127
    gray60,//!#999999 = 153 153 153
    gray70,//!#b3b3b3 = 179 179 179
    gray80,//!#cccccc = 204 204 204
    gray90,//!#e5e5e5 = 229 229 229
    gray100//!#ffffff = 255 255 255
  };

  enum class GnuArrowType
:  unsigned int {
    noHead,
    head,
    backHead,
    heads
  };

  enum class GnuArrowFill
:  unsigned int {
    filled,
    empty,
    nofilled
  };

  enum class GnuCoords
:  unsigned int {
    first,
    second,
    graph,
    screen,
    character
  };

  const string gnuPlotStyles[28]
    { "lines", "points", "linespoints", "impulses", "dots", "steps", "fsteps",
      "histeps", "boxes", "filledcurves", "histograms", "labels", "xerrorbars",
      "xerrorlines", "errorbars", "errorlines", "yerrorbars", "yerrorlines",
      "boxerrorbars", "xyerrorbars", "xyerrorlines", "boxxyerrorbars",
      "financebars", "candlesticks", "vectors", "image", "rgbimage", "pm3d" };

  const string gnuArrowType[4]
    { "noHead", "head", "backHead", "heads" };

  const string gnuArrowFill[3]
    { "filled", "empty", "nofilled" };

  const string gnuCoords[5]
    { "first", "second", "graph", "screen", "character" };

  const string gnuColorNames[112]
    { "white", "black", "dark-grey", "red", "web-green", "web-blue ",
      "dark-magenta", "dark-cyan", "dark-orange ", "dark-yellow", "royalblue",
      "goldenrod", "dark-spring-green", "purple", "steelblue", "dark-red",
      "dark-chartreuse", "orchid", "aquamarine", "brown", "yellow", "turquoise",
      "grey0 ", "grey10", "grey20", "grey30", "grey40", "grey50", "grey60",
      "grey70", "grey", "grey80  ", "grey90 ", "grey100", "light-red ",
      "light-green ", "light-blue", "light-magenta", "light-cyan",
      "light-goldenrod", "light-pink", "light-turquoise", "gold", "green",
      "dark-green ", "spring-green", "forest-green", "sea-green ", "blue",
      "dark-blue", "midnight-blue", "navy", "medium-blue", "skyblue", "cyan",
      "magenta", "dark-turquoise", "dark-pink", "coral", "light-coral",
      "orange-red", "salmon", "dark-salmon", "khaki", "dark-khaki",
      "dark-goldenrod", "beige", "olive", "orange", "violet", "dark-violet",
      "plum", "dark-plum", "dark-olivegreen", "orangered4", "brown4", "sienna4",
      "orchid4", "mediumpurple3", "slateblue1", "yellow4", "sienna1", "tan1",
      "sandybrown", "light-salmon", "pink", "khaki1", "lemonchiffon", "bisque",
      "honeydew", "slategrey", "seagreen", "antiquewhite", "chartreuse",
      "greenyellow", "gray", "light-gray", "light-grey", "dark-gray",
      "slategray", "gray0", "gray10", "gray20", "gray30", "gray40", "gray50",
      "gray60", "gray70", "gray80", "gray90", "gray100" };

}
