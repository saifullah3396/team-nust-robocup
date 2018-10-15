/**
 * @file Utils/include/PlotEnv.h
 *
 * This file declares the class PlotEnv.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <map>
#include "Eigen/Dense"
#include "Utils/include/GnuPlotDefinitions.h"
#include "Utils/include/GnuPlot.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/DebugUtils.h"

using namespace std;
using namespace Eigen;
using namespace Utils;

namespace GnuPlotEnv
{

  struct LineType
  {
    LineType() :
      width(1.0), pointType(1), pointSize(1.0), pointsInterval(0.0)
    {
    }

    LineType(const double& width, const unsigned& pointType,
      const double& pointSize, const double& pointsInterval) :
      width(width), pointType(pointType), pointSize(pointSize),
        pointsInterval(pointsInterval)
    {
    }

    double width;
    unsigned pointType;
    double pointSize;
    double pointsInterval;
  };

  struct LineStyle
  {
    LineStyle(const unsigned& id = 0, const LineType& type = LineType(),
      const GnuPlotColor& color = GnuPlotColor::black) :
      id(id), color(color), type(type)
    {
    }

    unsigned id;
    GnuPlotColor color;
    LineType type;
  };

  struct ArrowStyle
  {
    ArrowStyle(const unsigned& id = 0, const GnuArrowType& arrowType =
      GnuArrowType::head, const GnuArrowFill& fillType = GnuArrowFill::filled,
      const GnuCoords& sizeCoords = GnuCoords::screen, const double& len = 0.01,
      const double& angle = 15, const double& bAngle = 45, const bool& front =
        true, const LineStyle& lineStyle = LineStyle()) :
      id(id), arrowType(gnuArrowType[(unsigned) arrowType]),
        fillType(gnuArrowFill[(unsigned) fillType]),
        sizeCoords(gnuCoords[(unsigned) sizeCoords]), len(len), angle(angle),
        bAngle(bAngle), front(front), lineStyle(lineStyle)
    {
    }

    unsigned id;
    LineStyle lineStyle;
    string fillType;
    string arrowType;
    string sizeCoords;
    bool front;
    double len;
    double angle;
    double bAngle;
  };

  /**
   * @class PlotEnv
   * @brief Class that initiates a GnuPlot environment and wraps the
   *   GnuPlot plotting utilities through various methods.
   */
  template <typename Scalar>
  class PlotEnv : public Gnuplot
  {
  public:
    /**
     * @brief Constructor that initializes the GnuPlot base class with 
     *   default parameters.
     */
    PlotEnv(const std::string& title, const std::string& labelx,
      const std::string& labely, const std::string& labelz,
      const Matrix<Scalar, 2, 1>& xRange, const Matrix<Scalar, 2, 1>& yRange, const Matrix<Scalar, 2, 1>& zRange) :
      title(title), lsIdCount(0), asIdCount(0)
    {
      setupEnv(labelx, labely, labelz, xRange, yRange, zRange);
    }

    /**
     * @brief Constructor that initializes the GnuPlot base class with 
     *   default parameters
     * 
     * @param style: Type of style to be used during plot
     */
    PlotEnv(const GnuPlotStyle& style) :
      Gnuplot(gnuPlotStyles[(unsigned) style]), title(title)
    {
    }

    /**
     * @brief Plots a 1D graph.
     * 
     * @param x: A vector of values to plot on the x-axis
     * @param title: Title of the plot
     * @param labelx: Label for the x-axis of the plot
     * @param labely: Label for the y-axis of the plot
     * @param style: Type of style to be used during plot
     */
    PlotEnv(const std::vector<double>& x, const std::string& title,
      const std::string& labelx, const std::string& labely,
      const GnuPlotStyle& style) :
      Gnuplot(x, title, labelx, labely, gnuPlotStyles[(unsigned) style]), title(title)
    {
    }

    /**
     * @brief Plots a 2D graph.
     * 
     * @param x: A vector of values to plot on the x-axis
     * @param y: A vector of values to plot on the y-axis
     * @param title: Title of the plot
     * @param labelx: Label for the x-axis of the plot
     * @param labely: Label for the y-axis of the plot
     * @param style: Type of style to be used during plot
     */
    PlotEnv(const std::vector<double>& x, const std::vector<double>& y,
      const std::string& title, const std::string& labelx,
      const std::string& labely, const GnuPlotStyle& style) :
      Gnuplot(x, y, title, labelx, labely, gnuPlotStyles[(unsigned) style]), title(title)
    {
    }

    /**
     * @brief Plots a 3D graph.
     * 
     * @param x: A vector of values to plot on the x-axis
     * @param y: A vector of values to plot on the y-axis
     * @param z: A vector of values to plot on the z-axis
     * @param title: Title of the plot
     * @param labelx: Label for the x-axis of the plot
     * @param labely: Label for the y-axis of the plot
     * @param labelz: Label for the z-axis of the plot
     * @param style: Type of style to be used during plot
     */
    PlotEnv(const std::vector<double>& x, const std::vector<double>& y,
      const std::vector<double>& z, const std::string& title,
      const std::string& labelx, const std::string& labely,
      const std::string& labelz, const GnuPlotStyle& style) :
        Gnuplot(
          x,
          y,
          z,
          title,
          labelx,
          labely,
          labelz,
          gnuPlotStyles[(unsigned) style]), 
        title(title)
    {
    }

    /**
     * @brief Plots a 2D graph.
     * 
     * @param x: A vector of values to plot on the x-axis
     * @param y: A vector of values to plot on the y-axis
     * @param title: Title of the plot
     * @param labelx: Label for the x-axis of the plot
     * @param labely: Label for the y-axis of the plot
     * @param style: Type of style to be used during plot
     */
    void plot2D(
      const vector<Scalar>& x, 
      const vector<Scalar>& y,
      const GnuPlotStyle& style = GnuPlotStyle::lines)
    {
      plot_xy(x, y, title);
    }


    /**
     * @brief Plots a 2D graph.
     * 
     * @param x: A vector of values to plot on the x-axis
     * @param y: A vector of values to plot on the y-axis
     * @param title: Title of the plot
     * @param labelx: Label for the x-axis of the plot
     * @param labely: Label for the y-axis of the plot
     * @param style: Type of style to be used during plot
     */
    void plot2D(
      const Matrix<Scalar, Dynamic, 1>& x, 
      const Matrix<Scalar, Dynamic, 1>& y,
      const GnuPlotStyle& style = GnuPlotStyle::lines)
    {
      vector<Scalar> vecX = vector<Scalar>(x.data(), x.data() + x.rows() * x.cols());
      vector<Scalar> vecY = vector<Scalar>(y.data(), y.data() + y.rows() * y.cols());
      plot_xy(vecX, vecY, title);
    }

    /**
     * @brief Plots a 3D graph.
     * 
     * @param x: A vector of values to plot on the x-axis
     * @param y: A vector of values to plot on the y-axis
     * @param z: A vector of values to plot on the z-axis
     * @param title: Title of the plot
     * @param labelx: Label for the x-axis of the plot
     * @param labely: Label for the y-axis of the plot
     * @param labelz: Label for the z-axis of the plot
     * @param style: Type of style to be used during plot
     */
    void plot3D(const vector<Scalar>& x, const vector<Scalar>& y,
      const vector<Scalar>& z, const GnuPlotStyle& style = GnuPlotStyle::lines)
    {
      plot_xyz(x, y, z, title);
    }
    
    /**
     * @brief Plots a 3D graph.
     * 
     * @param x: A vector of values to plot on the x-axis
     * @param y: A vector of values to plot on the y-axis
     * @param z: A vector of values to plot on the z-axis
     * @param title: Title of the plot
     * @param labelx: Label for the x-axis of the plot
     * @param labely: Label for the y-axis of the plot
     * @param labelz: Label for the z-axis of the plot
     * @param style: Type of style to be used during plot
     */
    void plot3D(const Matrix<Scalar, Dynamic, 1>& x, const Matrix<Scalar, Dynamic, 1>& y,
      const Matrix<Scalar, Dynamic, 1>& z, const GnuPlotStyle& style = GnuPlotStyle::lines)
    {
      vector<Scalar> vecX = vector<Scalar>(x.data(), x.data() + x.rows() * x.cols());
      vector<Scalar> vecY = vector<Scalar>(y.data(), y.data() + y.rows() * y.cols());
      vector<Scalar> vecZ = vector<Scalar>(z.data(), z.data() + z.rows() * z.cols());
      plot_xyz(vecX, vecY, vecZ, title);
    }

    inline void
    setupEnv(const std::string& labelx =
      "x-Axis", const std::string& labely = "y-Axis",
      const std::string& labelz = "z-Axis",
      const Matrix<Scalar, 2, 1>& xRange = Matrix<Scalar, 2, 1>(-1.f, 1.f), const Matrix<Scalar, 2, 1>& yRange =
        Matrix<Scalar, 2, 1>(-1.f, 1.f), const Matrix<Scalar, 2, 1>& zRange = Matrix<Scalar, 2, 1>(-1.f, 1.f))
    {
      Gnuplot::set_terminal_std("qt");
      if (two_dim) cmd("set size ratio -1"); //! Equal x-y ratios for better view
      else cmd("set view equal xyz"); //! Equal x-y-z ratios for better view
      //cmd("set size ratio -1");
      set_xrange(xRange[0], xRange[1]);
      set_yrange(yRange[0], yRange[1]);
      set_zrange(zRange[0], zRange[1]);
      set_title(title);
      set_xlabel(labelx);
      set_ylabel(labely);
      set_zlabel(labelz);
      //! Setup an empty plot
      //cmd("plot 1/0");
      LineStyle ls;
      ls.color = GnuPlotColor::black;
      ls.type.pointSize = 0.1;
      ls.type.pointType = 7;
      setLineStyle("DefLine", ls);

      PRINT("ls.id: " << ls.id);

      ArrowStyle as1;
      as1.lineStyle = ls;

      setArrowStyle("DefArrow", as1);

      ArrowStyle as2;
      as2.lineStyle = ls;
      setArrowStyle("DefFrameArrow", as2);
    }
    
    inline void
    setCmd(const std::ostringstream& cmdstr)
    {
      cmd(cmdstr.str());
    }


    inline void
    updateArrowStyle(const ArrowStyle& as)
    {
      std::ostringstream cmdstr;
      cmdstr << "set style arrow " << as.id << " " << as.arrowType << " " << as.fillType << " size " << as.sizeCoords << " " << as.len << "," << as.angle << "," << as.bAngle << " ls " << as.lineStyle.id;
      PRINT("Arrow style string: " << cmdstr.str());
      cmd(cmdstr.str());
    }

    inline void
    updateLineStyle(const LineStyle& ls)
    {
      std::ostringstream cmdstr;
      cmdstr << "set style line " << ls.id << " lw " << ls.type.width << " pt " << ls.type.pointType << " ps " << ls.type.pointSize << " lc rgb '" << gnuColorNames[(unsigned) ls.color] << "'";
      PRINT("Line style string: " << cmdstr.str());
      cmd(cmdstr.str());
    }

    inline void
    setLineStyle(const string& name, LineStyle& ls)
    {
      if (lineStyles.find(name) != lineStyles.end()) {
        PRINT("LineStyle with requested name already exists...");
        PRINT("Updating style...");
        ls.id = lineStyles[name].id;
        lineStyles[name] = ls;
      } else {
        PRINT("Adding a new LineStyle with requested name...");
        ls.id = ++lsIdCount;
        lineStyles.insert(make_pair(name, ls));
      }
      updateLineStyle(ls);
    }

    inline void
    setArrowStyle(const string& name, ArrowStyle& as)
    {
      if (arrowStyles.find(name) != arrowStyles.end()) {
        PRINT("ArrowStyle with requested name already exists...");
        PRINT("Updating style...");
        as.id = arrowStyles[name].id;
        arrowStyles[name] = as;
      } else {
        PRINT("Adding a new ArrowStyle with requested name...");
        as.id = ++asIdCount;
        arrowStyles.insert(make_pair(name, as));
      }
      updateArrowStyle(as);
    }
    
    inline void
    setCircle(const Matrix<Scalar, 2, 1>& center, const Scalar& radius)
    {
      std::ostringstream cmdstr;
      cmdstr 
        << "set object 1 circle at " 
        << center[0] 
        << "," 
        << center[1] 
        << " size first " 
        << radius 
        << " fs transparent solid 0.35 fc rgb 'red'";
      cmd(cmdstr.str());
      replot();
    }
    
    inline void
    setSphere(const Matrix<Scalar, 3, 1> center, const Scalar& radius)
    {
      std::ostringstream cmdstr;
      cmdstr 
        << "set parametric;\n"
        << "set urange [-pi:pi];\n"
        << "set vrange [-pi/2:pi/2];\n"
        << "fx(v,u) = " << center[0] << "+" << radius << " *cos(v)*cos(u);\n"
        << "fy(v,u) = " << center[1] << "+" << radius << " *cos(v)*sin(u);\n"
        << "fz(v)   = " << center[2] << "+" << radius << " *sin(v);\n";
        if (getNPlots() > 0)
          cmdstr << "replot ";
        else
          cmdstr << "splot ";
        cmdstr << "fx(v,u),fy(v,u),fz(v)";
      cout << cmdstr.str() << endl;  
      cmd(cmdstr.str());
      replot();
    }

    inline void
    setArrow(const Matrix<Scalar, 2, 1>& from, const Matrix<Scalar, 2, 1>& to, const string& as =
      "DefArrow")
    {
      std::ostringstream cmdstr;
      cmdstr << "set arrow from " << from[0] << "," << from[1] << " to " << to[0] << "," << to[1] << " as " << arrowStyles[as].id;
      cmd(cmdstr.str());
      replot();
    }

    inline void
    setArrow(const Matrix<Scalar, 3, 1>& from, const Matrix<Scalar, 3, 1>& to, const string& as =
      "DefArrow")
    {
      std::ostringstream cmdstr;
      cmdstr << "set arrow from " << from[0] << "," << from[1] << "," << from[2] << " to " << to[0] << "," << to[1] << "," << to[2] << " as " << arrowStyles[as].id;
      cmd(cmdstr.str());
      replot();
    }

    inline void
    setFrame(const Matrix<Scalar, 2, 1>& pos = Matrix<Scalar, 3, 1>::Zero(), const double& rot = 0.f,
      const double& frameSize = 0.1, const string& as = "DefFrameArrow")
    {
      Matrix<Scalar, 2, 1> to;
      Matrix<Scalar, 2, 2> rMat;
      MathsUtils::makeRotationZ(rMat, rot);
      for (int i = 0; i < pos.size(); ++i) {
        to = pos;
        to[i] += frameSize;
        to = rMat * to;
        setArrow(pos, to, as);
      }
      plotPoint(pos);
      replot();
    }

    inline void
    setFrame(const Matrix<Scalar, 3, 1>& pos = Matrix<Scalar, 3, 1>::Zero(), const Matrix<Scalar, 3, 1>& rot =
      Matrix<Scalar, 3, 1>::Zero(), const double& frameSize = 0.1, const string& as =
      "DefFrameArrow")
    {
      Matrix<Scalar, 3, 1> to;
      Matrix<Scalar, 3, 3> rMat;
      MathsUtils::makeRotationXYZ(rMat, rot[0], rot[1], rot[2]);
      for (int i = 0; i < pos.size(); ++i) {
        to = pos;
        to[i] += frameSize;
        to = rMat * to;
        setArrow(pos, to, as);
      }
      plotPoint(pos);
      replot();
    }

    inline void 
    plotPoint(const Scalar& x, const Scalar& y, const unsigned& lsId)
    {
      std::ostringstream cmdstr;
      if (getNPlots() > 0)
        cmdstr << "replot ";
      else
        cmdstr << "plot ";
      cmdstr << "\"<echo '" << x << " " << y << "'\" with points ls " << lineStyles["DefLine"].id << "ps " << 1.0;
      cmd(cmdstr.str());
    }
    
    inline void 
    plotPoint(const Matrix<Scalar, 2, 1>& vec)
    {
      std::ostringstream cmdstr;
      if (getNPlots() > 0)
        cmdstr << "replot ";
      else
        cmdstr << "plot ";
      cmdstr << "\"<echo '" << vec[0] << " " << vec[1] << "'\" with points ls " << lineStyles["DefLine"].id << "ps " << 1.0;
      cmd(cmdstr.str());
    }
    
    inline void 
    plotPoint(const Matrix<Scalar, 3, 1>& vec)
    {
      std::ostringstream cmdstr;
      if (getNPlots() > 0)
        cmdstr << "replot ";
      else
        cmdstr << "splot ";
      cmdstr << "\"<echo '" << vec[0] << " " << vec[1] << " " << vec[2] << "'\" with points ls " << lineStyles["DefLine"].id << "ps " << 1.0;
      cmd(cmdstr.str());
    }
    
    inline int 
    getNPlots()
    {
      return nplots;
    }
    
    string title;

    unsigned lsIdCount;
    map<string, LineStyle> lineStyles;

    unsigned asIdCount;
    map<string, ArrowStyle> arrowStyles;

    const double frameSize = 0.025;
  };

  template class PlotEnv<float>;
  template class PlotEnv<double>;
}
