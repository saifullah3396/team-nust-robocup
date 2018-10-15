#include "../include/PlotEnv.h"

using namespace GnuPlotEnv;

int
main()
{
  Vector2d xRange;
  Vector2d yRange;
  Vector2d zRange;
  xRange[0] = -1.0;
  xRange[1] = 1.0;
  yRange[0] = -1.0;
  yRange[1] = 1.0;
  zRange[0] = -1.0;
  zRange[1] = 1.0;
  PlotEnv::set_terminal_std("wxt");
  auto pe = PlotEnv(
    "MyGraph",
    "x-Axis",
    "y-Axis",
    "z-Axis",
    xRange,
    yRange,
    zRange);
  pe.plotfile_xyz(
    "/home/sensei/team-nust-robocup-spl/Logs/Robots/Sim/KickModule/surfacePointsLeft.txt",
    1,
    2,
    3,
    "SurfacePlot");
  pe.setArrow(Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0));
  pe.setFrame(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0));
  pe.showonscreen();
  while (true)
    ;
  return 1;
}
