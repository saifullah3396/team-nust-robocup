#include <iostream>
#include <fstream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "../include/BSpline.h"
#include "../include/BSplineNormalFinder.h"

using namespace std;

int main(int argc, char* argv[]) 
{
  VectorXf normal;
  normal.resize(3);
  normal[0] = 1;
  normal[1] = 0;
  normal[2] = 0;
  if (argc == 1) {
    BSpline spline(string(ROOT_DIR) + "/../Config/left_foot_contour.xml");
    //vector<vector<float> > out;
    //vector<float> timeOut;
    //spline.evaluateSpline(out, timeOut, 0);
    VectorXf res;
    if (spline.findNormalToVec(Vector3f(1.0, 0.0, 0), res))
      cout << "res: " << res << endl;
    spline.plotSpline();
  } else if (argc == 2) {
    BSpline spline(argv[1]); 
    vector<vector<float> > out;
    vector<float> timeOut;
    spline.evaluateSpline(out, timeOut, 0);
    //spline.plotSpline();
  }
  return 1;
}
