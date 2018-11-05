#include <fstream>
#include "boost/lexical_cast.hpp"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <bits/stdc++.h>
#include "../Eigen/Dense"

using namespace std;
using namespace Eigen;

struct SplineContour
{
  unsigned order;
  vector<float> knots;
  vector<vector<float> > coeffs;
};

void readSpline(std::fstream& fs, const string& name)
{
  boost::property_tree::ptree pt;
  read_xml(fs, pt);
  vector<SplineContour> footContours;
  BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, pt.get_child("spline")) {
    int i = 0;
    cout << v.first << endl;
    if (v.first == "<xmlattr>") {
      if (v.second.get_child("name").data() != name)  {
        break;
      }
    }
    SplineContour sc;
    BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, v.second) {
      if (v.first == "knots") {
        BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, v.second) {
          sc.knots.push_back(boost::lexical_cast<float>(v.second.data()));
        }
      } else if (v.first == "order") {
        sc.order = v.second.get<unsigned>("value");
      } else if (v.first == "coeffs") {
        BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, v.second) {
          sc.coeffs.push_back(vector<float>());
          BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, v.second) {
            sc.coeffs.back().push_back(boost::lexical_cast<float>(v.second.data()));
          }
          ++i;
        }
      }
    }
    footContours.push_back(sc);
  }
}

int main () 
{
  fstream file;
  file.open("left_foot_contour.xml");
  readSpline(file, "left_foot");
  file.close();
  file.open("right_foot_contour.xml");
  readSpline(file, "right_foot");
  file.close();
  return 1;
}
