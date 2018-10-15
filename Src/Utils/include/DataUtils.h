/**
 * @file Utils/include/DataUtils.h
 *
 * This file declares the class DataUtils.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#ifndef _DATA_UTILS_H_
#define _DATA_UTILS_H_

#include <boost/shared_ptr.hpp>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "Utils/include/DebugUtils.h"
#include "Utils/include/ObstacleType.h"
#include "Utils/include/PostureState.h"
#include "Utils/include/PlanningState.h"
#include "Utils/include/StiffnessState.h"
#include "Utils/include/RobotStateDefinitions.h"

using namespace Eigen;
using namespace std;
using namespace cv;

struct BallInfo;
struct BehaviorInfo;
struct Camera;
struct GoalInfo;
struct Obstacle;
struct ObsObstacles;
struct OccupancyMap;
struct RoboCupGameControlData;
struct TeamBallInfo;
struct TeamRobot;
struct ClientInfo;

namespace Utils
{
  /**
   * @class DataUtils
   * @brief Class that provides functions for variables data type
   *   conversions or handling.
   */
  class DataUtils
  {
  public:
    /**
     * @brief Class constructor.
     */
    DataUtils();

    /**
     * @brief Class destructor.
     */
    ~DataUtils();

    /**
     * @brief Converts a given variable to string.
     * @param var Input variable
     * @return string Output string
     */
    template<typename T>
      static inline string
      varToString(const T& var)
      {
        stringstream ss;
        try {
          ss << var;
        } catch (exception &e) {
          ERROR(
            "Exception caught while converting a variable to string: " << e.what())
        }
        return ss.str();
      }

    /**
     * @brief Converts a given string to variable.
     * @param str Input string
     */
    template<typename T>
      static inline void
      stringToVarTemplate(string const &str, T& var)
      {
        try {
          istringstream istr(str);
          if (!(istr >> std::boolalpha >> var)) throw("Invalid type for string to variable conversion");
        } catch (exception &e) {
          ERROR(
            "Exception caught while converting a variable to string: " << e.what())
        }
      }

    /**
     * @brief Converts a given string to variable.
     * @param str Input string
     */
    static inline void
    stringToVar(string const &str, bool& var)
    {
      stringToVarTemplate<bool>(str, var);
    }

    /**
     * @brief Converts a given string to variable.
     * @param str Input string
     */
    static inline void
    stringToVar(string const &str, int& var)
    {
      stringToVarTemplate<int>(str, var);
    }

    /**
     * @brief Converts a given string to variable.
     * @param str Input string
     */
    static inline void
    stringToVar(string const &str, unsigned& var)
    {
      stringToVarTemplate<unsigned>(str, var);
    }

    /**
     * @brief Converts a given string to variable.
     * @param str Input string
     */
    static inline void
    stringToVar(string const &str, float& var)
    {
      stringToVarTemplate<float>(str, var);
    }

    /**
     * @brief Converts a given string to variable.
     * @param str Input string
     */
    static inline void
    stringToVar(string const &str, double& var)
    {
      stringToVarTemplate<double>(str, var);
    }

    /**
     * @brief Converts a given string to variable.
     * @param str Input string
     */
    static inline void
    stringToVar(string const &str, vector<int>& var)
    {
      string tmp = str;
      tmp = str.substr(1, str.size() - 2);
      vector < string > values = splitString(tmp, ',');
      if (values.size() != var.size()) return;
      for (size_t i = 0; i < values.size(); ++i) {
        stringToVarTemplate<int>(values[i], var[i]);
      }
    }

    /**
     * @brief Converts a given string to variable.
     * @param str Input string
     */
    static inline void
    stringToVar(string const &str, string& var)
    {
      var = str;
    }

    /**
     * @brief Converts a given byte buffer to hex string.
     * @param buffer Input byte buffer
     * @param size Size of the buffer
     * @return string Buffer contents as hex string
     */
    static inline string
    bytesToHexString(const unsigned char*& buffer, const int& size = -1)
    {
      size_t bufferSize = sizeof(buffer);
      if (size != -1) bufferSize = size;
      char* converted = new char[bufferSize * 2 + 1];
      for (size_t i = 0; i < bufferSize; ++i)
        sprintf(&converted[i * 2], "%02X", buffer[i]);
      string temp(converted);
      delete[] converted;
      return temp;
    }

    /**
     * @brief Converts a given byte buffer to string.
     * @param buffer Input byte buffer
     * @param size Size of the buffer
     * @return string Buffer contents
     */
    static inline string
    convertBytesToString(const unsigned char*& buffer, const int& size)
    {
      int bufferSize = sizeof(buffer);
      if (size != -1) bufferSize = size;
      string temp = "";
      for (int i = 0; i < bufferSize; i++) {
        if (buffer[i] == '\n') temp += '\n';
        temp += buffer[i];
      }
      return temp;
    }

    /**
     * @brief Returns a vector of splitted strings with respect to the
     *   given delimiter.
     * @param string Input string
     * @param delim Delimiter
     * @param elems Vector of splitted string components
     * @return vector Vector of splitted string components
     */
    static inline vector<string>&
    splitString(const string& s, const char& delim, vector<string>& elems)
    {
      stringstream ss(s);
      string item;
      while (getline(ss, item, delim)) {
        elems.push_back(item);
      }
      return elems;
    }

    /**
     * @brief Returns a vector of splitted strings with respect to the
     *   given delimiter.
     * @param string Input string
     * @param delim Delimiter
     * @return vector Vector of splitted string components
     */
    static inline vector<string>
    splitString(const string& s, const char& delim)
    {
      std::vector < std::string > elems;
      splitString(s, delim, elems);
      return elems;
    }

    /**
     * @brief This function sets the starting bracket.
     * @param out String to be modified
     * @return void
     */
    static inline void
    getStringStartArray(string& out)
    {
      out += '{';
    }

    /**
     * @brief This function sets the ending bracket.
     * @param out String to be modified
     * @return void
     */
    static inline void
    getStringEndArray(string& out)
    {
      out += '}';
    }

    /**
     * @brief Converts the input boolean to string and adds it to input string.
     * @param out String to be modified
     * @param value Input boolean
     * @return void
     */
    static inline void
    getString(string& out, const bool& value)
    {
      int temp = value;
      out += varToString(temp);
    }

    /**
     * @brief Converts the input numeric variable to string and adds it to input string.
     * @param out String to be modified
     * @param value Input variable
     * @return void
     */
    template<typename T>
      static inline void
      getString(string& out, const T& value)
      {
        out += varToString(value);
      }

    /**
     * @brief Converts the input cv::Point2 variable data to string and adds it
     *   to input string.
     * @param out String to be modified
     * @param value Input variable
     * @return void
     */
    template<typename T>
      static inline void
      getString(string& out, const Point_<T>& value)
      {
        out += string("{") + varToString(value.x) + string(", ") + varToString(
          value.y) + string("}");
      }

    /**
     * @brief Converts the input cv::Point3 variable data to string and adds it
     *   to input string.
     * @param out String to be modified
     * @param value Input variable
     * @return void
     */
    template<typename T>
      static inline void
      getString(string& out, const Point3_<T>& value)
      {
        out += string("{") + varToString(value.x) + string(", ") + varToString(
          value.y) + string(", ") + varToString(value.z) + string("}");
      }

    /**
     * @brief Converts the input ObstacleType variable data to string and adds it
     *   to input string.
     * @param out String to be modified
     * @param type Input variable
     * @return void
     */
    static inline void
    getString(string& out, const ObstacleType& type)
    {
      getString(out, (unsigned) type);
    }

    /**
     * @brief Converts the input PostureState variable data to string and adds it
     *   to input string.
     * @param out String to be modified
     * @param state Input variable
     * @return void
     */
    static inline void
    getString(string& out, const PostureState& state)
    {
      getString(out, (unsigned) state);
    }

    /**
     * @brief Converts the input PlanningState variable data to string and adds it
     *   to input string.
     * @param out String to be modified
     * @param state Input variable
     * @return void
     */
    static inline void
    getString(string& out, const PlanningState& state)
    {
      getString(out, (unsigned) state);
    }

    /**
     * @brief Converts the input StiffnessState variable data to string and adds it
     *   to input string.
     * @param out String to be modified
     * @param state Input variable
     * @return void
     */
    static inline void
    getString(string& out, const StiffnessState& state)
    {
      getString(out, (unsigned) state);
    }

    /**
     * @brief Converts the input BallInfo variable data to string and adds it
     *   to input string.
     * @param out String to be modified
     * @param ballInfo Input variable
     * @return void
     */
    static void
    getString(string& out, const BallInfo& ballInfo);

    /**
     * @brief Converts the input BehaviorInfo variable data to string and adds it
     *   to input string.
     * @param out String to be modified
     * @param behaviorInfo Input variable
     * @return void
     */
    static void
    getString(string& out, const BehaviorInfo& behaviorInfo);


    /**
     * @brief Converts the input Camera variable data to string and adds it to
     *   string.
     * @param out String to be modified
     * @param camera Input variable
     * @return void
     */
    static void
    getString(string& out, const Camera& camera);

    /**
     * @brief Converts the input ClientInfo variable data to string and adds it to
     *   string.
     * @param out String to be modified
     * @param clientInfo Input variable
     * @return void
     */
    static void
    getString(string& out, const ClientInfo& clientInfo);

    /**
     * @brief Converts the input GoalInfo variable data to string and adds it
     *   to input string.
     * @param out String to be modified
     * @param goalInfo Input variable
     * @return void
     */
    static void
    getString(string& out, const GoalInfo& goalInfo);

    /**
     * @brief Converts the input JointRequest variable data to string and adds it
     *   to input string.
     * @param out String to be modified
     * @param req Input variable
     * @return void
     */
    //static void
    //getString(string& out, const JointRequest& req);

    /**
     * @brief Converts the input LedRequest variable data to string and adds it
     *   to input string.
     * @param out String to be modified
     * @param req Input variable
     * @return void
     */
    //static void
    //getString(string& out, const LedRequest& req);

    /**
     * @brief Converts the input Obstacle variable data to string and adds it to
     *   string.
     * @param out String to be modified
     * @param obstacle Input variable
     * @return void
     */
    static void
    getString(string& out, const Obstacle& obstacle);

    /**
     * @brief Converts the input ObsObstacles variable data to string and 
     *   adds it to string.
     * @param out String to be modified
     * @param obsObstacles Input variable
     * @return void
     */
    static void
    getString(string& out, const ObsObstacles& obsObstacles);

    /**
     * @brief Converts the input OccupancyMap variable data to string and adds it to the
     *   string.
     * @param out String to be modified
     * @param OccupancyMap Input variable
     * @return void
     */
    static void
    getString(string& out, const OccupancyMap& occupancyMap);

    /**
     * @brief Converts the input RobotPose2D variable data to string and adds it to
     *   string.
     * @param out String to be modified
     * @param robotPose2D Input variable
     * @return void
     */
    template<typename T>
      static inline void
      getString(string& out, const RobotPose2D<T>& robotPose2D)
      {
        getStringStartArray(out);
        getString(out, robotPose2D.x);
        out += ',';
        getString(out, robotPose2D.y);
        out += ',';
        getString(out, robotPose2D.theta);
        getStringEndArray(out);
      }

    /**
     * @brief Converts the input RoboCupGameControlData variable data
     *   to string and adds it to input string.
     * @param out String to be modified
     * @param data Input variable
     * @return void
     */
    static void
    getString(string& out, const RoboCupGameControlData& data);

    /**
     * @brief Converts the input StiffnessRequest variable data and
     *   adds it to input string.
     * @param out String to be modified
     * @param req Input variable
     * @return void
     */
    //static void
    //getString(string& out, const StiffnessRequest& req);

    /**
     * @brief Converts the input TeamBallInfo variable data to string and adds it
     *   to input string.
     * @param out String to be modified
     * @param ballInfo Input variable
     * @return void
     */
    static void
    getString(string& out, const TeamBallInfo& teamBallInfo);

    /**
     * @brief Converts the input TeamRobot variable data to string and adds it to
     *   string.
     * @param out String to be modified
     * @param teamRobot Input TeamRobot variable
     * @return void
     */
    static void
    getString(string& out, const TeamRobot& teamRobot);

    /**
     * @brief Converts the input VelocityInput variable data to string and adds it to
     *   string.
     * @param out String to be modified
     * @param VelocityInput Input variable
     * @return void
     */
    template<typename T>
      static inline void
      getString(string& out, const VelocityInput<T>& velocityInput)
      {
        getStringStartArray(out);
        getString(out, velocityInput.dX);
        out += ',';
        getString(out, velocityInput.dY);
        out += ',';
        getString(out, velocityInput.dTheta);
        getStringEndArray(out);
      }

    /**
     * @brief Converts the input cv::Mat variable data to string and adds it to
     *   string.
     * @param out String to be modified
     * @param mat Input variable
     * @return void
     */
    static inline void
    getString(string& out, const Mat& mat)
    {
      out += 1;
    }

    /**
     * @brief Converts the input Vector2f variable data to string and adds it to
     *   string.
     * @param out String to be modified
     * @param vec Input variable
     * @return void
     */
    static void
    getString(string& out, const Matrix<float, 2, 1, 0, 2, 1>& vec);

    /**
     * @brief Converts the input RotatedRect variable data to string and adds it to
     *   string.
     * @param out String to be modified
     * @param teamRobot Input RotatedRect variable
     * @return void
     */
    static void
    getString(string& out, const RotatedRect& rRect);

    /**
     * @brief Converts the input Matrix4f variable data to string and adds it to
     *   string.
     * @param out String to be modified
     * @param eMat Input variable
     * @return void
     */
    static inline void
    getString(string& out, const Matrix4f& eMat)
    {
      out += 1; // TEMPFIX
    }

    /**
     * @brief Converts the input vector of variable data to string and adds it to
     *   string.
     * @param out String to be modified.
     * @param value Input vector
     * @return void
     */
    template<typename T>
      static inline void
      getString(string& out, const vector<T>& value)
      {
        size_t size = value.size();
        int commaLimit = size - 1;
        getStringStartArray(out);
        for (size_t j = 0; j < size; ++j) {
          getString(out, value[j]);
          if (j != commaLimit) {
            out += ',';
          }
        }
        getStringEndArray(out);
      }
  };
}
#endif //! _DATA_UTILS_H_
