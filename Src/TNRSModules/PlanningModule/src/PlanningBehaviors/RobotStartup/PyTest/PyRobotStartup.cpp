#include <boost/python.hpp>
#include "PlanningBehaviors/RobotStartup/RobotStartup.h"

using namespace boost::python;

struct RobotStartupWrap : RobotStartup, wrapper<RobotStartup>
{
  RobotStartupWrap(
    PlanningModule* planningModule, 
    const BehaviorConfigPtr& config,
    const string& name = "RobotStartup") :
    RobotStartup(planningModule, config, name)
  {
  }

  void initiate()
  {
      this->get_override("initiate")();
  }
  
  void update()
  {
      this->get_override("update")();
  }
  
  void finish()
  {
      this->get_override("finish")();
  }
  
  void loadExternalConfig()
  {
      if (override loadExternalConfig = this->get_override("loadExternalConfig"))
        loadExternalConfig();
      RobotStartup::loadExternalConfig();
  }

  void default_loadExternalConfig() { this->RobotStartup::loadExternalConfig(); }
};

BOOST_PYTHON_MODULE(PyRobotStartup)
{
    class_<RobotStartupWrap, boost::noncopyable>("RobotStartup", init<PlanningModule*, const BehaviorConfigPtr&, const string&>())
      .def("initiate", pure_virtual(&RobotStartup::initiate))
      .def("update", pure_virtual(&RobotStartup::update))
      .def("finish", pure_virtual(&RobotStartup::finish))
      .def("loadExternalConfig", &RobotStartup::loadExternalConfig, &RobotStartupWrap::default_loadExternalConfig);
}
