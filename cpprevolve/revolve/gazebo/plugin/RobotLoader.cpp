// Use factory/builder pattern for loading the different modules of the robot.


#include "RobotLoader.h"

namespace gz = gazebo;

using namespace revolve::gazebo;

RobotLoader::RobotLoader()
{
}

RobotLoader::~RobotLoader()
{
}

void RobotLoader::reset()
{
  this->motorFactory_.reset();
  this->sensorFactory_.reset();
}

/////////////////////////////////////////////////
void RobotLoader::Load(::gazebo::physics::ModelPtr _parent, sdf::ElementPtr robotConfiguration)
{
  try {

    this->model_ = _parent;

    if (not _sdf->HasElement("rv:robot_config")) {
        std::cerr << "No `rv:robot_config` element found, controller not initialized." << std::endl;
        return;
    }

    // Load motors
    this->motorFactory_ = GetMotorFactory(_parent);
    this->LoadActuators(robotConfiguration);

    // Load sensors
    this->sensorFactory_ = GetSensorFactory(_parent);
    this->LoadSensors(robotConfiguration);

    // Load brain, this needs to be done after the motors and sensors so they
    // can potentially be reordered.
    this->LoadBrain(robotConfiguration);

    // Call the battery loader
    this->LoadBattery(robotConfiguration);


    // Bind to the world update event to perform some logic
    this->onBeginUpdateConnection = gz::event::Events::ConnectWorldUpdateBegin(
      [this] (const ::gazebo::common::UpdateInfo &_info) {this->OnBeginUpdate(_info);});

    // Call startup function which decides on actuation
    this->Startup(_parent, _sdf);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Error Loading the Robot Loader, expectation: " << std::endl << e.what() << std::endl;
    throw;
  }
}

void RobotLoader::DoUpdate()
{
  auto currentTime = time - initTime_;

  if (brain_)
    brain_->Update(motors_, sensors_, currentTime, actuationTime_);

  if (battery_)
    battery_->Update(currentTime, actuationTime_);
}


/////////////////////////////////////////////////
void RobotLoader::LoadActuators(const sdf::ElementPtr _sdf)
{
  if (not _sdf->HasElement("rv:brain") or not _sdf->GetElement("rv:brain")->HasElement("rv:actuators"))
    return;

  auto actuators = _sdf->GetElement("rv:brain")->GetElement("rv:actuators");

  // Load actuators of type servomotor
  if (actuators->HasElement("rv:servomotor"))
  {
    auto servomotor = actuators->GetElement("rv:servomotor");
    while (servomotor)
    {
      auto servomotorObj = this->motorFactory_->Create(servomotor, this->battery_);
      motors_.push_back(servomotorObj);
      servomotor = servomotor->GetNextElement("rv:servomotor");
    }
  }
}

/////////////////////////////////////////////////
void RobotLoader::LoadSensors(const sdf::ElementPtr _sdf)
{
  if (not _sdf->HasElement("rv:brain") or not _sdf->GetElement("rv:brain")->HasElement("rv:sensors"))
    return;

  auto sensors = _sdf->GetElement("rv:brain")->GetElement("rv:sensors");

  // Load sensors
  auto sensor = sensors->GetElement("rv:sensor");
  while (sensor)
  {
    auto sensorObj = this->sensorFactory_->Create(sensor);
    sensors_.push_back(sensorObj);
    sensor = sensor->GetNextElement("rv:sensor");
  }
}

/////////////////////////////////////////////////
void RobotLoader::LoadBrain(const sdf::ElementPtr _sdf)
{
  if (not _sdf->HasElement("rv:brain"))
  {
    std::cerr << "No robot brain detected, this is probably an error." << std::endl;
    return;
  }

  auto brain_sdf = _sdf->GetElement("rv:brain");
  auto controller_type = brain_sdf->GetElement("rv:controller")->GetAttribute("type")->GetAsString();
  auto learner = brain_sdf->GetElement("rv:learner")->GetAttribute("type")->GetAsString();
  std::cout << "Loading controller " << controller_type << " and learner " << learner << std::endl;

  if ("offline" == learner and "ann" == controller_type)
  {
    brain_.reset(new NeuralNetwork(this->model_, brain_sdf, motors_, sensors_));
  }
  else if ("rlpower" == learner and "spline" == controller_type)
  {
    if (not motors_.empty())
      brain_.reset(new RLPower(this->model_, brain_sdf, motors_, sensors_));
  }
  else if ("bo" == learner and "cpg" == controller_type)
  {
    brain_.reset(new DifferentialCPG(this->model_, _sdf, motors_, sensors_, this->battery_));
  }
  else if ("offline" == learner and "cpg" == controller_type)
  {
    brain_.reset(new DifferentialCPGClean(brain_sdf, motors_));
  }
  else if ("offline" == learner and "cppn-cpg" == controller_type)
  {
    brain_.reset(new DifferentialCPPNCPG(brain_sdf, motors_));
  }
  else
  {
    throw std::runtime_error("Robot brain is not defined.");
  }
}


/////////////////////////////////////////////////
void RobotLoader::LoadBattery(const sdf::ElementPtr _sdf)
{
    if (_sdf->HasElement("rv:battery"))
    {
        sdf::ElementPtr batteryElem = _sdf->GetElement("rv:battery");

        double battery_initial_charge = 0.0;
        try {
            battery_initial_charge = std::stod(batteryElem->GetAttribute("initial_charge")->GetAsString());
        } catch(std::invalid_argument &e) {
            std::clog << "Initial charge of the robot not set, using 0.0" << std::endl;
        }

        this->battery_.reset(new ::revolve::gazebo::Battery(battery_initial_charge)); // set initial battery (joules)
        this->battery_->UpdateParameters(batteryElem);
        this->battery_->ResetVoltage();
        this->battery_->robot_name = this->model_->GetName();
    }
}
