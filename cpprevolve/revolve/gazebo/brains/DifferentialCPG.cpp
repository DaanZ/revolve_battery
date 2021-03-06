/*
 * Copyright (C) 2015-2018 Vrije Universiteit Amsterdam
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Description: TODO: <Add brief description about file purpose>
 * Author: Milan Jelisavcic
 * Date: December 29, 2018
 *
 */

#include <cstdlib>
#include <map>
#include <tuple>

#include "../motors/Motor.h"
#include "../sensors/Sensor.h"

#include "DifferentialCPG.h"

namespace gz = gazebo;

using namespace revolve::gazebo;

/////////////////////////////////////////////////
DifferentialCPG::DifferentialCPG(
    const ::gazebo::physics::ModelPtr &_model,
    const sdf::ElementPtr _settings,
    const std::vector< revolve::gazebo::MotorPtr > &_motors,
    const std::vector< revolve::gazebo::SensorPtr > &_sensors)
    : nextState_(nullptr)
    , input_(new double[_sensors.size()])
    , output_(new double[_motors.size()])
{
  // Create transport node
  this->node_.reset(new gz::transport::Node());
  this->node_->Init();

  auto name = _model->GetName();
  // Listen to network modification requests
//  alterSub_ = node_->Subscribe(
//      "~/" + name + "/modify_diff_cpg", &DifferentialCPG::Modify,
//      this);

  if (not _settings->HasElement("rv:brain"))
  {
    std::cerr << "No robot brain detected, this is probably an error."
              << std::endl;
    throw std::runtime_error("DifferentialCPG brain did not receive settings");
  }

  std::cout << _settings->GetDescription() << std::endl;
  auto motor = _settings->HasElement("rv:motor")
               ? _settings->GetElement("rv:motor")
               : sdf::ElementPtr();
  while(motor)
  {
    if (not motor->HasAttribute("x") or not motor->HasAttribute("y"))
    {
      std::cerr << "Missing required motor attributes (x- and/or y- coordinate)"
                << std::endl;
      throw std::runtime_error("Robot brain error");
    }
    auto motorId = motor->GetAttribute("part_id")->GetAsString();
    auto coordX = std::atoi(motor->GetAttribute("x")->GetAsString().c_str());
    auto coordY = std::atoi(motor->GetAttribute("y")->GetAsString().c_str());

    this->positions_[motorId] = {coordX, coordY};
    this->neurons_[{coordX, coordY, 1}] = {1.f, 0.f, 0.f};
    this->neurons_[{coordX, coordY, -1}] = {1.f, 0.f, 0.f};

//    TODO: Add check for duplicate coordinates

    motor = motor->GetNextElement("rv:motor");
  }

  // Add connections between neighbouring neurons
  for (const auto &position : this->positions_)
  {
    auto name = position.first;
    int x, y; std::tie(x, y) = position.second;

    if (this->connections_.count({x, y, 1, x, y, -1}))
    {
      continue;
    }
    if (this->connections_.count({x, y, -1, x, y, 1}))
    {
      continue;
    }
    this->connections_[{x, y, 1, x, y, -1}] = 1.f;
    this->connections_[{x, y, -1, x, y, 1}] = 1.f;

    for (const auto &neighbour : this->positions_)
    {
      int nearX, nearY;
      std::tie(nearX, nearY) = neighbour.second;
      if ((x+1) == nearX or (y+1) == nearY or (x-1) == nearX or (y-1) == nearY)
      {
        this->connections_[{x, y, 1, nearX, nearY, 1}] = 1.f;
        this->connections_[{nearX, nearY, 1, x, y, 1}] = 1.f;
      }
    }
  }

  // Initialise array of neuron states for Update() method
  this->nextState_ = new double[this->neurons_.size()];
}

/////////////////////////////////////////////////
DifferentialCPG::~DifferentialCPG()
{
  delete[] this->nextState_;
  delete[] this->input_;
  delete[] this->output_;
}

/////////////////////////////////////////////////
void DifferentialCPG::Update(
    const std::vector< revolve::gazebo::MotorPtr > &_motors,
    const std::vector< revolve::gazebo::SensorPtr > &_sensors,
    const double _time,
    const double _step)
{
  boost::mutex::scoped_lock lock(this->networkMutex_);

  // Read sensor data and feed the neural network
  unsigned int p = 0;
  for (const auto &sensor : _sensors)
  {
    sensor->Read(this->input_ + p);
    p += sensor->Inputs();
  }

  this->Step(_time, this->output_);

  // Send new signals to the motors
  p = 0;
  for (const auto &motor: _motors)
  {
    motor->Update(this->output_ + p, _step);
    p += motor->Outputs();
  }
}

void DifferentialCPG::Step(
    const double _time,
    double *_output)
{
  auto i = 0;
  for (const auto &neuron : this->neurons_)
  {
    // The map key is representing x-, y-, and z-coordinates of a neuron and
    // map value represents bias, gain, and current state of the neuron.
    int x, y, z;
    std::tie(x, y, z) = neuron.first;

    double biasA, gainA, stateA;
    std::tie(biasA, gainA, stateA) = neuron.second;

    auto inputA = 0.f;
    for (auto const &connection : this->connections_)
    {
      int x1, y1, z1, x2, y2, z2;
      std::tie(x1, y1, z1, x2, y2, z2) = connection.first;
      auto weightBA = connection.second;

      if (x2 == x and y2 == y and z2 == z)
      {
        auto input = std::get<2>(this->neurons_[{x1, y1, z1}]);
        inputA += weightBA * input + biasA;
      }
    }

    this->nextState_[i] = stateA + (inputA * _time);
    ++i;
  }

  i = 0; auto j = 0;
  for (auto &neuron : this->neurons_)
  {
    double biasA, gainA, stateA;
    std::tie(biasA, gainA, stateA) = neuron.second;

    neuron.second = {biasA, gainA, this->nextState_[i]};
    if (i % 2 == 0)
    {
      _output[j] = this->nextState_[i];
      j++;
    }
    ++i;
  }
}
