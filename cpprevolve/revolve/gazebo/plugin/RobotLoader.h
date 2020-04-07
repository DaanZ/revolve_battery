/*
* Copyright (C) 2017 Vrije Universiteit Amsterdam
*
* Licensed under the Apache License, Version 2.0 (the "License");
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
* Author: Elte Hupkes
* Created on: May 3, 2015
*
*/

#ifndef REVOLVE_GAZEBO_PLUGIN_ROBOTLOADER_H_
#define REVOLVE_GAZEBO_PLUGIN_ROBOTLOADER_H_

#include <vector>

#include  <stdexcept>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>

#include <revolve/gazebo/Types.h>
#include <revolve/gazebo/battery/Battery.h>
#include <revolve/gazebo/motors/MotorFactory.h>
#include <revolve/gazebo/sensors/SensorFactory.h>
#include <revolve/gazebo/brains/Brains.h>
#include <revolve/gazebo/battery/Battery.h>

#include <revolve/msgs/robot_states.pb.h>

#include <revolve/gazebo/util/TopicHelper.h>

#include "revolve/brains/controller/sensors/Sensor.h"
#include "revolve/brains/controller/actuators/Actuator.h"

namespace revolve
{
  namespace gazebo
  {
    class RobotLoader
    {
    public:
      /// \brief Constructor
      RobotLoader();

      /// \brief Destructor
      ~RobotLoader();

      void reset();

      /// \brief Load method
      void Load(::gazebo::physics::ModelPtr _parent, sdf::ElementPtr robotConfiguration);

      void DoUpdate();

      // TODO make private
      /// \brief Shared pointer to the battery
      std::shared_ptr<::revolve::gazebo::Battery> battery_;

    protected:

      /// \brief Holds an instance of the motor factory
      MotorFactoryPtr motorFactory_;

      /// \brief Holds an instance of the sensor factory
      SensorFactoryPtr sensorFactory_;

      /// \brief Brain controlling this model
      BrainPtr brain_;

      /// \brief Motors in this model
      std::vector< MotorPtr > motors_;

      /// \brief Sensors in this model
      std::vector< SensorPtr > sensors_;

      /// \brief Pointer to the model
      ::gazebo::physics::ModelPtr model_;

    private:
      /// \brief Detects and loads motors in the plugin spec
      void LoadActuators(const sdf::ElementPtr _sdf);

      /// \brief Detects and loads sensors in the plugin spec.
      void LoadSensors(const sdf::ElementPtr _sdf);

      /// \brief Loads the brain from the `rv:brain` element.
      /// \details By default this tries to construct a `StandardNeuralNetwork`.
      void LoadBrain(const sdf::ElementPtr _sdf);

      /// \brief Loads / initializes the robot battery
      void LoadBattery(const sdf::ElementPtr _sdf);

    };
  } /* namespace gazebo */
} /* namespace revolve */

#endif /* REVOLVE_GAZEBO_PLUGIN_ROBOTLOADER_H_ */
