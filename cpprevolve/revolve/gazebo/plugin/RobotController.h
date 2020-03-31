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

#ifndef REVOLVE_GAZEBO_PLUGIN_ROBOTCONTROLLER_H_
#define REVOLVE_GAZEBO_PLUGIN_ROBOTCONTROLLER_H_

#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

#include <revolve/gazebo/Types.h>
#include <revolve/gazebo/battery/Battery.h>

#include <revolve/msgs/robot_states.pb.h>

namespace revolve
{
  namespace gazebo
  {
    class RobotController
            : public ::gazebo::ModelPlugin
    {
      public:
      /// \brief Constructor
      RobotController();

      /// \brief Destructor
      virtual ~RobotController();

      /// \brief Load method
      void Load(
              ::gazebo::physics::ModelPtr _parent,
              sdf::ElementPtr _sdf) override;

      /// \return Factory class that creates motors for this model
      virtual MotorFactoryPtr MotorFactory(
          ::gazebo::physics::ModelPtr _model);

      /// \return Factory class that creates motors for this robot
      virtual SensorFactoryPtr SensorFactory(
          ::gazebo::physics::ModelPtr _model);

      /// \brief Update event which, by default, is called periodically
      /// according to the update rate specified in the robot plugin.
      virtual void DoUpdate(const ::gazebo::common::UpdateInfo _info);

      protected:
      /// \brief Detects and loads motors in the plugin spec
      virtual void LoadActuators(const sdf::ElementPtr _sdf);

      /// \brief Detects and loads sensors in the plugin spec.
      virtual void LoadSensors(const sdf::ElementPtr _sdf);

      /// \brief Loads the brain from the `rv:brain` element.
      /// \details By default this tries to construct a `StandardNeuralNetwork`.
      virtual void LoadBrain(const sdf::ElementPtr _sdf);

      /// \brief Loads / initializes the robot battery
      virtual void LoadBattery(const sdf::ElementPtr _sdf);

      /// \brief Method called at the end of the default `Load` function.
      /// \details This  should be used to initialize robot actuation, i.e.
      /// register some update event. By default, this grabs the
      /// `update_rate` from the robot config pointer, and binds
      virtual void Startup(
              ::gazebo::physics::ModelPtr _parent,
              sdf::ElementPtr _sdf);

      // Listener for analysis requests
      virtual void HandleRequest(ConstRequestPtr &request);

      /// \brief Default method bound to world update event, checks whether the
      /// \brief actuation time has passed and updates if required.
      void CheckUpdate(const ::gazebo::common::UpdateInfo _info);

      /// \brief Networking node
      ::gazebo::transport::NodePtr node_;

      /// \brief Holds an instance of the motor factory
      MotorFactoryPtr motorFactory_;

      /// \brief Holds an instance of the sensor factory
     SensorFactoryPtr sensorFactory_;

      /// \brief Brain controlling this model
      BrainPtr brain_;

      /// \brief Actuation time, in seconds
      double actuationTime_;

      /// \brief Time of initialisation
      double initTime_;

      /// \brief Time of the last actuation, in seconds and nanoseconds
      ::gazebo::common::Time lastActuationTime_;

      // Publisher for periodic robot poses
      ::gazebo::transport::PublisherPtr robotStatesPub_;

      ::gazebo::transport::SubscriberPtr requestSub_;

      // Response publisher
      ::gazebo::transport::PublisherPtr responsePub_;

      /// \brief Motors in this model
      std::vector< MotorPtr > motors_;

      /// \brief Sensors in this model
      std::vector< SensorPtr > sensors_;

      /// \brief Pointer to the model
      ::gazebo::physics::ModelPtr model_;

      /// \brief Pointer to the world
      ::gazebo::physics::WorldPtr world_;

      // Frequency at which robot info is published
      // Defaults to 0, which means no update at all
      unsigned int robotStatesPubFreq_;

      /// \brief Shared pointer to the battery
      std::shared_ptr<::revolve::gazebo::Battery> battery_;

      // Last (simulation) time robot info was sent
      double lastRobotStatesUpdateTime_;

      private:
      /// \brief Driver update event pointer
      ::gazebo::event::ConnectionPtr updateConnection_;
    };
  } /* namespace gazebo */
} /* namespace revolve */

#endif /* REVOLVE_GAZEBO_PLUGIN_ROBOTCONTROLLER_H_ */
