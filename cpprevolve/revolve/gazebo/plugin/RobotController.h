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

#include <revolve/gazebo/util/TopicHelper.h>
#include "revolve/gazebo/plugin/RobotLoader.h"

#include "revolve/brains/controller/sensors/Sensor.h"
#include "revolve/brains/controller/actuators/Actuator.h"

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
      virtual void Load(::gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

      /// \brief Update event which, by default, is called periodically
      /// according to the update rate specified in the robot plugin.
      virtual void DoUpdate(const ::gazebo::common::UpdateInfo _info);

    protected:

      // Method called
      virtual void OnBeginUpdate(const ::gazebo::common::UpdateInfo &_info);

      /// \brief Method called at the end of the default `Load` function.
      /// \details This  should be used to initialize robot actuation, i.e.
      /// register some update event. By default, this grabs the
      /// `update_rate` from the robot config pointer, and binds
      virtual void Startup(::gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

      // Listener for analysis requests
      virtual void HandleRequest(ConstRequestPtr &request);

      /// \brief Default method bound to world update event, checks whether the
      /// \brief actuation time has passed and updates if required.
      void CheckUpdate(const ::gazebo::common::UpdateInfo _info);

      RobotLoader robotLoader;

      /// \brief Networking node
      ::gazebo::transport::NodePtr node_;

      /// \brief Actuation time, in seconds
      double actuationTime_;

      /// \brief Time of initialisation
      double initTime_;

      /// \brief Time of the last actuation, in seconds and nanoseconds
      ::gazebo::common::Time lastActuationTime_;

      ::gazebo::transport::SubscriberPtr requestSub_;

      // Response publisher
      ::gazebo::transport::PublisherPtr responsePub_;

      // Pointer to the update event connection
      ::gazebo::event::ConnectionPtr onBeginUpdateConnection;

      // Death sentence list. It collects all the end time for all robots that have
      // a death sentence
      // NEGATIVE DEATH SENTENCES mean total lifetime, death sentence not yet initialized.
      std::map<std::string, double> death_sentences_;

      /// \brief Pointer to the world
      ::gazebo::physics::WorldPtr world_;

      // Publisher for periodic robot poses
      ::gazebo::transport::PublisherPtr robotStatesPub_;

      // Frequency at which robot info is published
      // Defaults to 0, which means no update at all
      unsigned int robotStatesPubFreq_;

      // Last (simulation) time robot info was sent
      double lastRobotStatesUpdateTime_;

      // Mutex for the deleteMap_
      boost::mutex death_sentences_mutex_;

      // boost::mutex world_insert_remove_mutex;
      ::gazebo::physics::Model_V models_to_remove;

    private:
      void ProcessDeath(revolve::msgs::RobotState *stateMsg, const ::gazebo::physics::ModelPtr model, const double time);

      bool StatePublicationTime(const double time);

      void SendState(const ::gazebo::common::UpdateInfo _info, const bool process_death);

      /// \brief Driver update event pointer
      ::gazebo::event::ConnectionPtr updateConnection_;

    };
  } /* namespace gazebo */
} /* namespace revolve */

#endif /* REVOLVE_GAZEBO_PLUGIN_ROBOTCONTROLLER_H_ */
