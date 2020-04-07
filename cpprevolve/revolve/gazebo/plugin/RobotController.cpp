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
* Author: Elte Hupkes
* Date: May 3, 2015
*
*/
#include "RobotController.h"

namespace gz = gazebo;

using namespace revolve::gazebo;

/////////////////////////////////////////////////
/// Default actuation time is given and this will be overwritten by the plugin
/// config in Load.
RobotController::RobotController()
    : actuationTime_(0)
    , robotStatesPubFreq_(5)
    , lastRobotStatesUpdateTime_(0)
{
  this->robotLoader = RobotLoader();
}

/////////////////////////////////////////////////
RobotController::~RobotController()
{
  this->robotLoader.reset();

  this->node_.reset();
  this->world_.reset();

  unsubscribe(this->requestSub_);

  cleanup(this->robotStatesPub_);
  cleanup(this->responsePub_);
}

/////////////////////////////////////////////////
void RobotController::OnBeginUpdate(const ::gazebo::common::UpdateInfo &_info) {

  if (not this->robotStatesPubFreq_)
    return;

  auto time = _info.simTime.Double();
  bool sendMessage = this->StatePublicationTime(time);
  if (sendMessage) {
    // Send robot info update message, this only sends the
    // main pose of the robot (which is all we need for now)
    msgs::RobotStates msg;
    gz::msgs::Set(msg.mutable_time(), _info.simTime);

    this->SendState(_info, true);
  }

  for (const auto &model: this->models_to_remove) {
    std::cout << "Removing " << model->GetScopedName() << std::endl;
    gz::transport::requestNoReply(this->world_->Name(), "entity_delete", model->GetScopedName());
    std::cout << "Removed " << model->GetScopedName() << std::endl;
  }
  this->models_to_remove.clear();

}


/////////////////////////////////////////////////
// Process insert and delete requests
void RobotController::HandleRequest(ConstRequestPtr &request)
{
  if (request->request() == "set_robot_state_update_frequency")
  {
    auto frequency = request->data();
    assert(frequency.find_first_not_of( "0123456789" ) == std::string::npos);
    this->robotStatesPubFreq_ = (unsigned int)std::stoul(frequency);
    std::cout << "Setting robot state update frequency to " << this->robotStatesPubFreq_ << "." << std::endl;

    respond(request->id(), request->request(), "success");
  }
  else if (request->request() == "insert_sdf") // TODO also in worldcontroller.
  {
    sdf::SDF robotSDF;
    robotSDF.SetFromString(request->data());

    // Get the model name, store in the expected map
    auto name = robotSDF.Root()->GetElement("model")->GetAttribute("name")->GetAsString();
    double lifespan_timeout = request->dbl_data();

    if (lifespan_timeout > 0)
    {
      boost::mutex::scoped_lock lock(this->death_sentences_mutex_);
      // Initializes the death sentence negative because I don't dare to take the
      // simulation time from this thread.
      death_sentences_[name] = -lifespan_timeout;
    }

    robotSDF.Root()->Reset();
  }
}

/////////////////////////////////////////////////
/// Default startup, bind to CheckUpdate
void RobotController::Startup(::gazebo::physics::ModelPtr, sdf::ElementPtr)
{
  this->updateConnection_ = gz::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&RobotController::CheckUpdate, this, _1));
}

/////////////////////////////////////////////////
void RobotController::CheckUpdate(const ::gazebo::common::UpdateInfo _info)
{
  auto diff = _info.simTime - lastActuationTime_;

  if (diff.Double() > actuationTime_)
  {
    this->DoUpdate(_info);
    lastActuationTime_ = _info.simTime;
  }
}

bool RobotController::StatePublicationTime(const double time)
{
  auto seconds = 1.0 / this->robotStatesPubFreq_;
  return (time - this->lastRobotStatesUpdateTime_) >= seconds;
}

/////////////////////////////////////////////////
void RobotController::Load(::gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  try
  {
    // Store the pointer to the model / world
    this->world_ = _parent->GetWorld();
    this->initTime_ = this->world_->SimTime().Double();

        // Create transport node
    this->node_.reset(new gz::transport::Node());
    this->node_->Init();

    this->requestSub_ = this->node_->Subscribe("~/request", &RobotController::HandleRequest, this);

    // Publisher for inserted models
    this->responsePub_ = this->node_->Advertise< gz::msgs::Response >("~/response");

    // Robot pose publisher
    this->robotStatesPub_ = this->node_->Advertise< revolve::msgs::RobotStates >("~/revolve/robot_states", 500);

    auto robotConfiguration = _sdf->GetElement("rv:robot_config");
    if (robotConfiguration->HasElement("rv:update_rate")) {
      auto updateRate = robotConfiguration->GetElement("rv:update_rate")->Get<double>();
      this->actuationTime_ = 1.0 / updateRate;
    }

    this->robotLoader.Load(_parent, robotConfiguration);

  }
  catch (const std::exception &e)
  {
    std::cerr << "Error Loading the Robot Loader, expectation: " << std::endl << e.what() << std::endl;
    throw;
  }
}

/////////////////////////////////////////////////
/// Default update function simply tells the brain to perform an update
void RobotController::DoUpdate(const ::gazebo::common::UpdateInfo _info)
{
  ///TODO fix this when you have the right amount of initial charge for robots
  // if (battery_->current_charge < 0) { std::exit(0); }

  auto time = _info.simTime.Double();

  this->robotLoader.DoUpdate();

  if (not this->robotStatesPubFreq_)
    return;

  bool send_message = this->StatePublicationTime(time);
  if (send_message)
    this->SendState(_info, false);
}

void RobotController::ProcessDeath(revolve::msgs::RobotState *stateMsg, const ::gazebo::physics::ModelPtr model, const double time)
{
  // Death sentence check
  const std::string name = model->GetName();
  bool death_sentence = false;

  double death_sentence_value = 0;
  {
    boost::mutex::scoped_lock lock_death(death_sentences_mutex_);
    death_sentence = death_sentences_.count(name) > 0;

    if (death_sentence)
      death_sentence_value = death_sentences_[name];
    else
      return; // Does this fare with the scoped_lock?
  }


  if (death_sentence_value < 0) {
    // Initialize death sentence
    death_sentences_[name] = time - death_sentence_value;
    stateMsg->set_dead(false);
  } else {
    bool alive = death_sentence_value > time;
    stateMsg->set_dead(not alive);

    if (not alive) {
      boost::mutex::scoped_lock lock(this->death_sentences_mutex_);
      this->death_sentences_.erase(model->GetName());

      this->models_to_remove.emplace_back(model);
    }
  }
}

void RobotController::SendState(const ::gazebo::common::UpdateInfo _info, const bool process_death)
{
  // Send robot info update message, this only sends the main pose of the robot (which is all we need for now)
  msgs::RobotStates msg;
  gz::msgs::Set(msg.mutable_time(), _info.simTime);
  auto time = _info.simTime.Double();

  boost::recursive_mutex::scoped_lock lock_physics(*this->world_->Physics()->GetPhysicsUpdateMutex());
  for (const auto &model : this->world_->Models())
  {
    if (model->IsStatic())
      continue; // Ignore static models such as the ground and obstacles

    revolve::msgs::RobotState *stateMsg = msg.add_robot_state();
    stateMsg->set_name(model->GetScopedName());
    stateMsg->set_id(model->GetId());

    auto poseMsg = stateMsg->mutable_pose();
    auto relativePose = model->RelativePose();
    gz::msgs::Set(poseMsg, relativePose);

    if (process_death)
      this->ProcessDeath(stateMsg, model, time);

    if (this->robotLoader.battery_)
      stateMsg->set_battery_charge(this->robotLoader.battery_->current_charge);
  }

  if (msg.robot_state_size() > 0)
  {
    this->robotStatesPub_->Publish(msg);
    this->lastRobotStatesUpdateTime_ = time;
  }
}
