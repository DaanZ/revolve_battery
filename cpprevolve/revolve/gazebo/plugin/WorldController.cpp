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
* Date: June 6, 2015
*
*/

#include <string>

#include "WorldController.h"

namespace gz = gazebo;

using namespace revolve::gazebo;

/////////////////////////////////////////////////
WorldController::WorldController()
    : delete_robot_queue()
{
}

WorldController::~WorldController()
{
  unsubscribe(this->requestSub_);
  unsubscribe(this->responseSub_);
  unsubscribe(this->modelSub_);
  cleanup(this->requestPub_);
  cleanup(this->responsePub_);
}

/////////////////////////////////////////////////
void WorldController::Load(gz::physics::WorldPtr world, sdf::ElementPtr)
{
  gz::physics::PhysicsEnginePtr physicsEngine = world->Physics();
  assert(physicsEngine != nullptr);

  // Turn on threading
  physicsEngine->SetParam("thread_position_correction", true);
  physicsEngine->SetParam("island_threads", 8);

  std::cout << "World plugin loaded." << std::endl;

  // Store the world
  this->world_ = world;

  // Create transport node
  this->node_.reset(new gz::transport::Node());
  this->node_->Init();

  // Process request message
  this->requestSub_ = this->node_->Subscribe("~/request", &WorldController::HandleRequest, this);
  this->requestPub_ = this->node_->Advertise< gz::msgs::Request >("~/request");

  // Response Topic
  this->responseSub_ = this->node_->Subscribe("~/response", &WorldController::HandleResponse, this);
  this->responsePub_ = this->node_->Advertise< gz::msgs::Response >("~/response");

  // Since models are added asynchronously, we need some way of detecting
  // our model add. We do this using a model info subscriber.
  this->modelSub_ = this->node_->Subscribe("~/model/info", &WorldController::OnModel, this);

  // Bind to the world update event to perform some logic
  this->onEndUpdateConnection = gz::event::Events::ConnectWorldUpdateEnd([this] () {this->OnEndUpdate();});
}

void WorldController::Reset()
{
    //this->world_->SimTime().Double();
}

void WorldController::OnEndUpdate()
{
  // check if there are robots to delete
  this->DeleteRobot();

  // check if there are robots to insert
  this->InsertRobot();
}

void WorldController::DeleteRobot()
{
  // Check if a robot is left in the delete robot queue.
  std::tuple< ::gazebo::physics::ModelPtr, int> delete_robot;
  {
    boost::mutex::scoped_lock lock(this->deleteMutex_);
    if (not this->delete_robot_queue.empty()) {
      // NOTE only deletes one robot, Why?
      delete_robot = this->delete_robot_queue.front();
      this->delete_robot_queue.pop();
    }
  }

  // If a robot is to be removed, we get the model and remove it from the world, confirming by sending a response.
  auto model = std::get<0>(delete_robot);
  if (model)
  {
    // boost::recursive_mutex::scoped_lock lock_physics(*this->world_->Physics()->GetPhysicsUpdateMutex());
    this->world_->RemoveModel(model);

    auto request_id = std::get<1>(delete_robot);
    respond(request_id, "delete_robot", "success");
  }
}

void WorldController::InsertRobot()
{
  boost::mutex::scoped_lock lock(this->insertMutex_);
  for (auto &iterator: this->insertMap_)
  {
    bool &insert_operation_pending = std::get<2>(iterator.second);
    if (insert_operation_pending)
    { // Start insert operation!
      // boost::recursive_mutex::scoped_lock lock_physics(*this->world_->Physics()->GetPhysicsUpdateMutex());
      const std::string &robotSDF = std::get<1>(iterator.second);
      this->world_->InsertModelString(robotSDF);
      insert_operation_pending = false;
      break;
    }
  }
}

/////////////////////////////////////////////////
// Process insert and delete requests
void WorldController::HandleRequest(ConstRequestPtr &request)
{
  if (request->request() == "delete_robot")
  {
    auto name = request->data();
    std::cout << "Processing request `" << request->id() << "` to delete robot `" << name << "`" << std::endl;
    std::cerr << "Model `" << name << "` could not be found in the world." << std::endl;

    respond(request->id(), request->request(), "error");
  }
  else if (request->request() == "insert_sdf")
  {
    std::cout << "Processing insert model request ID `" << request->id() << "`." << std::endl;
    sdf::SDF robotSDF;
    robotSDF.SetFromString(request->data());

    // Get the model name, store in the expected map
    auto name = robotSDF.Root()->GetElement("model")->GetAttribute("name")->GetAsString();
    {
        boost::mutex::scoped_lock lock(this->insertMutex_);
        this->insertMap_[name] = std::make_tuple(request->id(), robotSDF.ToString(), true);
    }

    //TODO insert here, it's better
    //this->world_->InsertModelString(robotSDF.ToString());

    // Don't leak memory
    // https://bitbucket.org/osrf/sdformat/issues/104/memory-leak-in-element
    robotSDF.Root()->Reset();
  }
}

/////////////////////////////////////////////////
void WorldController::OnModel(ConstModelPtr &msg)
{
  auto name = msg->name();
  std::cout << "WorldController::OnModel(" << name << ')' << std::endl;

  int id;
  bool insert_operation_pending;
  {
    boost::mutex::scoped_lock lock(this->insertMutex_);
    if (this->insertMap_.count(name) <= 0)
      return;  // Insert was not requested here, ignore it

    const std::tuple<int, std::string, bool> &entry = this->insertMap_[name];
    id = std::get<0>(entry);

    insert_operation_pending = std::get<2>(entry);
    if (insert_operation_pending)
      return; // Insert operation has not been done yet (but you should never be here, because we are in the "OnModel" function

    this->insertMap_.erase(name);
  }

  // Respond with the inserted model
  // TODO not able to use Respond function because of inserted serialize to string.
  gz::msgs::Response response;
  response.set_request("insert_sdf");
  response.set_response("success");
  response.set_id(id);

  msgs::ModelInserted inserted;
  inserted.mutable_model()->CopyFrom(*msg);
  gz::msgs::Set(inserted.mutable_time(), this->world_->SimTime());
  inserted.SerializeToString(response.mutable_serialized_data());

  this->responsePub_->Publish(response);

  std::cout << "Model `" << name << "` inserted, world now contains " << this->world_->ModelCount() << " models." << std::endl;
}

/////////////////////////////////////////////////
void WorldController::HandleResponse(ConstResponsePtr &response)
{
  if (response->request() not_eq "entity_delete")
    return;
}
