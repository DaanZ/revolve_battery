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

void unsubscribe(gz::transport::SubscriberPtr &subscription)
{
    if (subscription)
        subscription->Unsubscribe();
}

void fini(gz::transport::PublisherPtr &publisher)
{
    if (publisher)
        publisher->Fini();
}

WorldController::~WorldController()
{
    unsubscribe(this->requestSub_);
    unsubscribe(this->responseSub_);
    unsubscribe(this->modelSub_);
    fini(this->requestPub_);
    fini(this->responsePub_);
}

/////////////////////////////////////////////////
void WorldController::Load(
    gz::physics::WorldPtr world,
    sdf::ElementPtr /*_sdf*/)
{
  std::cout << "World plugin loaded." << std::endl;

  // Store the world
  this->world_ = world;

  // Create transport node
  this->node_.reset(new gz::transport::Node());
  this->node_->Init();

  // Subscribe to insert request messages
  this->requestSub_ = this->node_->Subscribe(
      "~/request",
      &WorldController::HandleRequest,
      this);

  // Publisher for `entity_delete` requests.
  this->requestPub_ = this->node_->Advertise< gz::msgs::Request >(
      "~/request");

  // Publisher for inserted models
  this->responseSub_ = this->node_->Subscribe(
      "~/response",
      &WorldController::HandleResponse,
      this);

  // Publisher for inserted models
  this->responsePub_ = this->node_->Advertise< gz::msgs::Response >(
      "~/response");

  // Since models are added asynchronously, we need some way of detecting
  // our model add. We do this using a model info subscriber.
  this->modelSub_ = this->node_->Subscribe(
      "~/model/info",
      &WorldController::OnModel,
      this);

  // Bind to the world update event to perform some logic
  this->updateConnection_ = gz::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&WorldController::OnUpdate, this, _1));

}

void WorldController::Reset()
{
    //this->world_->SimTime().Double();
}

/////////////////////////////////////////////////
void WorldController::OnUpdate(const ::gazebo::common::UpdateInfo &_info)
{
    { // check if there are robots to delete
        std::tuple< ::gazebo::physics::ModelPtr, int> delete_robot;
        {
            boost::mutex::scoped_lock lock(this->deleteMutex_);
            if (not this->delete_robot_queue.empty()) {
                delete_robot = this->delete_robot_queue.front();
                this->delete_robot_queue.pop();
            }
        }
        auto model = std::get<0>(delete_robot);
        auto request_id = std::get<1>(delete_robot);
        if (model) {
            this->world_->RemoveModel(model);

            gz::msgs::Response resp;
            resp.set_id(request_id);
            resp.set_request("delete_robot");
            resp.set_response("success");
            this->responsePub_->Publish(resp);
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
    std::cout << "Processing request `" << request->id()
              << "` to delete robot `" << name << "`" << std::endl;

    auto model = this->world_->ModelByName(name);
    if (model)
    {
      // Tell the world to remove the model
      // Using `World::RemoveModel()` from here crashes the transport
      // library, the cause of which I've yet to figure out - it has
      // something to do with race conditions where the model is used by
      // the world while it is being updated. Fixing this by sending a request
      // to execute in the main tread - `WorldController::OnUpdate` function.
      // Beware, this means that the robot will not be removed while the simulation
      // is paused.

      this->deleteMutex_.lock();
      this->delete_robot_queue.emplace(std::make_tuple(model, request->id()));
      this->deleteMutex_.unlock();

    }
    else
    {
      std::cerr << "Model `" << name << "` could not be found in the world."
                << std::endl;
      gz::msgs::Response resp;
      resp.set_id(request->id());
      resp.set_request("delete_robot");
      resp.set_response("error");
      this->responsePub_->Publish(resp);
    }
  }
  else if (request->request() == "insert_sdf")
  {
    std::cout << "Processing insert model request ID `" << request->id() << "`."
              << std::endl;
    sdf::SDF robotSDF;
    robotSDF.SetFromString(request->data());

    // Get the model name, store in the expected map
    auto name = robotSDF.Root()->GetElement("model")->GetAttribute("name")
                        ->GetAsString();

    this->insertMutex_.lock();
    this->insertMap_[name] = request->id();
    this->insertMutex_.unlock();

    this->world_->InsertModelString(robotSDF.ToString());

    // Don't leak memory
    // https://bitbucket.org/osrf/sdformat/issues/104/memory-leak-in-element
    robotSDF.Root()->Reset();
  }
}

/////////////////////////////////////////////////
void WorldController::OnModel(ConstModelPtr &msg)
{
  auto name = msg->name();

  int id;
  {
    boost::mutex::scoped_lock lock(this->insertMutex_);
    if (this->insertMap_.count(name) <= 0)
    {
      // Insert was not requested here, ignore it
      return;
    }
    id = this->insertMap_[name];
    this->insertMap_.erase(name);
  }

  // Respond with the inserted model
  gz::msgs::Response resp;
  resp.set_request("insert_sdf");
  resp.set_response("success");
  resp.set_id(id);

  msgs::ModelInserted inserted;
  inserted.mutable_model()->CopyFrom(*msg);
  gz::msgs::Set(inserted.mutable_time(), this->world_->SimTime());
  inserted.SerializeToString(resp.mutable_serialized_data());

  this->responsePub_->Publish(resp);

  std::cout << "Model `" << name << "` inserted, world now contains "
            << this->world_->ModelCount() << " models." << std::endl;
}

/////////////////////////////////////////////////
void WorldController::HandleResponse(ConstResponsePtr &response)
{
}
