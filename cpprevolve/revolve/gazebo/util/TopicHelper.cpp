#include "TopicHelper.h"

namespace gz = gazebo;

void unsubscribe(gz::transport::SubscriberPtr &subscription)
{
  if (subscription)
    subscription->Unsubscribe();
}

void cleanup(gz::transport::PublisherPtr &publisher)
{
  if (publisher)
    publisher->Fini();
}

void respond(int const &id, std::string const &topic, std::string const &message)
{
  gz::msgs::Response response;
  response.set_id(id);
  response.set_request(topic);
  response.set_response(message);
  this->responsePub_->Publish(response);
}