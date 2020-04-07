

#ifndef REVOLVE_TOPICHELPER_H
#define REVOLVE_TOPICHELPER_H

namespace revolve {
    namespace gazebo {

        void unsubscribe(::gazebo::transport::SubscriberPtr &subscription);

        void cleanup(::gazebo::transport::PublisherPtr &publisher);

        void respond(int const &id, std::string const &topic, std::string const &message);
    }
}

#endif  // REVOLVE_TOPICHELPER_H