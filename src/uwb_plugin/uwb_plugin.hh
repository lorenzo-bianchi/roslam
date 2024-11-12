#ifndef UWB_PLUGIN_HH
#define UWB_PLUGIN_HH

#include <chrono>
#include <iostream>
#include <map>
#include <random>

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include "gz/sim/Util.hh"
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sensors/Manager.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/msgs/stringmsg.pb.h>

using namespace gz;
using namespace sim;
using namespace systems;

namespace CustomPlugins
{
  class UWBPlugin : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPostUpdate
  {
  public:
    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &/*_eventMgr*/) override;

    void PostUpdate(const gz::sim::UpdateInfo &/*_info*/,
                    const gz::sim::EntityComponentManager &ecm) override;

  private:
    void process_entity(const gz::sim::Entity &entity,
                       const gz::sim::components::Name *name,
                       const gz::sim::EntityComponentManager &ecm,
                       std::map<int, gz::math::Vector3d>& anchor_poses);

    // Name of the link to attach to
    std::string link_name;
    std::string target_link;
    std::chrono::steady_clock::time_point last_update_time;
    std::chrono::milliseconds update_rate;
    std::string robot_namespace;
    std::string link_path;
    double mean;
    double sigma;
    bool inter_robot_distances = false;
    double prob_loss = 0.0;

    transport::Node node;
    transport::Node::Publisher publisher;

    std::default_random_engine generator;
    std::normal_distribution<double> normal_distribution;
    std::uniform_real_distribution<double> uniform_distribution;
  };
}

#endif