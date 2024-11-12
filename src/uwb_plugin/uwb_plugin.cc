#include "uwb_plugin.hh"

namespace CustomPlugins
{
  void UWBPlugin::Configure(const Entity &entity,
                            const std::shared_ptr<const sdf::Element> &sdf,
                            EntityComponentManager &ecm,
                            EventManager &/*_eventMgr*/)
  {
    // Get parameters from sdf
    this->robot_namespace = sdf->Get<std::string>("namespace");
    this->target_link = sdf->Get<std::string>("target_link");
    this->update_rate = std::chrono::milliseconds(int(1e3 / sdf->Get<int>("update_rate")));
    this->mean = sdf->Get<double>("mean");
    this->sigma = sdf->Get<double>("sigma");
    this->inter_robot_distances = sdf->Get<bool>("inter_robot_distances");
    this->prob_loss = sdf->Get<double>("prob_loss_measurement");

    // Find link name
    auto link_entity = ecm.Component<components::ParentEntity>(entity);
    while (link_entity)
    {
      auto name_comp = ecm.Component<components::Name>(link_entity->Data());
      if (name_comp)
      {
        this->link_name = name_comp->Data();
        break;
      }
      link_entity = ecm.Component<components::ParentEntity>(link_entity->Data());
    }
    this->link_path = "/empty/" + this->robot_namespace + "/" + this->link_name;

    // Initialize transport publisher
    std::string topic_name = "/uwb_raw_data";
    // std::string topic_name = this->robot_namespace + "/uwb_raw_data";
    this->publisher = this->node.Advertise<gz::msgs::StringMsg>(topic_name);

    // Initialize random generator for noise
    this->normal_distribution = std::normal_distribution<double>(this->mean, this->sigma);

    // Initialize random generator for loss
    this->uniform_distribution = std::uniform_real_distribution<double>(0.0, 1.0);

    // Initialize last update time
    this->last_update_time = std::chrono::steady_clock::now();
  }

  void UWBPlugin::PostUpdate(const UpdateInfo &/*_info*/,
                             const EntityComponentManager &ecm)
  {
    auto current_time = std::chrono::steady_clock::now();
    auto delta_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(current_time - this->last_update_time);
    if (delta_time < this->update_rate)
    {
      return;
    }
    this->last_update_time = current_time;

    std::map<int, gz::math::Vector3d> anchor_poses;

    ecm.Each<components::Name>(
      [&ecm, this, &anchor_poses](const Entity &entity, const components::Name *name) -> bool
      {
        this->process_entity(entity, name, ecm, anchor_poses);
        return true;
      });

    // Compute distance from anchor_pose[-1] to all others
    std::string text_msg = this->robot_namespace + ";";
    for (const auto &anchor : anchor_poses)
    {
      if (anchor.first == -1) continue;

      double distance = (anchor.second - anchor_poses[-1]).Length();
      if (abs(distance) < 1e-6) continue; // Don't consider the anchor on the same robot
      double noisy_distance = distance + this->normal_distribution(generator);

      if (this->uniform_distribution(generator) < this->prob_loss)
      {
        continue;
      }

      text_msg += std::to_string(anchor.first) + "," +
                  std::to_string(anchor.second.X()) + "," +
                  std::to_string(anchor.second.Y()) + "," +
                  std::to_string(anchor.second.Z()) + "," +
                  std::to_string(noisy_distance) + ";";
    }

    // Publish the UWB measurements as string msg
    gz::msgs::StringMsg msg;
    msg.set_data(text_msg);
    this->publisher.Publish(msg);
  }

  void UWBPlugin::process_entity(const Entity &entity,
                                 const components::Name *name,
                                 const EntityComponentManager &ecm,
                                 std::map<int, gz::math::Vector3d>& anchor_poses)
  {
    // get last part of this->link_part
    std::string link_name = this->link_path.substr(this->link_path.find_last_of('/') + 1);
    // check if name->Data() is a substring in this->target_link or link_name
    if (name->Data().find(this->target_link) == std::string::npos &&
        name->Data().find(link_name) == std::string::npos) return;

    std::vector<std::string> names;
    names.push_back(name->Data());

    // Traverse parent entities to construct full path
    auto parent_entity = ecm.Component<components::ParentEntity>(entity);
    // gz::math::Pose3d global_pose = ecm.Component<components::Pose>(entity)->Data(); // Start with local pose
    gz::math::Pose3d global_pose = worldPose(entity, ecm);

    while (parent_entity)
    {
      // auto parent_pose = ecm.Component<components::Pose>(parent_entity->Data());
      // if (parent_pose)
      // {
      //   global_pose = parent_pose->Data() * global_pose; // Accumulate global pose
      // }

      auto parent_name = ecm.Component<components::Name>(parent_entity->Data());
      if (parent_name)
      {
        names.insert(names.begin(), parent_name->Data());
      }
      parent_entity = ecm.Component<components::ParentEntity>(parent_entity->Data());
    }

    // Real anchor
    if (names[2] == this->target_link)
    {
      // convert from "anchor_" to the end to an integer
      std::string id_str = names[1].substr(names[1].find_last_of('_') + 1);
      int id = std::stoi(id_str);
      anchor_poses[id] = global_pose.Pos();
      anchor_poses[id].Z() = 0.0;
      return;
    }

    // Robot anchor
    if (this->inter_robot_distances && names[2].find(this->target_link) != std::string::npos)
    {
      // convert from "robot" to the end to an integer
      std::string id_str = names[1].substr(names[1].find_last_of('t') + 1); // name is robotX
      int id = 100 + std::stoi(id_str);
      anchor_poses[id] = global_pose.Pos();
      anchor_poses[id].Z() = 0.0;
      return;
    }

    // Robot
    std::string full_path;
    for (const auto &part : names)
    {
      full_path += "/" + part;
    }

    if (full_path == this->link_path)
    {
      anchor_poses[-1] = global_pose.Pos();
      anchor_poses[-1].Z() = 0.0;
    }
  }

  // Register plugin
  IGNITION_ADD_PLUGIN(UWBPlugin,
                      gz::sim::System,
                      UWBPlugin::ISystemConfigure,
                      UWBPlugin::ISystemPostUpdate)

  // Register plugin alias
  IGNITION_ADD_PLUGIN_ALIAS(UWBPlugin, "custom::UWBPlugin")
}
