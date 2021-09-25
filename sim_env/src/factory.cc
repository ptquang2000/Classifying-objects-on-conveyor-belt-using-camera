#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
class Factory : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Create a new transport node
    transport::NodePtr node(new transport::Node());

    // Initialize the node with the world name
    node->Init(_parent->Name());

    // Create a publisher on the ~/factory topic
    transport::PublisherPtr factoryPub =
    node->Advertise<msgs::Factory>("~/factory");

    // Create the message
    msgs::Factory msg;

    // Model file to load
    msg.set_sdf_filename("model://a_sphere");

    // Pose to initialize the model to
    msgs::Set(msg.mutable_pose(),
        ignition::math::Pose3d(
          ignition::math::Vector3d(0, 2.3, 0.525),
          ignition::math::Quaterniond(0, 0, 0)));

    // Send the message
    factoryPub->Publish(msg);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}