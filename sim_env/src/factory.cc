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
    transport::NodePtr node(new transport::Node());

    node->Init(_parent->Name());

    transport::PublisherPtr factoryPub =
    node->Advertise<msgs::Factory>("~/factory");

		msgs::Factory msg;
		msg.set_sdf_filename("model://sphere");

		msgs::Set(msg.mutable_pose(),
		ignition::math::Pose3d(
    ignition::math::Vector3d(0, 2.3, 0.525),
    ignition::math::Quaterniond(0, 0, 0)));

factoryPub->Publish(msg);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}
