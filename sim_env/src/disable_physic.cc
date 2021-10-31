#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <boost/bind.hpp>

namespace gazebo
{
	class DisablePhysic : public WorldPlugin
	{

		public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
			{
				this->_world = _world;
				this->_update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
						boost::bind(&DisablePhysic::on_update, this, _1));
			}
		public: void on_update(const gazebo::common::UpdateInfo& /*info*/) {
				if (this->_world->PhysicsEnabled())
					this->_world->SetPhysicsEnabled(false);
			}
		public: physics::WorldPtr _world;
		public: event::ConnectionPtr _update_connection;
	};
	GZ_REGISTER_WORLD_PLUGIN(DisablePhysic)
}
