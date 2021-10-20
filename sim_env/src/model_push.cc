#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

const double VEL = -0.3;

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      if (std::ceil(this->model->WorldPose().Pos()[1] * 10) > 0)
	      this->model->SetLinearVel(ignition::math::Vector3d(0, VEL, 0));
      if (std::ceil(this->model->WorldPose().Pos()[1] * 10) <= 0)
	      this->model->SetLinearVel(ignition::math::Vector3d(VEL, 0, 0));
      if (std::ceil(this->model->WorldPose().Pos()[0] * 10) <= -25)
        this->model->GetWorld()->RemoveModel(this->model);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
