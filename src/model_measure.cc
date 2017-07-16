#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_tutorials/inverted_pendulum_states.h"


namespace gazebo
{
  class ModelMeasure : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Store the pointers to the joints
      this->jointPendulum_ = this->model->GetJoint("pendulam_hinge");
      this->wheel1_ = this->model->GetJoint("left_wheel_hinge");
      this->wheel2_ = this->model->GetJoint("left_wheel_hinge_2");
      this->wheel3_ = this->model->GetJoint("right_wheel_hinge");
      this->wheel4_ = this->model->GetJoint("right_wheel_hinge_2");

      k1 = -29.51006;
      k2 = -4.92088;
      k3 = -0.70711;

      this->jointPendulum_->SetForce(0, 50.0);
      

      this->rosPub = this->rosNode.advertise<gazebo_tutorials::inverted_pendulum_states>("pendulum_states", 1000);


      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelMeasure::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      angle = this->jointPendulum_->GetAngle(0);
      double radian = angle.Radian();
      double speed = this->jointPendulum_->GetVelocity(0);
      double wheel_speed = this->wheel1_->GetVelocity(0);
      out = (k1*radian + k2*speed + k3*wheel_speed*0.1);
      this->wheel1_->SetForce(0,-out);
      this->wheel2_->SetForce(0,-out);
      this->wheel3_->SetForce(0,-out);
      this->wheel4_->SetForce(0,-out);
      msg.wheel_torques = out;
      msg.pendulum_angle = radian*57.3;
      msg.pendulum_angle_velocity = speed*57.3;
      msg.chasis_speed = wheel_speed*0.1;
      rosPub.publish(msg);
      ros::spinOnce();
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: math::Angle angle;
    private: double out,k1,k2,k3;
    private: ros::NodeHandle rosNode;
    private: ros::Publisher rosPub;
    private: gazebo_tutorials::inverted_pendulum_states msg;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Pointers to joints
    physics::JointPtr jointPendulum_;
    physics::JointPtr wheel1_;
    physics::JointPtr wheel2_;
    physics::JointPtr wheel3_;
    physics::JointPtr wheel4_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelMeasure)
}
