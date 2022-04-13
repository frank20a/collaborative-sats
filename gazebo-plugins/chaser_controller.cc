#ifndef _CHASER_CONTROLLER_HH_
#define _CHASER_CONTROLLER_HH_

#define POS_X 1
#define NEG_X 2
#define POS_Y 4
#define NEG_Y 8
#define POS_Z 16
#define NEG_Z 32
#define POS_YAW 64
#define NEG_YAW 128

// std
#include <string>

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// ros
#include <gazebo_ros/node.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;

class ControllerNode : public rclcpp::Node {
    public:
        ControllerNode() : Node("chaser_controller") {};
};

namespace gazebo {
    class ChaserController : public ModelPlugin {

        public: 
            ChaserController() {}
            ~ChaserController() {}

            void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
                cerr << "\n\n\nPlugin loaded\n\n\n";
                
                this->ros_node_ = gazebo_ros::Node::Get(sdf)
                this->pub_ = ros_node_->create_subscription<std_msgs::msg::String>(
                    "chaser_controller",
                    rclcpp::SensorDataQoS(),
                    std::bind(&ChaserController::OnRosMsg, this, std::placeholders::_1)
                );
            }


        private: 
            void OnRosMsg(const std_msgs::msg::String::SharedPtr msg) {
                cerr << "Received: " << msg->data.c_str() << endl;
            }

        private:  
            gazebo_ros::Node::SharedPtr ros_node_{nullptr};
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_{nullptr};

    };

    GZ_REGISTER_MODEL_PLUGIN(ChaserController)
}

#endif