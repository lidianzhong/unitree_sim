
#include <unistd.h>
#include <cmath>
#include <std_msgs/msg/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;
// Create a soprt_request class for soprt commond request
class soprt_request : public rclcpp::Node
{
public:
    soprt_request() : Node("req_sender")
    {
        // the req_puber is set to subscribe "/api/sport/request" topic with dt
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        req_ros1_bridge_puber = this->create_publisher<std_msgs::msg::String>("/ros1/move", 10);

        sleep(1);
        run();
    };

private:
    void run()
    {
        // 机器狗站起来
        StandUpToRos1Bridge(req_ros1_bridge);
        req_ros1_bridge_puber->publish(req_ros1_bridge);

        std::cout << "Wait for 3 seconds to stand up..." << std::endl;
        sleep(3);

        // 机器狗进入控制状态
        ReadyToRos1Bridge(req_ros1_bridge);
        req_ros1_bridge_puber->publish(req_ros1_bridge);

        std::cout << "Wait for 2 seconds to start..." << std::endl;
        sleep(2);

        // ============== 算法部分实现 =================
        // 下面给出了一个简单的例子，机器狗以1m/s的速度向前运动
        double vx = 0;   // x velocity
        double vy = 1;   // y velocity
        double vyaw = 0; // yaw velocity

        // Go2 机器狗运动
        sport_req.Move(req, vx, vy, vyaw);
        req_puber->publish(req);

        // 仿真机器狗运动
        MoveToRos1Bridge(req_ros1_bridge, vx, vy, vyaw);
        req_ros1_bridge_puber->publish(req_ros1_bridge);

        // =================== END ======================
    }

    // Move robot to the bridge between ROS2 and ROS1
    void MoveToRos1Bridge(std_msgs::msg::String &req, float vx, float vy, float vyaw)
    {
        nlohmann::json js;
        js["api_id"] = ROBOT_SPORT_API_ID_MOVE;
        js["x"] = vx;
        js["y"] = vy;
        js["yaw"] = vyaw;
        req.data = js.dump();
    }

    void StandUpToRos1Bridge(std_msgs::msg::String &req)
    {
        nlohmann::json js;
        js["api_id"] = ROBOT_SPORT_API_ID_STANDUP;
        req.data = js.dump();
    }

    void ReadyToRos1Bridge(std_msgs::msg::String &req)
    {
        nlohmann::json js;
        js["api_id"] = ROBOT_SPORT_API_ID_READY;
        req.data = js.dump();
    }

    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr req_ros1_bridge_puber;

    unitree_api::msg::Request req;         // Unitree Go2 ROS2 request message
    std_msgs::msg::String req_ros1_bridge; // Unitree Go2 ROS1 request message
    SportClient sport_req;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);            // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_; // Create a timer callback object to send sport request in time intervals

    rclcpp::spin(std::make_shared<soprt_request>()); // Run ROS2 node

    rclcpp::shutdown();

    return 0;
}
