
#include <memory>
#include "vector"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "irobot_create_msgs/msg/audio_note_vector.hpp"
#include "irobot_create_msgs/msg/audio_note.hpp"

struct Position
{
    Position(double xin, double yin) : x(xin), y(yin), z(0) {}
    Position() : x(0), y(0), z(0) {}
    double x, y, z;
};

class GoToPickPlace : public rclcpp::Node
{
private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
    Position Pick = Position(2.29, -37.43);
    Position Place = Position(-1.28, -1.96);
    std::vector<Position> goal_vec = {Pick, Place};
    size_t index;

    rclcpp::Publisher<irobot_create_msgs::msg::AudioNoteVector>::SharedPtr pub_audio;

public:
    GoToPickPlace() : Node("GoToPickPlaceNode"), index(0)
    {

        client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        pub_audio = this->create_publisher<irobot_create_msgs::msg::AudioNoteVector>("/cmd_audio", 10);

        RCLCPP_INFO(this->get_logger(), "Waiting For Action Server..");
        client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "Start Sending Goal Position");

        send_goal();
    }

    void send_goal()
    {
        if (index >= goal_vec.size())
        {
            RCLCPP_INFO(this->get_logger(), "ALL POSITION REACH");
            return;
        }

        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();
        goal.pose.pose.position.x = goal_vec[index].x;
        goal.pose.pose.position.y = goal_vec[index].y;
        goal.pose.pose.orientation.z = goal_vec[index].z;
        goal.pose.pose.orientation.w = 1.0;

        auto goal_option = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

        goal_option.result_callback = std::bind(&GoToPickPlace::results_callback, this, std::placeholders::_1);

        client_->async_send_goal(goal, goal_option);
        RCLCPP_INFO(this->get_logger(), "Already Sending Goal Position");
    }

    void results_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
    {

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_INFO(this->get_logger(), "Goal #%zu reached successfully!", index + 1);

            irobot_create_msgs::msg::AudioNoteVector audio_vector_msg;
            irobot_create_msgs::msg::AudioNote audio_msg;
            if (index == 0)
            {
                audio_msg.frequency = 523; // A4
                audio_msg.max_runtime.sec = 1.5;
                audio_msg.max_runtime.nanosec = 500000000;
                audio_vector_msg.notes.push_back(audio_msg);

                pub_audio->publish(audio_vector_msg);
            }
            else if (index == 1)
            {
                audio_msg.frequency = 523; // C5
                audio_msg.max_runtime.sec = 1.5;
                audio_msg.max_runtime.nanosec = 500000000;
                audio_vector_msg.notes.push_back(audio_msg);

                audio_msg.frequency = 440; // C5
                audio_msg.max_runtime.sec = 1.5;
                audio_msg.max_runtime.nanosec = 500000000;
                audio_vector_msg.notes.push_back(audio_msg);

                pub_audio->publish(audio_vector_msg);
            }
            // Proceed to next goal
            index++;
            send_goal();
            break;
        }

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), " Goal #%zu aborted!", index + 1);
            send_goal();
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), " Goal #%zu canceled!", index + 1);
            break;

        default:
            RCLCPP_ERROR(this->get_logger(), " Unknown result code for goal #%zu!", index + 1);
            break;
        }
        // index++;
        // send_goal();
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPickPlace>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
