#include <memory>
// client
#include <chrono>
#include <cstdlib>
#include <string.h>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
// subscriber
#include "std_msgs/msg/bool.hpp"
// client
#include "interfaces/srv/record_wav.hpp"
#include "interfaces/srv/calc_feature_vals.hpp"

// subscriber
using std::placeholders::_1;
// client
using namespace std::chrono_literals;

class MainRobotController : public rclcpp::Node
{
public:
    MainRobotController()
        : Node("main_robot_controller")
    {
        // [PC->Raspi] 録音を開始するtopic
        start_rec_sub = this->create_subscription<std_msgs::msg::Bool>(
            "start_rec_topic", 10, std::bind(&MainRobotController::rec_cb, this, _1));
    }

private:
    const char *ROBOT_ID = std::getenv("ROBOT_ID");
    mutable int file_no = 0;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_rec_sub;

    void rec_cb(const std_msgs::msg::Bool::SharedPtr msg) const
    {
        if (msg->data == true)
        {
            RCLCPP_INFO(this->get_logger(), "I heard: True");

            const std::string client_name = std::string("record_wav_client_") + std::string(ROBOT_ID);
            std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(client_name);
            const std::string service_name = std::string("record_wav_srv_") + std::string(ROBOT_ID);
            rclcpp::Client<interfaces::srv::RecordWav>::SharedPtr client = node->create_client<interfaces::srv::RecordWav>(service_name);

            auto request = std::make_shared<interfaces::srv::RecordWav::Request>();
            // define dirname(date)
            time_t now = time(NULL);
            struct tm *pnow = localtime(&now);
            std::string dirname = std::to_string(pnow->tm_year + 1900) + "-" + std::to_string(pnow->tm_mon + 1) + "-" + std::to_string(pnow->tm_mday) + "-no" + std::to_string(file_no);
            file_no++;
            // set value
            request->dirname = dirname;

            while (!client->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            auto result = client->async_send_request(request);

            std::string file_path;
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                file_path = result.get()->file_path;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "File path: %s", file_path);
                if (result.get()->success == true)
                {
                    this->calc_features_cb(result.get());
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service record_wav");
            }
        }
    }

    void calc_features_cb(const std::shared_ptr<interfaces::srv::RecordWav::Response> response) const
    {
        const std::string client_name = std::string("calc_feature_vals_client_") + std::string(ROBOT_ID);
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(client_name);
        const std::string service_name = std::string("calc_feature_vals_srv_") + std::string(ROBOT_ID);
        rclcpp::Client<interfaces::srv::CalcFeatureVals>::SharedPtr client = node->create_client<interfaces::srv::CalcFeatureVals>(service_name);

        auto request = std::make_shared<interfaces::srv::CalcFeatureVals::Request>();
        request->file_dir_path = response->file_path;

        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            return;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service record_wav");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainRobotController>());
    rclcpp::shutdown();
    return 0;
}