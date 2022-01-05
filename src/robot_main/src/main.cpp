#include <memory>
// client
#include <chrono>
#include <cstdlib>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
// subscriber
#include "interfaces/msg/start_rec.hpp"
// client
#include "interfaces/srv/record_wav.hpp"
#include "interfaces/srv/calc_feature_vals.hpp"
#include "interfaces/srv/calc_proba.hpp"

// subscriber
using std::placeholders::_1;
// client
using namespace std::chrono_literals;

class MainRobotController : public rclcpp::Node
{
public:
    MainRobotController()
        : Node("main_robot_controller_node")
    {
        // [PC->Raspi] 録音を開始するtopic
        start_rec_sub = this->create_subscription<interfaces::msg::StartRec>(
            "start_rec_topic", 10, std::bind(&MainRobotController::rec_cb, this, _1));
    }

private:
    const char *ROBOT_ID = std::getenv("ROBOT_ID");
    const std::string prefix = std::string("robot") + std::string(ROBOT_ID) + std::string("_");
    rclcpp::Subscription<interfaces::msg::StartRec>::SharedPtr start_rec_sub;

    void rec_cb(const interfaces::msg::StartRec::SharedPtr msg) const
    {
        if (msg->flag == true)
        {
            RCLCPP_INFO(this->get_logger(), "I heard: True");

            const std::string client_name = prefix + std::string("record_wav_client");
            std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(client_name);
            // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("record_wav_client");
            const std::string service_name = prefix + std::string("record_wav_srv");
            rclcpp::Client<interfaces::srv::RecordWav>::SharedPtr client = node->create_client<interfaces::srv::RecordWav>(service_name);
            // rclcpp::Client<interfaces::srv::RecordWav>::SharedPtr client = node->create_client<interfaces::srv::RecordWav>("record_wav_srv");

            auto request = std::make_shared<interfaces::srv::RecordWav::Request>();
            // set value
            request->dirname = msg->dir_name;

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
        const std::string client_name = prefix + std::string("calc_features_client");
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(client_name);
        // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("calc_features_client");
        const std::string service_name = prefix + std::string("calc_feature_vals_srv");
        rclcpp::Client<interfaces::srv::CalcFeatureVals>::SharedPtr client = node->create_client<interfaces::srv::CalcFeatureVals>(service_name);
        // rclcpp::Client<interfaces::srv::CalcFeatureVals>::SharedPtr client = node->create_client<interfaces::srv::CalcFeatureVals>("calc_feature_vals_srv");

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
            if (result.get()->success == true)
            {
                this->predict_cb(result.get());
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service calc_feature_vals");
        }
    }

    void predict_cb(const std::shared_ptr<interfaces::srv::CalcFeatureVals::Response> response) const
    {
        const std::string client_name = prefix + std::string("pred_client");
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(client_name);
        // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pred_client");
        const std::string service_name = prefix + std::string("calc_proba_srv");
        rclcpp::Client<interfaces::srv::CalcProba>::SharedPtr client = node->create_client<interfaces::srv::CalcProba>(service_name);
        // rclcpp::Client<interfaces::srv::CalcProba>::SharedPtr client = node->create_client<interfaces::srv::CalcProba>("calc_proba_srv");

        auto request = std::make_shared<interfaces::srv::CalcProba::Request>();
        request->feature_vals = response->feature_vals;

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
            if (result.get()->success == true)
            {
                return;
            }
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