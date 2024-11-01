#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rail_robot_inspection_msgs/msg/rail_position.hpp>
#include <rail_robot_inspection_msgs/msg/sensor_data.hpp>

namespace orbbec_camera_node {  // 添加命名空间

class OrbbecCameraNode : public rclcpp::Node {
public:
    explicit OrbbecCameraNode(const rclcpp::NodeOptions& options)
        : Node("orbbec_camera_node", options), cameras_enabled_(false) {
        
        rail_position_sub_ = this->create_subscription<rail_robot_inspection_msgs::msg::RailPosition>(
            "rail_position", 10,
            std::bind(&OrbbecCameraNode::railPositionCallback, this, std::placeholders::_1));

        sensor_data_pub_ = this->create_publisher<rail_robot_inspection_msgs::msg::SensorData>(
            "orbbec_data", 10);

        color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&OrbbecCameraNode::colorImageCallback, this, std::placeholders::_1));

        depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&OrbbecCameraNode::depthImageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "OrbbecCameraNode started.");
    }

private:
    void railPositionCallback(const rail_robot_inspection_msgs::msg::RailPosition::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received position signal: Position ID %d", msg->position_id);

        toggleCameras(true);

        if (!color_image_ || !depth_image_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for camera data...");
            return;
        }

        rail_robot_inspection_msgs::msg::SensorData sensor_data;
        sensor_data.header.stamp = this->get_clock()->now();
        sensor_data.position_id = msg->position_id;
        sensor_data.color_image = *color_image_;
        sensor_data.depth_image = *depth_image_;

        sensor_data_pub_->publish(sensor_data);
        RCLCPP_INFO(this->get_logger(), "Published camera data for position %d", msg->position_id);

        toggleCameras(false);
    }

    void colorImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (cameras_enabled_) {
            color_image_ = msg;
        }
    }

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (cameras_enabled_) {
            depth_image_ = msg;
        }
    }

    void toggleCameras(bool enable) {
        if (enable != cameras_enabled_) {
            cameras_enabled_ = enable;
            if (enable) {
                RCLCPP_INFO(this->get_logger(), "Camera data acquisition enabled.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Camera data acquisition disabled.");
                color_image_.reset();
                depth_image_.reset();
            }
        }
    }

    rclcpp::Subscription<rail_robot_inspection_msgs::msg::RailPosition>::SharedPtr rail_position_sub_;
    rclcpp::Publisher<rail_robot_inspection_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;

    sensor_msgs::msg::Image::SharedPtr color_image_;
    sensor_msgs::msg::Image::SharedPtr depth_image_;
    bool cameras_enabled_;
};

}  // namespace orbbec_camera_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera_node::OrbbecCameraNode)