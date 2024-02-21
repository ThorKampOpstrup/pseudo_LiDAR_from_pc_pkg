#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/filter_indices.h"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#define PC_TOPIC "/zed/zed_node/point_cloud/cloud_registered"
// #define PC_TOPIC "/points_raw"
#define CONFIDENCE_MAP_TOPIC "/zed/zed_node/confidence/confidence_map"
#define PSEUDO_LIDAR_TOPIC "/pseudo_LiDAR"

class PCSubscriber : public rclcpp::Node
{
public:
    PCSubscriber() : Node("pc_subscriber")
    {
        pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            PC_TOPIC, 10, std::bind(&PCSubscriber::pc_callback, this, std::placeholders::_1));

        cm_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            CONFIDENCE_MAP_TOPIC, 10, std::bind(&PCSubscriber::cm_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PCSubscriber has been started.");

        pseduo_LiDAR_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(PSEUDO_LIDAR_TOPIC, 10);
    }

private:
    sensor_msgs::msg::PointCloud2 tmp_pc;
    sensor_msgs::msg::Image cm;
    cv::Mat cm_img;
    // const sensor_msgs::msg::Image::SharedPtr& cm;
    // void update_offset_for_subsequent_fields(sensor_msgs::msg::PointCloud2 &pc, int field_size, int start_index)
    // {
    //     for (size_t i = start_index; i < pc.fields.size(); i++)
    //     {
    //         pc.fields[i].offset += field_size;
    //     }
    // }

    // int get_offset(sensor_msgs::msg::PointCloud2 &pc, std::string field_name)
    // {
    //     for (const auto &field : pc.fields)
    //     {
    //         if (field.name == field_name)
    //         {
    //             return field.offset;
    //         }
    //     }

    //     std::cerr << "Field not found" << std::endl;
    //     return -1;
    // }
    // void print_intensity_field(sensor_msgs::msg::PointCloud2 &pc)
    // {
    //     for (uint32_t i = 0; i < pc.width * pc.height; i++)
    //     {
    //         auto offset = get_offset(pc, "intensity");
    //         float intensity;
    //         std::memcpy(&intensity, &pc.data[i * pc.point_step + offset], sizeof(intensity));
    //         std::cout << "intensity: " << intensity << std::endl;
    //     }
    // }

    // void set_intensity(sensor_msgs::msg::PointCloud2 &pc, uint32_t point_index, _Float32 intensity)
    // {

    //     auto offset = get_offset(pc, "intensity");
    //     std::memcpy(&pc.data[point_index * pc.point_step + offset], &intensity, sizeof(intensity));
    // }

    void set_all_intensities(pcl::PointCloud<pcl::PointXYZI> &pc, cv::Mat &cm_img)
    {
        for (int row = 0; row < cm_img.rows; row++)
        {
            for (int col = 0; col < cm_img.cols; col++)
            {
                int index_pc = (row * cm_img.cols + col);
                int intensity_value = (int)cm_img.at<uchar>(row, col);
                pc.points[index_pc].intensity = (float)intensity_value;
            }
        }
    }

    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> pcl_pc;
        pcl::fromROSMsg(*msg, pcl_pc);
        RCLCPP_WARN(this->get_logger(), "NOT A PROBLEM: 'Failed to find match for field 'intensity'");

        set_all_intensities(pcl_pc, cm_img);

        // //remove NAN points from the point cloud
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(pcl_pc, pcl_pc, indices);
        pcl::toROSMsg(pcl_pc, tmp_pc);
        pseduo_LiDAR_publisher_->publish(tmp_pc);

        RCLCPP_INFO(this->get_logger(), "Published pseudo LiDAR\n");
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pseduo_LiDAR_publisher_;

    void cm_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "cm: I heard: [%s]", msg->header.frame_id.c_str());
        // cm = *msg;

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg.get()->encoding);
        cm_img = cv_ptr->image;
        cm_img.convertTo(cm_img, CV_8U);
        // cv::imwrite("saved_image.jpg", cv_ptr->image);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cm_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}