#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace my_robot_planning
{
class DjikstraPlanner : public rclcpp::Node
{
public:
    DjikstraPlanner();

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::OccupancyGrid visited_map_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

    nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal);
};
}