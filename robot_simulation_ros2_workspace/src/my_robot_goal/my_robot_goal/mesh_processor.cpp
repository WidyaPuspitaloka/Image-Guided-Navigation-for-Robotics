#include <rclcpp/rclcpp.hpp>
#include <ros2_igtl_bridge/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <map>
#include <string>
#include <memory>

class BrainMeshProcessor : public rclcpp::Node
{
public:
  BrainMeshProcessor() : Node("brain_mesh_processor")
  {
    // Subscribe to OpenIGTLink point messages
    point_subscriber_ = this->create_subscription<ros2_igtl_bridge::msg::Point>(
      "/IGTL_POINT_IN", 10,
      std::bind(&BrainMeshProcessor::process_point, this, std::placeholders::_1));
      
    // Publisher for visualization markers
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/brain_structures", 10);
      
    // Diagnostics publisher
    status_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/brain_mesh_processor/status", 10);
      
    RCLCPP_INFO(this->get_logger(), "Brain Mesh Processor initialized");
    RCLCPP_INFO(this->get_logger(), "Listening for brain points on /IGTL_POINT_IN");
    
    // Initialize an empty brain model
    this->initialize_empty_brain_models();
    
    // Create a timer to republish brain models periodically
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2), 
      std::bind(&BrainMeshProcessor::publish_brain_models, this));
      
    // Publish initial status
    this->publish_status("Initialized - waiting for points");
  }

private:
  void initialize_empty_brain_models()
  {
    // Initialize containers for different brain structures
    brain_points_["hippocampus"] = std::vector<geometry_msgs::msg::Point>();
    brain_points_["vessels"] = std::vector<geometry_msgs::msg::Point>();
    brain_points_["ventricles"] = std::vector<geometry_msgs::msg::Point>();
    brain_points_["cortex"] = std::vector<geometry_msgs::msg::Point>();
    
    // Default colors for brain structures
    brain_colors_["hippocampus"] = {1.0, 0.8, 0.0, 0.8};  // Yellow/gold
    brain_colors_["vessels"] = {0.8, 0.0, 0.0, 0.7};      // Red
    brain_colors_["ventricles"] = {0.0, 0.0, 1.0, 0.6};   // Blue
    brain_colors_["cortex"] = {0.9, 0.9, 0.9, 0.4};       // Light gray
  }

  void process_point(const ros2_igtl_bridge::msg::Point::SharedPtr msg)
  {
    // Extract name and position
    std::string name = msg->name;
    float x = msg->pointdata.x * 0.001;  // Convert mm to m
    float y = msg->pointdata.y * 0.001;  // Convert mm to m
    float z = msg->pointdata.z * 0.001;  // Convert mm to m
    
    // Identify which brain structure this point belongs to
    std::string structure_name = "unknown";
    if (name.find("hippo") != std::string::npos) {
      structure_name = "hippocampus";
    } else if (name.find("vessel") != std::string::npos) {
      structure_name = "vessels";
    } else if (name.find("ventricle") != std::string::npos) {
      structure_name = "ventricles";
    } else if (name.find("cortex") != std::string::npos) {
      structure_name = "cortex";
    }
    
    // If it's a recognized structure
    if (brain_points_.find(structure_name) != brain_points_.end()) {
      // Create a ROS point
      geometry_msgs::msg::Point point;
      point.x = x;
      point.y = y;
      point.z = z;
      
      // Add point to appropriate structure
      brain_points_[structure_name].push_back(point);
      
      // Log info
      RCLCPP_INFO(this->get_logger(), "Added point to %s: [%.3f, %.3f, %.3f]", 
                  structure_name.c_str(), x, y, z);
                  
      // Update status
      this->publish_status("Added point to " + structure_name + 
                         ": points=" + std::to_string(brain_points_[structure_name].size()));
      
      // Republish brain models
      this->publish_brain_models();
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown structure type: %s", name.c_str());
    }
  }
  
  void publish_brain_models()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    
    // Create markers for each brain structure
    for (const auto& entry : brain_points_) {
      const std::string& structure_name = entry.first;
      const std::vector<geometry_msgs::msg::Point>& points = entry.second;
      
      if (points.empty()) {
        continue;
      }
      
      // Create point cloud marker
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = this->now();
      marker.ns = "brain_structures";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      
      // Set size
      marker.scale.x = 0.01;  // 1cm points
      marker.scale.y = 0.01;
      
      // Set color
      const auto& color = brain_colors_[structure_name];
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = color[3];
      
      // Add points
      marker.points = points;
      
      // Add to array
      marker_array.markers.push_back(marker);
    }
    
    // Publish if there are markers
    if (!marker_array.markers.empty()) {
      marker_publisher_->publish(marker_array);
      RCLCPP_INFO(this->get_logger(), "Published %ld brain structure markers", 
                  marker_array.markers.size());
    }
  }
  
  void publish_status(const std::string& status)
  {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = status;
    status_publisher_->publish(std::move(msg));
  }

  // Member variables
  rclcpp::Subscription<ros2_igtl_bridge::msg::Point>::SharedPtr point_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::map<std::string, std::vector<geometry_msgs::msg::Point>> brain_points_;
  std::map<std::string, std::vector<float>> brain_colors_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrainMeshProcessor>());
  rclcpp::shutdown();
  return 0;
}
