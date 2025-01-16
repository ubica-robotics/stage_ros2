#include <stage_ros2/stage_node.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

StageNode::Object::Object(
    size_t id, const Stg::Pose &pose, const std::string &name,
    StageNode *node)
    : initialized_(false), id_(id), initial_pose_(pose), name_(name), node_(node)
{
}

size_t StageNode::Object::id() const
{
  return id_;
}
void StageNode::Object::soft_reset()
{
  model->SetPose(this->initial_pose_);
  model->SetStall(false);
}

const std::string &StageNode::Object::name() const
{
  return name_;
}

void StageNode::Object::init()
{
  if (initialized_)
    return;

  model->Subscribe();
  initialized_ = true;
}

void StageNode::publish_object_visualization(StageNode * node)
{
  rclcpp::QoS qos(10);
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_object = node->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_viszualization", qos);
  visualization_msgs::msg::MarkerArray marker_array;

  for (const auto& object: node->objects_) {
    visualization_msgs::msg::Marker marker_text;
    marker_text.id = node->objects_.size() + object->id();
    marker_text.header.frame_id = "map";
    marker_text.header.stamp = object->node()->sim_time_;
    marker_text.pose.position.x = object->model->GetGlobalPose().x;
    marker_text.pose.position.y = object->model->GetGlobalPose().y;
    marker_text.pose.position.z = object->model->GetGlobalPose().z;
    marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker_text.text = object->name();
    marker_text.ns = "ObjectNames";
    marker_text.action = visualization_msgs::msg::Marker::MODIFY;
    marker_text.scale.z = 0.2;
    marker_text.color.r = 1.0;
    marker_text.color.g = 1.0;
    marker_text.color.b = 1.0;
    marker_text.color.a = 1.0;

    visualization_msgs::msg::Marker marker_pose;
    marker_pose.id = object->id();
    marker_pose.header.frame_id = "map";
    marker_pose.header.stamp = object->node()->sim_time_;
    marker_pose.pose.position = marker_text.pose.position;
    marker_pose.pose.orientation = createQuaternionMsgFromYaw(object->model->GetGlobalPose().a);
    marker_pose.type = visualization_msgs::msg::Marker::CUBE;
    marker_pose.ns = "ObjectPoses";
    marker_pose.action = visualization_msgs::msg::Marker::MODIFY;
    marker_pose.scale.x = object->model->GetGeom().size.x;
    marker_pose.scale.y = object->model->GetGeom().size.y;
    marker_pose.scale.z = object->model->GetGeom().size.z;
    marker_pose.color.r = 255.0;
    marker_pose.color.g = 0.0;
    marker_pose.color.b = 0.0;
    marker_pose.color.a = 1.0;

    marker_array.markers.push_back(marker_text);
    marker_array.markers.push_back(marker_pose);
  }
  pub_object->publish(marker_array);
}
