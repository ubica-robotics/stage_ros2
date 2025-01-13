#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

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