#include <stage_ros2/stage_node.hpp>

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
  model->SetPose(initial_pose_);
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

void StageNode::Object::set_pose_rel(const std::shared_ptr<const Vehicle>& vehicle, const Stg::Pose rel_pose)
{
  Stg::Pose v_pose = vehicle->positionmodel->GetGlobalPose();
  double glob_x = v_pose.x + std::cos(v_pose.a) * rel_pose.x - std::sin(v_pose.a) * rel_pose.y;
  double glob_y = v_pose.y + std::sin(v_pose.a) * rel_pose.x + std::cos(v_pose.a) * rel_pose.y;
  double glob_yaw = v_pose.a + rel_pose.a;
  model->SetPose(Stg::Pose(glob_x, glob_y, rel_pose.z, glob_yaw));
}