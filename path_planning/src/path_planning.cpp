#include "path_planning/path_planning.h"

PathPlanning::PathPlanning() : Node("path_planning")
{
    display_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",10);
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 
        10,
        std::bind(&PathPlanning::joystickCallback, this, std::placeholders::_1));

    obj_.header.frame_id = "world";
    obj_.id = "obstacles";
}

void PathPlanning::addObstacles()
{
    addBoxObstacle();
    addConeObstacle();
    psi_.applyCollisionObject(obj_);
}

void PathPlanning::initMoveGroup()
{
    // initialize movegroup
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "rv_3s_arm");

    move_group_ ->setNumPlanningAttempts(1000);
    move_group_ ->setPlanningTime(300.0);
}

void PathPlanning::planMotion()
{
    move_group_->setJointValueTarget(goal_angles_);
    plan_succeeded_ = move_group_->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS;
}

void PathPlanning::visualizeMotion()
{
    if (plan_succeeded_)
    {
        display_msg_.trajectory_start = plan_.start_state_;
        display_msg_.trajectory.push_back(plan_.trajectory_);
        display_pub_->publish(display_msg_);
    }
}

void PathPlanning::moveJoints()
{
    // if move plan succeeded, move robot
    if (plan_succeeded_)
    {
        if (move_group_->execute(plan_) != moveit::core::MoveItErrorCode::SUCCESS)
            RCLCPP_INFO(this->get_logger(), "Execution failed");
    }
    else // print error otherwise
    {
        RCLCPP_INFO(this->get_logger(), "Plan failed");
    }
}

void PathPlanning::addBoxObstacle()
{
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.5, 0.5, 0.25};

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.6;
    pose.position.y = 0.7;
    pose.position.z = 0.6;
    pose.orientation.w = 1.0;

    obj_.primitives.push_back(primitive);
    obj_.primitive_poses.push_back(pose);
    obj_.operation = obj_.ADD;

    RCLCPP_INFO(this->get_logger(), "Box obstacle added");
}

void PathPlanning::addConeObstacle()
{
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CONE;
    primitive.dimensions = {1.0, 0.1};
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.6;
    pose.position.y = 0.0;
    pose.position.z = 0.5;
    pose.orientation.w = 1.0;

    obj_.primitives.push_back(primitive);
    obj_.primitive_poses.push_back(pose);
    obj_.operation = obj_.ADD;

    RCLCPP_INFO(this->get_logger(), "cone obstacle added");
}

void PathPlanning::joystickCallback(const sensor_msgs::msg::Joy & msg)
{
    // only  once per press
    if ((msg.buttons[0] == 1) && !a_button_prev_)
        moveJoints();

    a_button_prev_ = msg.buttons[0];
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PathPlanning>();

    node->initMoveGroup();
    node->addObstacles();
    rclcpp::spin_some(node);

    node->planMotion();
    node->visualizeMotion();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}