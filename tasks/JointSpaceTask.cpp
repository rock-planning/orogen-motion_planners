/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointSpaceTask.hpp"

using namespace motion_planners;

JointSpaceTask::JointSpaceTask(std::string const& name)
    : JointSpaceTaskBase(name)
{
}

JointSpaceTask::JointSpaceTask(std::string const& name, RTT::ExecutionEngine* engine)
    : JointSpaceTaskBase(name, engine)
{
}

JointSpaceTask::~JointSpaceTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See JointSpaceTask.hpp for more detailed
// documentation about them.

bool JointSpaceTask::configureHook()
{
    if (! JointSpaceTaskBase::configureHook())
        return false;
    return true;
}
bool JointSpaceTask::startHook()
{
    if (! JointSpaceTaskBase::startHook())
        return false;
    return true;
}
void JointSpaceTask::updateHook()
{
    JointSpaceTaskBase::updateHook();
    
    // plan for target joint angles
    if(_target_joints_angle.read(target_joints_angle_) == RTT::NewData)
    {
	planner_status_.statuscode = motion_planners::PlannerStatus::INVALID;
        plan(target_joints_angle_);
    }
}

void JointSpaceTask::plan(base::commands::Joints &target_joints_angle)
{
    state(GOAL_RECEIVED);

    //planned_trajectory.clear();
    
   
    planner_->assignPlanningRequest(joints_status_, target_joints_angle, planner_config_.robot_model.planning_group_name, planner_status_);
    
    if(planner_status_.statuscode != PlannerStatus::PLANNING_REQUEST_SUCCESS)
    {
	collision_information_.collision_pair_names = planner_->getCollisionObjectNames();
        _collision_information.write(collision_information_);
	setPlannerStatus(planner_status_);
    }
    else
    {
	setPlannerStatus(planner_status_);
    
	if(planner_->solve(solution_, planner_status_))
	{
	    _planned_trajectory.write(solution_);
	    //state(PATH_FOUND);
	}
	
	setPlannerStatus(planner_status_);
    }
}
void JointSpaceTask::errorHook()
{
    JointSpaceTaskBase::errorHook();
}
void JointSpaceTask::stopHook()
{
    JointSpaceTaskBase::stopHook();
}
void JointSpaceTask::cleanupHook()
{
    JointSpaceTaskBase::cleanupHook();
}
