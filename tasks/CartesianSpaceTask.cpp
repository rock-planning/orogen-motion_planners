/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianSpaceTask.hpp"

using namespace motion_planners;

CartesianSpaceTask::CartesianSpaceTask(std::string const& name)
    : CartesianSpaceTaskBase(name)
{
}

CartesianSpaceTask::CartesianSpaceTask(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianSpaceTaskBase(name, engine)
{
}

CartesianSpaceTask::~CartesianSpaceTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CartesianSpaceTask.hpp for more detailed
// documentation about them.

bool CartesianSpaceTask::configureHook()
{
    if (! CartesianSpaceTaskBase::configureHook())
        return false;
    return true;
}
bool CartesianSpaceTask::startHook()
{
    if (! CartesianSpaceTaskBase::startHook())
        return false;
    return true;
}
void CartesianSpaceTask::updateHook()
{
    CartesianSpaceTaskBase::updateHook();     
    
    // plan for target joint angles
    if(_target_pose.read(target_pose_) == RTT::NewData)
    {
	planner_status_.statuscode = motion_planners::PlannerStatus::INVALID;
        plan(target_pose_);
    }
}

void CartesianSpaceTask::plan(base::samples::RigidBodyState &target_pose)
{
    state(GOAL_RECEIVED);

    //planned_trajectory.clear();    
   
    if(planner_->assignPlanningRequest(joints_status_, target_pose, planner_config_.robot_model.planning_group_name, planner_status_))
    {
    
	if(planner_status_.statuscode != PlannerStatus::PLANNING_REQUEST_SUCCESS)
	{
	    collision_information_.collision_pair_names = planner_->getCollisionObjectNames();
	    _collision_information.write(collision_information_);
	}
	else
	{
	    setPlannerStatus(planner_status_);
	    bool res = planner_->solve(solution_, planner_status_);
	    std::cout<<"Solve result = "<< res<<std::endl;
	    if(res)
	    {std::cout<<"Solution size = "<<solution_.size()<<std::endl;
		_planned_trajectory.write(solution_);
		//state(PATH_FOUND);
	    }
	    
	}
    }
    std::cout<<"klkl "<<planner_status_.statuscode<<"  "<<planner_status_.kinematic_status.statuscode<<std::endl;
    setPlannerStatus(planner_status_);
}

void CartesianSpaceTask::errorHook()
{
    CartesianSpaceTaskBase::errorHook();
}
void CartesianSpaceTask::stopHook()
{
    CartesianSpaceTaskBase::stopHook();
}
void CartesianSpaceTask::cleanupHook()
{
    CartesianSpaceTaskBase::cleanupHook();
}
