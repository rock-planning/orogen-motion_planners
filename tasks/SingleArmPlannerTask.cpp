/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SingleArmPlannerTask.hpp"

using namespace motion_planners;

SingleArmPlannerTask::SingleArmPlannerTask(std::string const& name)
    : SingleArmPlannerTaskBase(name)
{
}

SingleArmPlannerTask::~SingleArmPlannerTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SingleArmPlannerTask.hpp for more detailed
// documentation about them.

bool SingleArmPlannerTask::configureHook()
{
    if (! SingleArmPlannerTaskBase::configureHook())
        return false;
  
    initialised_planning_scene_ = false;

    planner_.reset(new motion_planners::MotionPlanners(config_));
    if(!planner_->initialize(planner_status_))
    {        
        setPlannerStatus(planner_status_);
        return false;
    }
    return true;
}
bool SingleArmPlannerTask::startHook()
{
    if (! SingleArmPlannerTaskBase::startHook())
        return false;
    return true;
}
void SingleArmPlannerTask::updateHook()
{
    SingleArmPlannerTaskBase::updateHook();

    // This state is already is verified in the abstract class.
    // TODO: Need to find a elegant solution,
    if(state() == NO_JOINT_STATUS)
        return;
    // adding or removing an object in the world.
    if(_known_object.readNewest(known_object_) == RTT::NewData)
    {
        if(!planner_->handleCollisionObjectInWorld(known_object_))
            state(MODEL_OBJECT_ERROR);
    }

    // adding or removing an object to the manipulator.
    if(_grasp_object.readNewest(grasp_object_) == RTT::NewData)
    {
        if(!planner_->handleGraspObject(grasp_object_))
            state(MODEL_OBJECT_ERROR);        
    }

    // plan for target joint angles
    if(_target_joints_angle.read(target_joints_angle_) == RTT::NewData)
    {
        planner_status_.statuscode = motion_planners::PlannerStatus::INVALID;
        plan(target_joints_angle_);
    }

    // plan for target joint angles
    if(_target_pose.read(target_pose_) == RTT::NewData)
    {
        planner_status_.statuscode = motion_planners::PlannerStatus::INVALID;
        _debug_target_pose.write(target_pose_);
        plan(target_pose_);
    }
    
    if(_target_group_state.read(target_group_state_) ==RTT::NewData)
    {
        planner_status_.statuscode = motion_planners::PlannerStatus::INVALID;
        plan(target_group_state_);
    }

    // get joint angles for start pose
    if (_debug_check_pose.read(debug_check_pose_) == RTT::NewData)
    {
        updatePlanningscene();
        state(GOAL_RECEIVED);

        if (planner_->assignPlanningRequest(joints_status_, debug_check_pose_, planner_status_)) {
            planner_status_.statuscode = motion_planners::PlannerStatus::KINEMATIC_ERROR;
            _debug_check_pose_solution.write(planner_->getGoalJointAngles());
        }
        setPlannerStatus(planner_status_);
    }

    // plan using the predicted trajectory
    if(_predicted_trajectory.read(predicted_trajectory_) == RTT::NewData)
    {
        planner_status_.statuscode = motion_planners::PlannerStatus::INVALID;        
        replan(predicted_trajectory_);
    }
    
    // plan for constrained target pose
    if(_constrainted_pose.read(constrainted_target_) == RTT::NewData)
    {
        planner_status_.statuscode = motion_planners::PlannerStatus::INVALID;        
        plan(constrainted_target_);
    }
}

void SingleArmPlannerTask::updatePlanningscene()
{
    if(convertEnvDataToOctomap())
    {
        if(!initialised_planning_scene_)
        {
            planner_->assignOctomapPlanningScene(input_octree_);
        }

        planner_->updateOctomap(input_octree_);
    }
}

// void SingleArmPlannerTask::writeCollisionObjectNames()
// {
//     collision_information_ = planner_->getCollidedObjectsNames();
//     _collision_information.write(collision_information_);	        
// }

void SingleArmPlannerTask::solve()
{
    state(PLANNING);

    solving_time_ = 0.0;
    if(planner_->solve(solution_, planner_status_, solving_time_))
        _planned_trajectory.write(solution_);

    _debug_num_of_iterations.write(planner_->planner_->getNumOfIterationsUsed());
    
    _debug_initial_trajectory.write(planner_->planner_->getInitialTrajectory());
    
    _solving_time.write(solving_time_);    
}


template<typename T>
void SingleArmPlannerTask::plan(T input)
{
    updatePlanningscene();
    state(GOAL_RECEIVED);

    if(planner_->assignPlanningRequest(joints_status_, input, planner_status_))
    {
        planner_->setStartAndGoal();  // this function will initialise the start and goal for the planner
        solve();
    }
    setPlannerStatus(planner_status_);
    if(planner_status_.statuscode != PlannerStatus::PLANNING_REQUEST_SUCCESS) 
        _collided_links.write(planner_->getCollidedObjectsNames());         
}

void SingleArmPlannerTask::replan(base::JointsTrajectory &input_trajectory)
{
    updatePlanningscene();

    state(GOAL_RECEIVED);

    if(planner_->usePredictedTrajectory(input_trajectory, planner_status_))
        solve();

    setPlannerStatus(planner_status_);

    if(planner_status_.statuscode != PlannerStatus::PLANNING_REQUEST_SUCCESS)
        _collided_links.write(planner_->getCollidedObjectsNames());

}

void SingleArmPlannerTask::errorHook()
{
    SingleArmPlannerTaskBase::errorHook();
}
void SingleArmPlannerTask::stopHook()
{
    SingleArmPlannerTaskBase::stopHook();
}
void SingleArmPlannerTask::cleanupHook()
{
    SingleArmPlannerTaskBase::cleanupHook();
}
