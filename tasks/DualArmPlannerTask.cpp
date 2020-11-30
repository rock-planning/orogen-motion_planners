/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DualArmPlannerTask.hpp"

using namespace motion_planners;

DualArmPlannerTask::DualArmPlannerTask(std::string const& name)
    : DualArmPlannerTaskBase(name)
{
}

DualArmPlannerTask::~DualArmPlannerTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DualArmPlannerTask.hpp for more detailed
// documentation about them.

bool DualArmPlannerTask::configureHook()
{
    if (! DualArmPlannerTaskBase::configureHook())
        return false;
    
    klc_config_ = _klc_config.value();

    dual_arm_planner_.reset(new motion_planners::DualArmMotionPlanners(config_, klc_config_));
    if(!dual_arm_planner_->initializeDualArmConfig(planner_status_))
    {        
        setPlannerStatus(planner_status_);
        return false;
    }
    
    return true;
}
bool DualArmPlannerTask::startHook()
{
    if (! DualArmPlannerTaskBase::startHook())
        return false;
    return true;
}
void DualArmPlannerTask::updateHook()
{
    DualArmPlannerTaskBase::updateHook();
    
    // This state is already is verified in the abstract class.
    // TODO: Need to find a elegant solution,
    if(state() == NO_JOINT_STATUS)
        return;

    // adding or removing an object in the world.
    if(_known_object.readNewest(known_object_) == RTT::NewData)
    {
        if(!dual_arm_planner_->handleCollisionObjectInWorld(known_object_))
            state(MODEL_OBJECT_ERROR);
    }

    // adding or removing an object to the manipulator.
    if(_grasp_object.readNewest(grasp_object_) == RTT::NewData)
    {
        if(!dual_arm_planner_->handleGraspObject(grasp_object_))
            state(MODEL_OBJECT_ERROR);        
    }

    // plan for target joint angles
    if(_target_joints_angle.read(target_joints_angle_) == RTT::NewData)
    {
        updatePlanningscene();
        planner_status_.statuscode = motion_planners::PlannerStatus::INVALID;
        state(GOAL_RECEIVED);

        if(dual_arm_planner_->assignDualArmPlanningRequest(joints_status_, target_joints_angle_, planner_status_))
        {
            dual_arm_planner_->setStartAndGoal();  // this function will initialise the start and goal for the planner
            state(PLANNING);
            
            if(dual_arm_planner_->solve(dual_arm_solution_, planner_status_, solving_time_))
            {
                _dual_arm_trajectory.write(dual_arm_solution_);
            }           
    
        }
        setPlannerStatus(planner_status_);
        if(planner_status_.statuscode != PlannerStatus::PLANNING_REQUEST_SUCCESS) 
            _collision_information.write(dual_arm_planner_->getCollidedObjectsNames());
    }
}

void DualArmPlannerTask::updatePlanningscene()
{
    if(!initialised_planning_scene_)
    {
        dual_arm_planner_->assignOctomapPlanningScene(input_octree_);
    }

    dual_arm_planner_->updateOctomap(input_octree_);
}

void DualArmPlannerTask::errorHook()
{
    DualArmPlannerTaskBase::errorHook();
}
void DualArmPlannerTask::stopHook()
{
    DualArmPlannerTaskBase::stopHook();
}
void DualArmPlannerTask::cleanupHook()
{
    DualArmPlannerTaskBase::cleanupHook();
}
