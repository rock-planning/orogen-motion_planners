/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PlannerTask.hpp"

using namespace motion_planners;

PlannerTask::PlannerTask(std::string const& name)
    : PlannerTaskBase(name)
{
}

PlannerTask::PlannerTask(std::string const& name, RTT::ExecutionEngine* engine)
    : PlannerTaskBase(name, engine)
{
}

PlannerTask::~PlannerTask()
{
}

bool PlannerTask::configureHook()
{
    if (! PlannerTaskBase::configureHook())
        return false;

    config_ = _config.value();    
    initialised_planning_scene_ = false;

    planner_.reset(new motion_planners::MotionPlanners(config_));
    if(!planner_->initialize(planner_status_))
    {
		
        setPlannerStatus(planner_status_);
        return false;
    }
	
    return true;
}
bool PlannerTask::startHook()
{
    if (! PlannerTaskBase::startHook())
        return false;    

    return true;
}

void PlannerTask::updateHook()
{
    PlannerTaskBase::updateHook();

    // read the current joint angles        
    if(_joints_status.readNewest(joints_status_) == RTT::NoData)
    {
        state(NO_JOINT_STATUS);
        return ;
    }

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
        if (planner_->assignPlanningRequest(joints_status_, debug_check_pose_, planner_status_)) {
            planner_status_.statuscode = motion_planners::PlannerStatus::INVALID;
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

void PlannerTask::updatePlanningscene()
{

    // read octmap data and convert to octree
    if(_octomap_in.readNewest(input_octomap_) == RTT::NewData)
    {
        if(!input_octomap_.data.empty())
        {
            std::cout<<"Updated OCTMAP"<<std::endl;
            input_octree_ = planning_environment::convertContainerToOctomap(input_octomap_);

            if(!initialised_planning_scene_)
            {
                planner_->assignOctomapPlanningScene(input_octree_);
                initialised_planning_scene_ = true;
            }

            planner_->updateOctomap(input_octree_);
        }
    }
}

void PlannerTask::replan(base::JointsTrajectory &input_trajectory)
{
    updatePlanningscene();

    state(GOAL_RECEIVED);

    if(planner_->usePredictedTrajectory(input_trajectory, planner_status_))
        solve();

    setPlannerStatus(planner_status_);

    if(planner_status_.statuscode != PlannerStatus::PLANNING_REQUEST_SUCCESS)
        writeCollisionObjectNames(); 

}

void PlannerTask::writeCollisionObjectNames()
{
    collision_information_ = planner_->getCollisionObjectNames();
    _collision_information.write(collision_information_);	        
}

void PlannerTask::solve()
{
    state(PLANNING);

    solving_time_ = 0.0;
    if(planner_->solve(solution_, planner_status_, solving_time_))
        _planned_trajectory.write(solution_);    

    _solving_time.write(solving_time_);    
}

void PlannerTask::setPlannerStatus(motion_planners::PlannerStatus &planner_status)
{
    switch(planner_status.statuscode)
    {
        case motion_planners::PlannerStatus::PATH_FOUND:
            state(PATH_FOUND); break;
        case motion_planners::PlannerStatus::NO_PATH_FOUND:
            state(NO_PATH_FOUND); break;
        case motion_planners::PlannerStatus::START_STATE_IN_COLLISION:
            state(START_STATE_IN_COLLISION); break;
        case motion_planners::PlannerStatus::GOAL_STATE_IN_COLLISION:
            state(GOAL_STATE_IN_COLLISION); break;
        case motion_planners::PlannerStatus::START_JOINTANGLES_NOT_AVAILABLE:
            state(START_JOINTANGLES_NOT_AVAILABLE); break;
        case motion_planners::PlannerStatus::GOAL_JOINTANGLES_NOT_AVAILABLE:
            state(GOAL_JOINTANGLES_NOT_AVAILABLE); break;
        case motion_planners::PlannerStatus::PLANNING_REQUEST_SUCCESS:
            state(PLANNING_REQUEST_SUCCESS); break;
        case motion_planners::PlannerStatus::CONSTRAINED_POSE_NOT_WITHIN_BOUNDS:
            state(CONSTRAINED_POSE_NOT_WITHIN_BOUNDS); break;
        case motion_planners::PlannerStatus::TIMEOUT:
            state(TIMEOUT); break;
        case motion_planners::PlannerStatus::INVALID_START_STATE:
            state(INVALID_START_STATE); break;
        case motion_planners::PlannerStatus::INVALID_GOAL_STATE:
            state(INVALID_GOAL_STATE); break;
        case motion_planners::PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
            state(UNRECOGNIZED_GOAL_TYPE); break;
        case motion_planners::PlannerStatus::APPROXIMATE_SOLUTION:
            state(APPROXIMATE_SOLUTION); break;
        case motion_planners::PlannerStatus::EXACT_SOLUTION:
            state(PATH_FOUND); break;
        case motion_planners::PlannerStatus::ROBOTMODEL_INITIALISATION_FAILED:
            state(ROBOTMODEL_INITIALISATION_FAILED); break;
        case motion_planners::PlannerStatus::PLANNER_INITIALISATION_FAILED:
            state(PLANNER_INITIALISATION_FAILED); break;
        case motion_planners::PlannerStatus::CRASH:
            state(CRASH); break;
        case motion_planners::PlannerStatus::INVALID:
        {
            switch(planner_status.kinematic_status.statuscode)
            {
                case kinematics_library::KinematicsStatus::KDL_TREE_FAILED:
                    state(KDL_TREE_FAILED); break;
                case kinematics_library::KinematicsStatus::KDL_CHAIN_FAILED:
                    state(KDL_CHAIN_FAILED); break;
                case kinematics_library::KinematicsStatus::URDF_FAILED:
                    state(URDF_FAILED); break;
                case kinematics_library::KinematicsStatus::NO_KINEMATIC_SOLVER_FOUND:
                    state(NO_KINEMATIC_SOLVER_FOUND); break;
                case kinematics_library::KinematicsStatus::IK_FOUND:
                    state(IK_FOUND); break;
                case kinematics_library::KinematicsStatus::NO_IK_SOLUTION:
                    state(NO_IK_SOLUTION); break;
                case kinematics_library::KinematicsStatus::NO_FK_SOLUTION:
                    state(NO_FK_SOLUTION); break; 	    
                case kinematics_library::KinematicsStatus::IK_TIMEOUT:
                    state(IK_TIMEOUT); break;        
                case kinematics_library::KinematicsStatus::IK_JOINTLIMITS_VIOLATED:
                    state(IK_JOINTLIMITS_VIOLATED); break;
                case kinematics_library::KinematicsStatus::INVALID_STATE:
                    state(INVALID_KINEMATIC_STATE); break;				    
                default:
                {
                    std::cout<<"unknown Kinematics state"<<planner_status.kinematic_status.statuscode<<std::endl;
                    state(UNKNOWN_STATE);
                    throw new std::runtime_error("This kinematic status is unknown");		    
                    break;
                }
            }
            break;
        }
        default:
        {
            std::cout<<"unknown planner state"<<planner_status.statuscode<<"   "<<motion_planners::PlannerStatus::INVALID<<std::endl;
            state(UNKNOWN_STATE);
            throw new std::runtime_error("This Planner status is unknown");
            break;
        }
    }
}

template<typename T>
void PlannerTask::plan(T input)
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
        writeCollisionObjectNames();    
}

void PlannerTask::errorHook()
{
    PlannerTaskBase::errorHook();
}
void PlannerTask::stopHook()
{
    PlannerTaskBase::stopHook();
}
void PlannerTask::cleanupHook()
{
    PlannerTaskBase::cleanupHook();

}
