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

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PlannerTask.hpp for more detailed
// documentation about them.

bool PlannerTask::configureHook()
{
    if (! PlannerTaskBase::configureHook())
        return false;

    config_ = _config.value();
    input_ptcloud.points.clear();
    
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
    
    planner_->assignPlanningScene(Eigen::Vector3d::Zero());
    
    return true;
}

void PlannerTask::updateHook()
{
    PlannerTaskBase::updateHook();
    
    // read the current joint angles        
    if(_joints_status.readNewest(joints_status_) == RTT::NewData)
    {	
	//state(RUNNING);    
    }
    else
    {
	state(NO_JOINT_STATUS);
	return ;
    }
        
    // read pointcloud cloud
    if(_environment_in.readNewest(input_ptcloud) == RTT::NewData)
    {        
        if(!input_ptcloud.points.empty())
        {
	    planner_->updatePointcloud(input_ptcloud, Eigen::Vector3d::Zero());
	}
	
	// debug
	if(_environment_out.connected())
	{
	    planner_->getSelfFilteredPointcloud(debug_ptcloud);
	    _environment_out.write(debug_ptcloud);
	}    
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
        plan(target_pose_);
    }
}

void PlannerTask::plan(base::samples::RigidBodyState &target_pose)
{
    state(GOAL_RECEIVED);

    //planned_trajectory.clear();    
   
    if(planner_->assignPlanningRequest(joints_status_, target_pose, config_.planner_config.robot_model_config.planning_group_name, planner_status_))
	solve();
    
    setPlannerStatus(planner_status_);
}

void PlannerTask::plan(base::commands::Joints &target_joints_angle)
{
    state(GOAL_RECEIVED);

    //planned_trajectory.clear();    
   
    if(planner_->assignPlanningRequest(joints_status_, target_joints_angle, config_.planner_config.robot_model_config.planning_group_name, planner_status_))    
	solve();
    
    setPlannerStatus(planner_status_);
    
    if(planner_status_.statuscode != PlannerStatus::PLANNING_REQUEST_SUCCESS)
    {
	collision_information_.collision_pair_names = planner_->getCollisionObjectNames();
	_collision_information.write(collision_information_);	    
    }
}

void PlannerTask::solve()
{    
    state(PLANNING);

    solving_time_ = 0.0;
    if(planner_->solve(solution_, planner_status_, solving_time_))
	_planned_trajectory.write(solution_);
    
    setPlannerStatus(planner_status_);
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

		case kinematics_library::KinematicsStatus::KDL_CHAIN_FAILED:
		    state(KDL_CHAIN_FAILED); break;
		case kinematics_library::KinematicsStatus::KDL_INITIALISATION_FAILED:
		    state(KDL_INITIALISATION_FAILED); break;
		case kinematics_library::KinematicsStatus::NO_KINEMATIC_SOLVER_FOUND:
		    state(NO_KINEMATIC_SOLVER_FOUND); break;
		case kinematics_library::KinematicsStatus::NO_IK_SOLUTION:
		    state(NO_IK_SOLUTION);break;
    		case kinematics_library::KinematicsStatus::NO_FK_SOLUTION:
		    state(NO_FK_SOLUTION);break;
    		case kinematics_library::KinematicsStatus::IK_TIMEOUT:
		    state(IK_TIMEOUT);break;
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
