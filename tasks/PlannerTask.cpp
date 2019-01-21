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
    input_ptcloud.points.clear();
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
}

void PlannerTask::updatePlanningscene()
{
    // read pointcloud cloud
    if(_environment_in.readNewest(input_ptcloud) == RTT::NewData)
    {
        if(!initialised_planning_scene_)
        {
            planner_->assignPointcloudPlanningScene(Eigen::Vector3d::Zero());
            initialised_planning_scene_ = true;
        }
        
        if(!input_ptcloud.points.empty())
            planner_->updatePointcloud(input_ptcloud, Eigen::Vector3d::Zero());

        // debug
        if(_environment_out.connected())
        {
            planner_->getSelfFilteredPointcloud(debug_ptcloud);
            _environment_out.write(debug_ptcloud);
        }
    }
    
    // read octmap data and convert to octree
    if(_octomap_in.readNewest(input_octomap_) == RTT::NewData)
    {
        
        if(!input_octomap_.data.empty())
        {
       
            input_octree_ = planning_environment::convertContainerToOctomap(input_octomap_);
            
            if(!initialised_planning_scene_)
            {
                planner_->assignOctomapPlanningScene(input_octree_, Eigen::Vector3d::Zero());
                initialised_planning_scene_ = true;
            }
                        
            planner_->updateOctomap(input_octree_, Eigen::Vector3d::Zero());
        }        
    }
   
}

void PlannerTask::plan(base::samples::RigidBodyState &target_pose)
{
    updatePlanningscene();

    state(GOAL_RECEIVED);

    //planned_trajectory.clear();

    if(planner_->assignPlanningRequest(joints_status_, target_pose, config_.planner_config.robot_model_config.planning_group_name, planner_status_))
        solve();

    setPlannerStatus(planner_status_);

    if(planner_status_.statuscode != PlannerStatus::PLANNING_REQUEST_SUCCESS)
        writeCollisionObjectNames();

}

void PlannerTask::plan(base::commands::Joints &target_joints_angle)
{
    updatePlanningscene();

    state(GOAL_RECEIVED);

    //planned_trajectory.clear();    

    if(planner_->assignPlanningRequest(joints_status_, target_joints_angle, config_.planner_config.robot_model_config.planning_group_name, planner_status_))    
        solve();

    setPlannerStatus(planner_status_);

    if(planner_status_.statuscode != PlannerStatus::PLANNING_REQUEST_SUCCESS)
        writeCollisionObjectNames();	

}

void PlannerTask::writeCollisionObjectNames()
{
    collision_information_.collision_pair_names = planner_->getCollisionObjectNames();
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
