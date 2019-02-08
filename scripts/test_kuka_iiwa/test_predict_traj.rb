require 'orocos'
require 'vizkit'
include Orocos
require 'pry'

Orocos.initialize


#def reverse_trajectory(input_traj, output_traj)
#
#    num_of_elements     = input_traj.elements[0].size
#    num_of_joints       = input_traj.elements.size
#    
#    out
#    for i in 0..num_of_joints-1
#        for i in num_of_elements-1.downto(0) do
#            input_traj.elements[
#        time_step=[]
#        for j in 0..num_of_elements-1
#            s=Types::Base::JointState.new
#            s.position      = new_traj_position[i][j]
#            s.speed         = 0.00 #Float::NAN
#            time_step.concat([s])
#        end
#        out_traj.elements.concat([time_step])
#    end
#
#end


def makeInverseTrajectory(trajectory)
    inverse_trajectory = Types::Base::JointsTrajectory.new
    inverse_trajectory.names = trajectory.names

    traj_size = trajectory.elements[0].size
    n_joints = trajectory.elements.size
    (0..n_joints-1).each do |i|
        inv_joint_traj = Types::Base::JointTrajectory.new
        (0..traj_size-1).each do |j|
            inv_joint_traj << trajectory.elements[i][traj_size-j-1]
        end
        inverse_trajectory.elements << inv_joint_traj
    end
    return inverse_trajectory
end



#kuka_planner	= Orocos.name_service.get "manipulatorplanner"
kuka_planner	= Orocos.name_service.get "flexipick_motion_planner"
    
predicted_trajectory = Types.base.JointsTrajectory.new
trajectory_data = Types.base.JointsTrajectory.new

kuka_planner.planned_trajectory.connect_to do |data|
    trajectory_data = data
    puts "Got inout trajectory of size #{trajectory_data.elements[0].size()}"
    predicted_trajectory = makeInverseTrajectory(trajectory_data)
    binding.pry
    kuka_planner.predicted_trajectory.write predicted_trajectory
end

    Vizkit.exec

