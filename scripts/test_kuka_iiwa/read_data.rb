require 'orocos'
require 'vizkit'
require 'rock/bundle'
require 'readline'
require 'pry'


include Orocos
Orocos.load_typekit 'base'


Orocos.initialize

@joints = Types.base.samples.Joints.new
@joints.names = ["joint_a1",
                 "joint_a2",
                 "joint_a3",
                 "joint_a4",
                 "joint_a5",
                 "joint_a6",
                 "joint_a7"]



def visualize_rollout(kuka_kinematic, traj_viewer, trajectory_data)
       
    joint_state = Types.base.JointState.new
    joint_state.speed = 0.5

    for i in 0..trajectory_data.size-1
        data_string = trajectory_data[i]
        data = data_string.split(/[\t,\n]/)
        @joints.elements.clear
        for j in 0 .. data.size
            joint_state.position = data[j].to_f
            @joints.elements.push(joint_state)
        end    
        kuka_kinematic_jt_status_port = kuka_kinematic.port('joints_status')
        kuka_kinematic.joints_status.write(@joints) 
        sleep(0.05)
    end
end


Bundles.run 'output' => nil  do

    ## get the service handle
    kuka_kinematic   = Orocos.name_service.get "kuka_kinematics"
    traj_viewer         = Orocos.name_service.get "traj_viewer"

    #read data
    num_rollout_file = File.readlines('./debug_data/num_rollouts.txt')
    num_rollout      = num_rollout_file.size


    for i in 1..num_rollout
        kuka_kinematic.pose_value.disconnect_from traj_viewer.trajectory_pose
        sleep(1)
        kuka_kinematic.pose_value.connect_to traj_viewer.trajectory_rollout_pose
        sleep(1)
        for j in 0..((num_rollout_file[i-1].to_i)-1)
            file ='./debug_data/noisy_'+i.to_s+'_'+j.to_s+'.txt' 
            #file ='./debug_data/noiseless_'+i.to_s+'.txt' 
            puts "#{file}"
            noisy_data_file = File.readlines(file)
            # visualize rollout
            visualize_rollout(kuka_kinematic, traj_viewer, noisy_data_file)
        #    Readline.readline"Press Enter" 
            traj_viewer.addRollouts
         #   Readline.readline"Press Enter" 
        end
        sleep(2)
        kuka_kinematic.pose_value.disconnect_from traj_viewer.trajectory_rollout_pose
        sleep(1)
        kuka_kinematic.pose_value.connect_to traj_viewer.trajectory_pose
        #traj_viewer.reset
        file ='./debug_data/noiseless_'+i.to_s+'.txt' 
        puts "#{file}"
        noisy_data_file = File.readlines(file)
        visualize_rollout(kuka_kinematic, traj_viewer, noisy_data_file)
        sleep(2)
        traj_viewer.reset
            
    end

    Vizkit.exec
    kuka_kinematic.stop

end

