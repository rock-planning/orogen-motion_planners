require 'orocos'
require 'vizkit'
require 'rock/bundle'
require 'pry'


include Orocos
Orocos.load_typekit 'base'


Bundles.initialize

kuka_viz              = Vizkit.default_loader.RobotVisualization
noisy_traj_plugin       = Vizkit.default_loader.TrajectoryVisualization
noiselsess_traj_plugin  = Vizkit.default_loader.TrajectoryVisualization
kuka_viz.modelFile 	= './data/kuka_iiwa.urdf'


Bundles.run 'kinematics_library::Task' => 'kuka_kinematics', 'output' => nil  do

    ## get the service handle
    kuka_kinematic   = Orocos.name_service.get "kuka_kinematics"
    traj_viewer         = Orocos.name_service.get "traj_viewer"

    Orocos.apply_conf_file(kuka_kinematic, './config/KinematicTask.yml', ['default'])

#    kuka_kinematic.port("pose_value").connect_to do |rbs,_|
#        noisy_traj_plugin.updateTrajectory(rbs.position)
#    end

    kuka_kinematic.port("pose_value").connect_to traj_viewer.trajectory_pose

    kuka_kinematic.configure
    kuka_kinematic.start

    #read data
    num_rollout_file = File.readlines('./debug_data/num_rollouts.txt')
    num_rollout      = num_rollout_file.size

            

    Vizkit.exec
    kuka_kinematic.stop

end

