require 'orocos'
require 'vizkit'
include Orocos
require 'pry'

Orocos.initialize
#Orocos::CORBA.name_service.ip = "127.0.0.1"
robot_viz = Vizkit.default_loader.RobotVisualization
robot_viz.modelFile = './data/kuka_iiwa.urdf'

 
Orocos.run  'motion_planners::PlannerTask'=>'manipulatorplanner',
            'joint_control::FakeJointDriverTask' => 'plant', :output => nil do


    kuka_planner	= Orocos.name_service.get "manipulatorplanner"
    fake_robot      = Orocos.name_service.get "plant"

	Orocos.apply_conf_file( kuka_planner,'./config/MotionPlanner.yml', ['kuka'] )    
	Orocos.apply_conf_file( fake_robot,'./config/plant.yml', ['default','kuka_arm'], true)
   

    kuka_planner.configure
    fake_robot.configure

    fake_robot.joint_state.connect_to kuka_planner.joints_status

    fake_robot.start   
    sleep 2.0
    kuka_planner.start

    fake_robotWriter = fake_robot.command.writer
    fake_robot_cmd = fake_robotWriter.new_sample

    timer = Qt::Timer.new
    trajectory_data = Types.base.JointsTrajectory.new
    counter = 0

    kuka_planner.planned_trajectory.connect_to do |data|
        trajectory_data = data
		puts "Got inout trajectory of size #{trajectory_data.elements[0].size()}"
        timer.start(100.0)
    end


    timer.connect(SIGNAL('timeout()')) do
        joints = Types.base.samples::Joints.new

        joints.names = ["joint_a1",
                        "joint_a2",
                        "joint_a3",
                        "joint_a4",
                        "joint_a5",
                        "joint_a6",
                        "joint_a7"]

        joint_state1 = Types.base.JointState.new
        joint_state1.position = trajectory_data.elements[0][counter].position
        joint_state1.speed = 0.1

        joint_state2 = Types.base::JointState.new
        joint_state2.position = trajectory_data.elements[1][counter].position
        joint_state2.speed = 0.1

        joint_state3 = Types.base::JointState.new
        joint_state3.position = trajectory_data.elements[2][counter].position
        joint_state3.speed = 0.1

        joint_state4 = Types.base::JointState.new
        joint_state4.position = trajectory_data.elements[3][counter].position
        joint_state4.speed = 0.1

        joint_state5 = Types.base::JointState.new
        joint_state5.position = trajectory_data.elements[4][counter].position
        joint_state5.speed = 0.1

        joint_state6 = Types.base::JointState.new
        joint_state6.position = trajectory_data.elements[5][counter].position
        joint_state6.speed = 0.1

        joint_state7 = Types.base::JointState.new
        joint_state7.position = trajectory_data.elements[6][counter].position
        joint_state7.speed = 0.1

        joints.elements.push(joint_state1)
        joints.elements.push(joint_state2)
        joints.elements.push(joint_state3)
        joints.elements.push(joint_state4)
        joints.elements.push(joint_state5)
        joints.elements.push(joint_state6)
        joints.elements.push(joint_state7)

        robot_viz.updateData(joints)
        fake_robotWriter.write(joints)
        joints.elements.clear()

        counter = counter +1
        if(counter >= trajectory_data.elements[0].size())
            counter = 0
            timer.stop()
        end
    end


    Vizkit.exec
    fake_robot.stop
    kuka_planner.stop

    Orocos.watch(kuka_planner, fake_robot)   
end
