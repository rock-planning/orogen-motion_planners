#! /usr/bin/env ruby

require 'vizkit'
require 'orocos'
require 'rock/bundle'
#require 'optparse'
#require 'pry'

=begin
hostname = "localhost"

OptionParser.new do |opt|
    opt.on('--host=HOSTNAME', String, "IP address of the host") do |host|
        hostname = host
    end
end.parse!

Orocos::CORBA.name_service.ip = hostname
Bundles.initialize

=end

#Bundles.initialize
Orocos.initialize

#Orocos.initialize

class TaskStruct
    attr_accessor :name
    attr_accessor :task
    attr_accessor :config

    def initialize()
        @config = Array.new
        @config << "default"
    end
end



def getTask(name, taskArray = nil)
    task = nil
    begin
        task = Orocos.name_service.get name
        puts "\n Task #{name} found"
        if(taskArray)
            struct = TaskStruct.new()
            struct.name = name
            struct.task = task
            taskArray << struct
        end
    rescue Exception => e
        puts e.message
        puts "\nCould not find task #{name}."                 
        return nil
    end
    return task
end


class KukaPlannerGui
    attr_accessor :widget

    def initialize
        @widget = Vizkit.load ('planning_viz.ui')


        # initialsing the start pose
        @widget.dSB_pos_x.setValue(0.0)
        @widget.dSB_pos_y.setValue(0.0)
        @widget.dSB_pos_z.setValue(0.0)
        @widget.dSB_rot_x.setValue(0.0)
        @widget.dSB_rot_y.setValue(0.0)
        @widget.dSB_rot_z.setValue(0.0)

    end


    def configure( planner_tasks)

        if (planner_tasks.size == 1) && (planner_tasks[0].name == "manipulatorplanner")
            @kuka_planner = planner_tasks[0].task
            planner_tasks[0].task = @kuka_planner

            # planner - input joint angle
            @kuka_planner_JsWriter = @kuka_planner.target_joints_angle.writer        

            # planner - input target pose
            @kuka_planner_TpWriter = @kuka_planner.target_pose.writer		    

            @kuka_planner.port('state').connect_to do |data|
                updateStatusENUM()
            end
            
            @kuka_planner.port('solving_time').connect_to do |data|
            @widget.pTextEdit_status.appendPlainText('Solved at : '+data.to_s+' secs');
        end


        end

        @joints_command = Types::Base::Commands::Joints.new
        @pose_command = Types::Base::Samples::RigidBodyState.new      

        planner_tasks
    end

    def connect
        @widget.rB_target_joint_space.connect( SIGNAL('clicked()') ) { targetJointSpace }
        @widget.rB_target_cartesian_space.connect( SIGNAL('clicked()') ) { targetCartesianSpace }
    end

    def targetJointSpace
        if(@widget.rB_target_joint_space.isChecked())
                @widget.rB_target_cartesian_space.setChecked(false)
                targetpose_enable(false)
                targetjoint_enable(true)
                @widget.sendButton.disconnect(SIGNAL('clicked()') )
                @widget.sendButton.connect(SIGNAL('clicked()') ) { planJointSpace }
        else
             @widget.rB_target_cartesian_space.setChecked(true)
             targetpose_enable(true)
             targetjoint_enable(false)
             @widget.sendButton.disconnect(SIGNAL('clicked()') )
             @widget.sendButton.connect(SIGNAL('clicked()') ) { planCartesianSpace }
        end
    end

    def targetCartesianSpace
        if(@widget.rB_target_cartesian_space.isChecked())
                @widget.rB_target_joint_space.setChecked(false)
                targetpose_enable(true)
                targetjoint_enable(false)
                @widget.sendButton.disconnect(SIGNAL('clicked()') )
                @widget.sendButton.connect(SIGNAL('clicked()') ) { planCartesianSpace }
        else
                @widget.rB_target_joint_space.setChecked(true)
                targetpose_enable(false)
                targetjoint_enable(true)
                @widget.sendButton.disconnect(SIGNAL('clicked()') )
                @widget.sendButton.connect(SIGNAL('clicked()') ) { planJointSpace }
        end
    end

    def updateStatusENUM () 
        case @kuka_planner.state
        when :PATH_FOUND
            updateStatus ("PATH FOUND")
        when :NO_PATH_FOUND
            updateStatus ("NO PATH FOUND")
		when :NO_IK_SOLUTION
            updateStatus ("No Inverse solution")	
        when :START_STATE_IN_COLLISION
            updateStatus ("START STATE IN COLLISION")
        when :GOAL_STATE_IN_COLLISION
            updateStatus ("GOAL STATE IN COLLISION")
        when :PLANNER_TIMEOUT
            updateStatus ("PLANNER TIMEOUT")            
        when :PLANNING
            updateStatus ("PLANNING")
        end
    end

    def updateStatus (statusinfo)
        @widget.pTextEdit_status.appendPlainText(statusinfo);
    end

    def targetpose_enable(value)
        @widget.dSB_pos_x.setEnabled(value)
        @widget.dSB_pos_y.setEnabled(value)
        @widget.dSB_pos_z.setEnabled(value)
        @widget.dSB_rot_x.setEnabled(value)
        @widget.dSB_rot_y.setEnabled(value)
        @widget.dSB_rot_z.setEnabled(value)
    end

    def targetjoint_enable(value)
        @widget.dSB_jt_1.setEnabled(value)
        @widget.dSB_jt_2.setEnabled(value)
        @widget.dSB_jt_3.setEnabled(value)
        @widget.dSB_jt_4.setEnabled(value)
        @widget.dSB_jt_5.setEnabled(value)
        @widget.dSB_jt_6.setEnabled(value)
    end

    def planJointSpace    
        puts "Joint space planner input received"
        @joints_command.names.clear
        @joints_command.elements.clear

        @joints_command.names =["joint_a1",
                                "joint_a2",
                                "joint_a3",
                                "joint_a4",
                                "joint_a5",
                                "joint_a6",
                                "joint_a7"]
        joint_state1 = Types::Base::JointState.new
        joint_state1.position = @widget.dSB_jt_1.value()* Math::PI / 180.0
        joint_state1.speed = 1.0 #motor RPM

        joint_state2 = Types::Base::JointState.new
        joint_state2.position = @widget.dSB_jt_2.value()* Math::PI / 180.0
        joint_state2.speed = 1.0 #motor RPM


        joint_state3 = Types::Base::JointState.new
        joint_state3.position = @widget.dSB_jt_3.value()* Math::PI / 180.0
        joint_state3.speed = 1.0 #motor RPM


        joint_state4 = Types::Base::JointState.new
        joint_state4.position = @widget.dSB_jt_4.value()* Math::PI / 180.0
        joint_state4.speed = 1.0 #motor RPM


        joint_state5 = Types::Base::JointState.new
        joint_state5.position = @widget.dSB_jt_5.value()* Math::PI / 180.0
        joint_state5.speed = 1.0 #motor RPM


        joint_state6 = Types::Base::JointState.new
        joint_state6.position = @widget.dSB_jt_6.value()* Math::PI / 180.0
        joint_state6.speed = 1.0 #motor RPM

        joint_state7 = Types::Base::JointState.new
        joint_state7.position = @widget.dSB_jt_7.value()* Math::PI / 180.0
        joint_state7.speed = 1.0 #motor RPM

        @joints_command.elements.push(joint_state1)
        @joints_command.elements.push(joint_state2)
        @joints_command.elements.push(joint_state3)
        @joints_command.elements.push(joint_state4)
        @joints_command.elements.push(joint_state5)
        @joints_command.elements.push(joint_state6)
        @joints_command.elements.push(joint_state7)

        @kuka_planner_JsWriter.write @joints_command
    end


    def planCartesianSpace
        puts "Cartesian space planner input received"

        @pose_command.position.x    = @widget.dSB_pos_x.value()
        @pose_command.position.y    = @widget.dSB_pos_y.value()
        @pose_command.position.z    = @widget.dSB_pos_z.value()
        @pose_command.orientation   = Eigen::Quaternion.from_euler(Eigen::Vector3.new(  @widget.dSB_rot_z.value()* Math::PI / 180.0 ,
                                                                                        @widget.dSB_rot_y.value()* Math::PI / 180.0 ,
                                                                                        @widget.dSB_rot_x.value()* Math::PI / 180.0 ),
                                                                                        2,1,0)
        @pose_command.sourceFrame = 'base_link'
        @pose_command.targetFrame = 'link_7'

        @kuka_planner_TpWriter.write @pose_command
    end

    def run
        updateStatus('Planner GUI is ready')		
        @widget.show
        Vizkit.exec		
    end
end



gui = KukaPlannerGui.new
configured = false

gui.widget.connect_to_task "" do |task|

    allTasks = Array.new()
    kuka_planner  = getTask('manipulatorplanner', allTasks)

    if(allTasks.size == 0)
        puts "No task is availalble"
        exit
    end

    gui.configure( allTasks)
    gui.connect

    gui.widget.setEnabled(true)
    configured = true
end
gui.run


