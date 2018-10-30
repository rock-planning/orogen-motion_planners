require 'orocos'
require 'vizkit'
include Orocos
require 'pry'

Orocos.initialize
Orocos.load_typekit 'motion_planners'

kuka_planner	= Orocos.name_service.get "manipulatorplanner"

# create a box 
box = Types.collision_detection.PrimitiveObject.new()
box.dimensions = [0.2, 0.2, 0.8] 

position = Types.base.Position.new(0,0,0)
orientation = Types::Base::Quaterniond.new(1.0, 0.0, 0.0, 0.0)
pose = Types.base.Pose.new()
pose.position = position
pose.orientation = orientation

#fill the model object
model_object = Types.motion_planners.ModelObject.new()
model_object.operation = :ADD 
model_object.model_type = :PRIMITIVES
model_object.primitive_object = box
box.dimensions = [0.2, 0.2, 0.8] 
model_object.object_name = "obstacle_1"
model_object.attach_link_name = "base_link"
model_object.relative_pose = pose 

#binding.pry
kuka_planner.known_object.write(model_object)

sleep(1)



