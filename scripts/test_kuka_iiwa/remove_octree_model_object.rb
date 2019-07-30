require 'orocos'
require 'vizkit'
include Orocos
require 'pry'

Orocos.initialize
Orocos.load_typekit 'motion_planners'

kuka_planner	= Orocos.name_service.get "flexipick_motion_planner"

# create a box 
box = Types.collision_detection.PrimitiveObject.new()
#box.dimensions = Types.base.Position.new(0.06, 0.06, 0.415)
box.dimensions = Types.base.Position.new(0.06, 0.06, 0.415)
# position = Types.base.Position.new(0,0,0)
position = Types.base.Position.new(-0.06,0.0,1.1)
orientation = Types::Base::Quaterniond.new(1.0, 0.0, 0.0, 0.0)
pose = Types.base.Pose.new()
pose.position = position
pose.orientation = orientation

#fill the model object
model_object = Types.motion_planners.ModelObject.new()
model_object.operation = :REMOVE
model_object.model_type = :OCTREE
model_object.primitive_object = box
model_object.object_name = "obstacle1"
model_object.attach_link_name = "ur_table_base"
model_object.relative_pose = pose 

kuka_planner.known_object.write(model_object)

sleep(1)



