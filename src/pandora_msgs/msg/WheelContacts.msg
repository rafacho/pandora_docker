# WheelContacts.msg
# NOTE(ros2-port): gazebo_msgs/ContactState (ROS1) has no direct equivalent; the closest
# analog under Gazebo Sim / ros_gz is ros_gz_interfaces/Contact, whose field layout differs
# (positions[]/normals[]/depths[]/wrenches[] instead of a single contact_position/wrench pair).
# Consumers (wheel_contacts.py, support_polygon.py, static_stability.py) were updated accordingly.

std_msgs/Header header                       #Stamp
ros_gz_interfaces/Contact wheel1    #Contact of wheel1
ros_gz_interfaces/Contact wheel2    #Contact of wheel2
ros_gz_interfaces/Contact wheel3    #Contact of wheel3
ros_gz_interfaces/Contact wheel4    #Contact of wheel4