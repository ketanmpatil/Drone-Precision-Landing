import rospy
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import Point

rospy.init_node('position_target')

# Create a publisher to send position target messages
target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

# Create a position target message
target_msg = PositionTarget()

# Set the position target message fields
target_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
target_msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.FORCE | PositionTarget.IGNORE_YAW_RATE
target_msg.position.x = 10.0
target_msg.position.y = 0.0
target_msg.position.z = 0.0

# Publish the position target message
rate = rospy.Rate(10) # 10 Hz
while not rospy.is_shutdown():
    target_pub.publish(target_msg)
    rate.sleep()