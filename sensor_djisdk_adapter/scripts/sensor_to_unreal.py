#!/usr/bin/python
import rospy
from dji_sdk.msg import Acceleration
from geometry_msgs.msg import QuaternionStamped,PointStamped,TwistStamped,PoseStamped,Pose
from nav_msgs.msg import Odometry


#use pose and twist

pose_pub = rospy.Publisher("/unreal/pose",Pose,queue_size=10)
pose = Pose()
def handle_odom(data):
    global pose
    pose = data.pose.pose
    pose.position.x = pose.position.x + 0.012;
    pose.position.y = pose.position.y + 0.011;
    pose.position.z = pose.position.z + 0.013;

    pose.orientation.w += 0.0001;
    pose.orientation.x += 0.0001;
    pose.orientation.y += 0.0001;
    pose.orientation.z += 0.0001;

def update_pose(e):
    global pose,pose_pub
    pose_pub.publish(pose)

if __name__ == "__main__":
    rospy.init_node("unreal_sensor_handler")
    rospy.Subscriber("/dji_sdk/odometry",Odometry,handle_odom)
    rospy.Timer(rospy.Duration(0.02),update_pose)
    rospy.loginfo("VICON ADAPTER READY")
    rospy.loginfo("Waiting for message come")
    rospy.spin()
