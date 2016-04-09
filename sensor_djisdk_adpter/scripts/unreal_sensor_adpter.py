import rospy
from dji_sdk.msg import Acceleration
from geometry_msgs.msg import QuaternionStamped,PointStamped,TwistStamped,PoseStamped
from nav_msgs.msg import Odometry


#use pose and twist

realtime_odometry = Odometry()
odometry_pub = rospy.Publisher("/dji_sdk/odometry",Odometry,queue_size=10)
acc_pub = rospy.Publisher("/dji_sdk/acceleration",Acceleration,queue_size=10)

flag_pose_updated = False
flag_twist_updated = False

def handle_pose(data):
    global  realtime_odometry
    realtime_odometry.header = data.header
    realtime_odometry.pose.pose = data.pose
    global flag_pose_updated
    if not flag_pose_updated:
        flag_pose_updated = True
        print "first update on pose"

def handle_twist(data):
    global realtime_odometry
    realtime_odometry.header = data.header
    realtime_odometry.twist.twist = data.twist
    global flag_twist_updated
    if not flag_twist_updated:
        print "first update on twist"
        flag_twist_updated = True

def update_odom(data):
    global flag_pose_updated,flag_twist_updated
    if flag_pose_updated and flag_twist_updated:
        odometry_pub.publish(realtime_odometry)

def handle_acc(data):
    global acc_pub
    tmp = Acceleration()
    tmp.ax = data.point.x
    tmp.ay = data.point.y
    tmp.az = data.point.z
    acc_pub.publish(tmp)


if __name__ == "__main__":
    rospy.init_node("unreal_sensor_handler")
    rospy.Subscriber("/Actor/pose",PoseStamped,handle_pose)
    rospy.Subscriber("/Actor/twist",TwistStamped,handle_twist)
    rospy.Subscriber("/Actor/accel",PointStamped,handle_acc)
    rospy.Timer(rospy.Duration(0.01),update_odom)
    print "wait for msg"
    rospy.spin()
