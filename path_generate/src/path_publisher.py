import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

midpoints = [[0, 0, 0], [1, 0, 0], [2, 0.5, 0],[3, 1, 0], [4, 1.5, 0 ],[5,2.5, 0], [0,1,1],[1,1,1],[2,1.5,1],[3,2,1],[4,2.5,1],[5,8,1]]

rospy.init_node('path_publisher')
path_pub = rospy.Publisher('my_path', Path, queue_size=10)

rate = rospy.Rate(10) # 10 Hz

while not rospy.is_shutdown():
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "my_frame"

    for point in midpoints:
        pose = PoseStamped()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1

        path.poses.append(pose)

    path_pub.publish(path)
    rate.sleep()