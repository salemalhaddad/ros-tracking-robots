/usr/bin/env python3
import rospy
from std_msgs.msg import String
import requests
import time
from turtlesim.msg import Pose


# Configuration
# robot_name = "my_robot"
server_address = "http://13.49.49.209:5000/"
api_delay = 1  # Delay in seconds

def send_post_request(x, y, robot_name):
    """Sends a POST request to the server with the robot's coordinates."""
    url = f"{server_address}/{robot_name}/{x}/{y}"
    try:
        response = requests.post(url)
        if response.status_code == 200:
            rospy.loginfo(f"Data sent successfully, robot: {robot_name}, x = {x}, y = {y}")
        else:
            rospy.logwarn(f"Failed to send data, status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        rospy.logerr(f"Request failed: {e}")

def robot_callback(data, robot_name):
    """Callback function for processing data."""
    x = data.x
    y = data.y

    send_post_request(x, y, robot_name)

def robot_listener():
    rospy.init_node('robot_tracking_node', anonymous=True)

    # Get parameters
    turtle_number = rospy.get_param('~turtle_number', 1)
    turtle_topic = f"/turtle{turtle_number}/pose"
    robot_name = rospy.get_param('~robot_name', f'my_turtle_{turtle_number}')

    # Subscribe to the dynamic topic with lambda function as a wrapper
    rospy.Subscriber(turtle_topic, Pose, lambda data: robot_callback(data, robot_name))

    rate = rospy.Rate(1/api_delay)
    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == '__main__':
    try:
        robot_listener()
    except rospy.ROSInterruptException:
        pass
