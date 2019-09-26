import math
import fetch_api
import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import Image, CameraInfo
from moveit_python import PlanningSceneInterface
from moveit_msgs.msg import OrientationConstraint
import sys
import graspit_commander
from math import pi
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from A3C_Interface import *
import tensorflow
import tf
import time

bridge = CvBridge()

def cvt2angle(euler):
    if euler < 0:
        return euler / pi * 180 + 360
    else:
        return euler / pi * 180

def cvt2euler(angle):
    if angle > 180:
        return ((angle - 360) / 180) * pi
    else:
        return (angle / 180) * pi

def getState(target_name, relative_entity_name='base_link'):
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    resp_coordinates = model_coordinates(target_name, relative_entity_name)

    rx, ry, rz, rw = resp_coordinates.pose.orientation.x, \
                    resp_coordinates.pose.orientation.y, \
                    resp_coordinates.pose.orientation.z, \
                    resp_coordinates.pose.orientation.w
    ex, ey, ez = tf.transformations.euler_from_quaternion([rx, ry, rz, rw])
    print('Pose of {} is x: {} y: {} z: {} rx: {}'.format(target_name, 
                                                  resp_coordinates.pose.position.x,
                                                  resp_coordinates.pose.position.y,
                                                  resp_coordinates.pose.position.z,
                                                  -ex
                                                  ))

    return resp_coordinates.pose.position.x, \
           resp_coordinates.pose.position.y, \
           resp_coordinates.pose.position.z, \
           -ex

def get_observation():
    '''
    alternatively, you can use
    depth_array = np.fromstring(image_msg.data, np.float32)
    but this will only produce 1d array
    '''
    c1_x, c1_y, c1_z, c1_rx = getState('cup1')
    c2_x, c2_y, c2_z, c2_rx = getState('cup2')

    image_msg = rospy.wait_for_message('/head_camera/depth_registered/image_raw', Image)
    depth_image = bridge.imgmsg_to_cv2(image_msg, '32FC1')
    depth_array = np.array(depth_image, dtype=np.float32)
    shape = depth_array.shape
    center_dist = depth_array[347, 337]
    print('center is: ' + str(center_dist))

    # return np.array([c1_x - c2_x, c1_y - c2_y, c1_z - c2_z, c1_rx, (center_dist - 0.66) / 0.2])
    # ros->unity, x->z, y->x, z->y
    return np.array([(c1_y - c2_y) * 10, 
                     (c1_z - c2_z) * 10, 
                     cvt2angle(c1_rx) / 180.0])

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def execute_action(act, arm):
    x_pose = 0.329202820349 - 0.5
    y_pose = -0.01
    z_pose = 0.06
    c1_x, c1_y, c1_z, c1_rx = getState('cup1')
    origin_angle = cvt2angle(c1_rx)
    cvt_angle = cvt2euler(origin_angle + np.clip(act[-1], 0, 1) * 10)

    print('Cup original angle: {}, converted angle: {}'.format(origin_angle, cvt_angle))

    x, y, z, w = cvt_e2q(cvt_angle, 0, 0)
    move_arm_to(arm, 
            c1_x + x_pose,
            np.clip(act[0], -0.1, 0.05) / 20 + c1_y + y_pose,
            min(np.clip(act[1], -0.1, 0.05) / 20 + c1_z + z_pose, 0.93),
            x, y, z, w)


def move_arm_to(arm, x, y, z, ox, oy, oz, ow, relative_frame='base_link'):
    rpose = PoseStamped()
    rpose.header.frame_id = relative_frame
    rpose.pose.position.x = x
    rpose.pose.position.y = y
    rpose.pose.position.z = z
    rpose.pose.orientation.x = ox
    rpose.pose.orientation.y = oy
    rpose.pose.orientation.z = oz
    rpose.pose.orientation.w = ow
    arm.move_to_pose(rpose, replan=True, execution_timeout=6)

def cvt_e2q(x, y, z):
    quad = Quaternion(*tf.transformations.quaternion_from_euler(x, y, z))
    return quad.x, quad.y, quad.z, quad.w

def main():
    rospy.init_node('pour_scene')
    wait_for_time()
    target_name = 'cup1'

    x_pose = 0.329202820349
    y_pose = -0.01
    z_pose = 0.060
    x_ori, y_ori, z_ori, w_ori = cvt_e2q(0, 0, 0)


    x,y,z, _ = getState('cup2')
    head = fetch_api.Head()
    arm = fetch_api.Arm()
    torso = fetch_api.Torso()

    torso.set_height(fetch_api.Torso.MAX_HEIGHT)
    head.look_at('base_link', x, y, z)

    sess = tensorflow.Session()
    model = load_model(sess)

    x, y, z, _ = getState(target_name)

    # observation test
    get_observation()
    
    move_arm_to(arm, x_pose, y + y_pose, z + z_pose, x_ori, y_ori, z_ori, w_ori)

    x, y, z, _ = getState('table1')
    base = fetch_api.Base()
    if x > 1:
        base.go_forward(0.6, speed=0.2)

    # collision in motion planning
    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.clear()

    length_table = 1
    width_table = 1
    height_table = 0.04
    x_table, y_table, z_table, _ = getState('table1')
    z_table = z + 0.7
    planning_scene.addBox('table', length_table, width_table, height_table,
                         x_table, y_table, z_table)
    
    length_box = 0.05
    width_box = 0.05
    height_box = 0.2
    x,y,z, _ = getState('cup1')
    x_box = x
    y_box = y
    z_box = z
    planning_scene.addBox('mug_1', length_box, width_box, height_box, x_box, y_box, z_box)


    length_box = 0.03
    width_box = 0.05
    height_box = 0.2
    x, y, z, _ = getState('cup2')
    x_box = x
    y_box = y
    z_box = z
    planning_scene.addBox('mug_2', length_box, width_box, height_box, x_box, y_box, z_box)

    # the initial position of gripper is (-0.5, 0, 0), and
    # the final position of gripper is (-0.5+x_pose, y_pose, z_pose).
    x, y, z, _ = getState(target_name)
    move_arm_to(arm, x - 0.5 + x_pose, y + y_pose, z + z_pose, 
            x_ori, y_ori, z_ori, w_ori)

    planning_scene.removeCollisionObject('mug_1')
    # planning_scene.removeCollisionObject('mug_2')
    # planning_scene.removeCollisionObject('mug')
    # planning_scene.removeCollisionObject('table')

    gripper = fetch_api.Gripper()
    effort = gripper.MAX_EFFORT
    gripper.close(effort)

    x, y, z, _ = getState(target_name)
    move_arm_to(arm, x - 0.5 + x_pose, y + y_pose, z + z_pose + 0.06, x_ori, y_ori, z_ori, w_ori)
    x, y, z, _ = getState('cup2')
    head.look_at('base_link', x, y, z)

    for _ in xrange(50):
        obs = get_observation()
        act = model.choose_action(obs)
        print('obs: {}, action: {}'.format(obs, act))
        execute_action(act, arm)
        time.sleep(0.5)

    
if __name__ == '__main__':
    main()
