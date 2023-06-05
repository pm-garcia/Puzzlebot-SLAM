#!/usr/bin/env python
import numpy as np
import numpy as np
import rospy, tf
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped, TwistWithCovariance, TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from fiducial_msgs.msg import FiducialTransformArray
import tf.transformations as tftr
import tf2_ros
import tf2_geometry_msgs

wl, wr = 0.0, 0.0
r, l = 0.05, 0.191
fiducials = None

def wl_callback(msg):
    global wl
    wl = msg.data

def wr_callback(msg):
    global wr
    wr = msg.data

def arucos_callback(fiducial_array):
    global fiducials
    fiducials = fiducial_array.transforms

def odometry(miu, u, sigma, cTime):
    x, y, th = np.squeeze(miu)
    v, w = np.squeeze(u)
    qRota = tf.transformations.quaternion_from_euler(0, 0, th)

    pose_cov = np.zeros((6, 6))
    pose_cov[0][0] = sigma[0][0]
    pose_cov[1][0] = sigma[1][0]
    pose_cov[1][1] = sigma[1][1]
    pose_cov[0][5] = sigma[0][2]
    pose_cov[0][1] = sigma[0][1]
    pose_cov[1][5] = sigma[1][2]
    pose_cov[5][0] = sigma[2][0]
    pose_cov[5][1] = sigma[2][1]
    pose_cov[5][5] = sigma[2][2]

    odom = Odometry()
    odom.header.stamp = cTime
    odom.header.frame_id = "odom"
    odom.child_frame_id = "odom/base_link"
    odom.pose.pose.position = Point(x, y, r)
    odom.pose.pose.orientation = Quaternion(qRota[0], qRota[1], qRota[2], qRota[3])
    odom.twist.twist.linear.x = v
    odom.twist.twist.angular.z = w
    odom.pose.covariance = pose_cov.flatten().tolist()

    return odom

def pose_stamped(odom, cTime):
    robot_pose = PoseWithCovarianceStamped()
    robot_pose.header.stamp = cTime
    robot_pose.header.frame_id = "map"
    robot_pose.pose.pose.position = odom.pose.pose.position
    robot_pose.pose.pose.orientation = odom.pose.pose.orientation
    robot_pose.pose.covariance = odom.pose.covariance

    return robot_pose

# def map_robot_transformation(miu):
#     x, y, th = np.squeeze(miu)
#     rotation_matrix_z = np.array([[np.cos(q), -np.sin(q), 0], 
#                                   [np.sin(q), np.cos(q), 0], 
#                                   [0, 0, 1]])
#     robot_map_tf = np.eye(4)
#     robot_map_tf[:3, :3] = rotation_matrix_z
#     robot_map_tf[:3, -1] = np.array([x, y, 0]).T
#     map_robot_tf = np.linalg.inv(robot_map_tf)
#     return map_robot_tf

# def camera_robot_transformation():
#     th = np.pi/2

#     camera_robot_tf = np.eye(4)
#     rotation_matrix_z = np.array([[np.cos(-th), -np.sin(-th), 0], 
#                                   [np.sin(-th), np.cos(-th), 0], 
#                                   [0, 0, 1]])
#     rotation_matrix_y = np.array([[np.cos(th), 0, np.sin(th)], 
#                                   [0, 1, 0], 
#                                   [-np.sin(th), 0, np.cos(th)]])
#     rotation_matrix = np.dot(rotation_matrix_y, rotation_matrix_z)
#     camera_robot_tf[:3, :3] = rotation_matrix

#     return camera_robot_tf

# # TODO: Modificar esto
# def create_dp(aruco_on_map, robot_on_map):
#     dp = aruco_on_map - robot_on_map
#     dp[-1] = np.linalg.norm(dp[:2])
#     return dp

def kalman(M, miu, sigma, u, fiducials, Q, R, dt, cTime):
    
    miu_pred = motion_model(miu, u, dt)
    # miu_antigua = motion_model(miu, u, dt)
    H = motion_model_jacobian(miu, u, dt)
    Q = motion_Q(miu, dt)
    sigma_pred = np.dot(np.dot(H, sigma), H.T) + Q
    # print("Miu antigua", miu_pred)
    # return miu_pred, sigma_pred
    odom = odometry(miu_pred, u, sigma_pred, cTime)
    robot_pose = pose_stamped(odom, cTime)

    # map_robot_tf = map_robot_transformation(miu)

    if fiducials is not None:
        # print("Elementos en fiducials", len(fiducials))
        # x, y, th = np.squeeze(miu_pred)
        # robot_on_map = robot_pose_map = np.array([[x], [y], [0]])

        for fiducial in fiducials:
            marker_id = fiducial.fiducial_id
            translation = fiducial.transform.translation
            rotation = fiducial.transform.rotation
            distance = np.sqrt((translation.x)**2 + (translation.y)**2 + (translation.z)**2)
            # print("Translation: ", translation)
            # print("rotation: ", rotation)

            if marker_id in M and distance < 1.8:
                m = np.array([[M[marker_id][0]], [M[marker_id][1]]])

                z_i = observation_lectures(translation, rotation)
                z_pred = observation_model(m, miu_pred)

                G = observation_model_jacobian(m, miu_pred)
                Z = np.dot(np.dot(G, sigma_pred), G.T) + R
                K = np.dot(np.dot(sigma_pred, G.T), np.linalg.inv(Z))
                # print(K.shape)
                # print("z_i_lectura", z_i)
                # print("z_pred_calculada", z_pred)
                # print("Miu_antigua", miu_pred)

                # * Create the dp vector for the real aruco
                # true_aruco = np.array(true_aruco)
                # aruco_dp = self.create_dp(true_aruco, robot_pose_map)

                # # * Transform the aruco pose to the map frame
                # aruco_pose = np.array([[fiducial.transform.translation.x],
                #                        [fiducial.transform.translation.y],
                #                        [fiducial.transform.translation.z],
                #                        [1]])
                # camera_robot_tf = camera_robot_transformation()
                # aruco_on_robot = np.dot(camera_robot_tf, aruco_pose)
                # aruco_on_map = np.dot(map_robot_tf, aruco_on_robot)

                # # * Create the dp vector for the seen aruco
                # dp = self.create_dp(aruco_m[:3], robot_pose_map)

                # # * Calculate the Kalman gain
                # Gk = self.get_Gk(dp)
                # Zk = np.matmul(Gk, np.matmul(sigma, Gk.T)) + self.Rk
                # Kk = np.matmul(sigma, np.matmul(Gk.T, np.linalg.inv(Zk)))

                # # * Calculate the innovation
                # z = self.get_z(dp, aruco_r)
                # true_aruco_homogenous = true_aruco
                # true_aruco_homogenous = np.append(true_aruco_homogenous, 1)
                # aruco_r = np.matmul(map_robot_transformation, true_aruco_homogenous)
                # z_hat = self.get_z(aruco_dp, aruco_r)

                error = z_i - z_pred
                error[1][0] = np.arctan2(np.sin(error[1][0]), np.cos(error[1][0]))
                # print("Error", error)

                miu_pred += np.dot(K, (error))
                # print("Miu_corregida", miu_pred)
                I = np.eye(len(miu_pred))
                sigma_pred = np.dot((I - np.dot(K, G)), sigma_pred)

        fiducials = None

    return miu_pred, sigma_pred, robot_pose, odom

def motion_model(miu, u, dt):
    x, y, th = np.squeeze(miu)
    v, w = np.squeeze(u)
    
    x_new = x + dt * v * np.cos(th) 
    y_new = y + dt * v * np.sin(th) 
    th_new = th + dt * w 

    th_new = np.arctan2(np.sin(th_new), np.cos(th_new))

    return np.array([[x_new], [y_new], [th_new]])

def motion_model_jacobian(miu, u, dt):
    x, y, th = np.squeeze(miu)
    v, w = np.squeeze(u)

    H = np.array([[1.0, 0.0, -dt * v * np.sin(th)],
                  [0.0, 1.0, dt * v * np.cos(th)],
                  [0.0, 0.0, 1.0]])
    
    return H

def motion_Q(miu, dt):
    x, y, th = np.squeeze(miu)

    j_w = 0.5*r*dt * np.array([[np.cos(th), np.cos(th)],
                              [np.sin(th), np.sin(th)],
                              [2/l, -2/l]])
    # TODO: Ajustar valores
    kr = 0.725
    kl = 0.715 # 89
    sigma_j = np.array([[kr*np.abs(wr), 0],
                        [0, kl*np.abs(wl)]])
    
    Q = np.dot(np.dot(j_w, sigma_j), j_w.T)

    return Q

def observation_model(m, miu_pred):
    x, y, th = np.squeeze(miu_pred)
    lx, ly = np.squeeze(m)
    # print("DistanciaX calculo", lx - x)
    # print("DistanciaZ calculo", ly - y)

    rho = np.sqrt((lx - x)**2 + (ly - y)**2)
    alpha = np.arctan2(ly - y, lx - x) - th
    alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
    # print("alpha medido", alpha)
    # print("Z calculada", np.array([[rho], [alpha]]))

    return np.array([[rho], [alpha]])

def observation_lectures(translation, rotation):
    # x, y, th = np.squeeze(miu_pred)

    transform = tf_buffer.lookup_transform("base_link", "camera_frame_optical", rospy.Time())
    # translation = transform.transform.translation
    # rotation = transform.transform.rotation
    
    translation_camera = np.array([translation.x, translation.y, translation.z])
    transform_camera_to_robot = tf2_geometry_msgs.transform_to_matrix(transform.transform)
    translation_robot = np.dot(transform_camera_to_robot[:3, :3], translation_camera) + transform_camera_to_robot[:3, 3]

    # listener = tf.TransformListener()
    # listener.waitForTransform("base_link", "camera_frame_optical", rospy.Time(), rospy.Duration(4.0))
    # (trans, rot) = listener.lookupTransform("base_link", "camera_frame_optical", rospy.Time())
    # translation_camera = np.array([translation.x, translation.y, translation.z])

    # # Matriz de transformacion desde el marco de la camara al marco del robot
    # transform_camera_to_robot = tftr.compose_matrix(translate=trans, angles=tftr.euler_from_quaternion(rot))

    # # Transformar el vector de traslacion y rotacion al marco del robot
    # translation_robot = np.dot(transform_camera_to_robot[:3, :3], translation_camera) + transform_camera_to_robot[:3, 3]
    # #rotation_robot = tftr.quaternion_multiply(rot, rotation)

    rho = np.sqrt((translation_robot[0])**2 + (translation_robot[1])**2 + (translation_robot[2])**2)
    alpha = np.arctan2(translation_robot[1], translation_robot[0]) #- th
    alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
    # print("alpha lectura", alpha)
    # print("Z real", np.array([[rho], [alpha]]))

    return np.array([[rho], [alpha]])   

def observation_model_jacobian(m, miu_pred):
    x, y, th = np.squeeze(miu_pred)
    lx, ly = np.squeeze(m)

    dx = lx - x
    dy = ly - y
    p = dx**2 + dy**2

    G = np.array([[-(dx/np.sqrt(p)), -(dy/np.sqrt(p)), 0.0], 
                  [dy/p, -(dx/p), -1.0]])
    
    return G 

def main():

    miu = np.array([[0], [0], [0]]) # Robot initial position 
    sigma = np.zeros((3, 3)) # Initial covariance matrix

    # Landmark position

    # M = {18:(1, 4),
    #      12:(7, 10),
    #      5:(4, -2),
    #      27:(2, -1)}

    # M = {18:(1, 2),
    #      12:(2, 5),
    #      5:(-1, 9),
    #      27:(2, -1)}

    M = {18:(1, 2),
         12:(2, 5),
         5:(-1, 9)}

    Q = np.zeros((3, 3)) # Motion model covariance matrix
    R = np.array([[0.002, 0], 
                  [0, 0.002]]) # Observation model covariance matrix

    f1 = r/l # To save some calculations
    f2 = 0.5*r

    v = 0.0
    w = 0.0
    x, y, th = np.squeeze(miu)

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "odom"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    brStatic.sendTransform(t)

    initial_state = PoseStamped()
    initial_state.header = t.header
    initial_state.pose.position = t.transform.translation
    initial_state.pose.orientation = t.transform.rotation

    current_time = rospy.get_time()
    last_time = current_time

    while not rospy.is_shutdown():

        cTime = rospy.Time.now()

        v = (wr + wl) * f2 # Compute of linear velocity
        w = (wr - wl) * f1 # Compute of angular velocity

        u = np.array([[v], [w]])

        current_time = rospy.get_time()
        dt = current_time - last_time
        last_time = current_time       

        miu, sigma, robot_pose, odom = kalman(M, miu, sigma, u, fiducials, Q, R, dt, cTime)

        x, y, th = np.squeeze(miu)
        qRota = tf.transformations.quaternion_from_euler(0, 0, th)

        robot_pose.pose.pose.position.x = x
        robot_pose.pose.pose.position.x = y
        qRota = tf.transformations.quaternion_from_euler(0, 0, th)
        robot_pose.pose.pose.orientation = Quaternion(qRota[0], qRota[1], qRota[2], qRota[3])
        pose_cov = np.zeros((6, 6))
        pose_cov[0][0] = sigma[0][0]
        pose_cov[1][0] = sigma[1][0]
        pose_cov[1][1] = sigma[1][1]
        pose_cov[0][5] = sigma[0][2]
        pose_cov[0][1] = sigma[0][1]
        pose_cov[1][5] = sigma[1][2]
        pose_cov[5][0] = sigma[2][0]
        pose_cov[5][1] = sigma[2][1]
        pose_cov[5][5] = sigma[2][2]
        robot_pose.pose.covariance = (pose_cov.flatten().tolist())
        pPose.publish(robot_pose)

        odom.pose.pose.position = Point(x, y, r)
        odom.pose.pose.orientation = Quaternion(qRota[0], qRota[1], qRota[2], qRota[3])
        odom.pose.covariance = pose_cov.flatten().tolist()
        pOdom.publish(odom)

        t = TransformStamped()
        t.header.stamp = cTime
        t.header.frame_id = "odom"
        t.child_frame_id = "kalman/base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation = robot_pose.pose.pose.orientation
        br.sendTransform(t)
        
        # Sleep
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('ekf_localisation')

        # tb = tf.TransformBroadcaster()
        br = tf2_ros.TransformBroadcaster()
        brStatic = tf2_ros.StaticTransformBroadcaster()
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        pOdom = rospy.Publisher('/odom', Odometry, queue_size=10)
        pPose = rospy.Publisher('/kalman_pose', PoseWithCovarianceStamped, queue_size=10)
        pJS = rospy.Publisher('/joint_states', JointState, queue_size=10)

        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, arucos_callback)
        rospy.Subscriber('/wl', Float32, wl_callback)
        rospy.Subscriber('/wr', Float32, wr_callback)

        rate = rospy.Rate(30)

        main()
    except rospy.ROSInitException:
        pass