#!/usr/bin/env python
import numpy as np
import numpy as np
import rospy, tf
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseWithCovariance, TwistWithCovariance, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray
from fiducial_msgs.msg import FiducialTransformArray
import tf.transformations as tftr
import csv

z = {}
wl, wr = 0.0, 0.0
r, l = 0.05, 0.18
fiducials = None
FS = 90
T = 1.0/FS
ruta_archivo = "datos.csv"

# def generar_csv(z_i_lectura, z_pred_calculada, Miu_antigua, Miu_corregida, Error):
#     # Abrir el archivo en modo escritura
#     with open(ruta_archivo, 'a') as archivo_csv:
#         # Crear el objeto escritor CSV
#          # Crear el objeto escritor CSV
#         escritor_csv = csv.writer(archivo_csv)

#         # Escribir los encabezados de las columnas
#         escritor_csv.writerow(["z_i_lectura", "z_pred_calculada", "Miu_antigua", "Miu_corregida", "Error"])

#         # Escribir los datos en las columnas correspondientes
#         escritor_csv.writerow([z_i_lectura, z_pred_calculada, Miu_antigua, Miu_corregida, Error])

#     print("Archivo CSV generado exitosamente.")

def wl_callback(msg):
    global wl
    wl = msg.data

def wr_callback(msg):
    global wr
    wr = msg.data

# def aruco_callback(msg):
#         global z
#         aruco_data = msg.data
        
#         if len(aruco_data) > 0:
#             z[aruco_data[2]] = np.array([aruco_data[0], aruco_data[1]])
#         elif len(aruco_data) == 0:
#             z = {}

def arucos_callback(fiducial_array):
    global fiducials
    fiducials = fiducial_array.transforms

    
    # for fiducial in fiducials:
    #     marker_id = fiducial.fiducial_id
    #     translation = fiducial.transform.translation
    #     rotation = fiducial.transform.rotation

def kalman(M, miu, sigma, u, z, Q, R, dt):
    global fiducials
    miu_pred = motion_model(miu, u, dt)
    miu_antigua = motion_model(miu, u, dt)
    H = motion_model_jacobian(miu, u, dt)
    Q = motion_Q(miu, dt)
    sigma_pred = np.dot(np.dot(H, sigma), H.T) + Q
    #print("Miu antigua", miu_pred)

    #return miu_pred, sigma_pred

    if fiducials is not None:
        print("Elementos en fiducials", len(fiducials))
        for fiducial in fiducials:
            marker_id = fiducial.fiducial_id
            translation = fiducial.transform.translation
            rotation = fiducial.transform.rotation
            
            print("Translation: ", translation)
            print("rotation: ", rotation)


            m = np.array([[M[marker_id][0]], [M[marker_id][1]]])

            z_i = observation_lectures(translation, rotation, miu_pred)
            z_pred = observation_model(m, miu_pred)

            G = observation_model_jacobian(m, miu_pred)
            
            Z = np.dot(np.dot(G, sigma_pred), G.T) + R
            
            K = np.dot(np.dot(sigma_pred, G.T), np.linalg.inv(Z))
            print(K.shape)
            print("z_i_lectura", z_i)
            print("z_pred_calculada", z_pred)
            print("Miu_antigua", miu_pred)
            error = z_i - z_pred
            error[1][0] = np.arctan2(np.sin(error[1][0]), np.cos(error[1][0]))
            #Normalizar error
            print("Error", error)
            miu_pred += np.dot(K, (error))
            print("Miu_corregida", miu_pred)
            I = np.eye(len(miu_pred))
            sigma_pred = np.dot((I - np.dot(K, G)), sigma_pred)
            #generar_csv(z_i, z_pred, miu_antigua, miu_pred, error)
            # while True:
            #      pass
        fiducials = None

                

    
    return miu_pred, sigma_pred

# def wrapped_angle(angle):
#     if angle > np.pi:
#         angle -= 2 * np.pi
#     elif angle < -np.pi:
#         angle += 2 * np.pi

#     return angle

def motion_model(miu, u, dt):
    x, y, th = np.squeeze(miu)
    v, w = np.squeeze(u)
    
    x_new = x + dt * v * np.cos(th) 
    y_new = y + dt * v * np.sin(th) 
    th_new = th + dt * w 

    th_new = np.arctan2(np.sin(th_new), np.cos(th_new))
    #th_new = np.fmod(th_new + np.pi, 2 * np.pi) - np.pi

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
    kr = 0.5
    kl = 0.8
    sigma_j = np.array([[kr*np.abs(wr), 0],
                        [0, kl*np.abs(wl)]])
    
    Q = np.dot(np.dot(j_w, sigma_j), j_w.T)

    return Q

def observation_model(m, miu_pred):
    x, y, th = np.squeeze(miu_pred)
    lx, ly = np.squeeze(m)

    print("DistanciaX calculo", lx - x)
    print("DistanciaZ calculo", ly - y)

    rho = np.sqrt((lx - x)**2 + (ly - y)**2)
    alpha = np.arctan2(ly - y, lx - x) - th

    alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

    print("alpha medido", alpha)

    #alpha = np.fmod(alpha + np.pi, 2 * np.pi) - np.pi
    #Checar angulo normalizado
    #print("Z calculada", np.array([[rho], [alpha]]))
    return np.array([[rho], [alpha]])

def observation_lectures(translation, rotation, miu_pred):
    x, y, th = np.squeeze(miu_pred)

    #rho = np.sqrt((translation.z)**2 + (translation.x)**2)
    listener = tf.TransformListener()
    listener.waitForTransform("base_link", "camera_frame_optical", rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform("base_link", "camera_frame_optical", rospy.Time())
    x_camera = translation.x
    y_camera = translation.y
    z_camera = translation.z
    translation_camera = np.array([x_camera, y_camera, z_camera])
    # Matriz de transformacion desde el marco de la camara al marco del robot
    transform_camera_to_robot = tftr.compose_matrix(translate=trans, angles=tftr.euler_from_quaternion(rot))

    # Transformar el vector de traslacion y rotacion al marco del robot
    translation_robot = np.dot(transform_camera_to_robot[:3, :3], translation_camera) + transform_camera_to_robot[:3, 3]
    #rotation_robot = tftr.quaternion_multiply(rot, rotation)

    # translation.x = translation.x * -1
    # print("DistanciaX camara", translation.x)
    # print("DistanciaZ camara", translation.z)
    rho = np.sqrt((translation_robot[0])**2 + (translation_robot[1])**2 + (translation_robot[2])**2)
    alpha = np.arctan2(translation_robot[1], translation_robot[0]) #- th
    #alpha = np.arccos(translation.z/translation.x)
    #Intentar corregir el sistema de coordenadas
    # alpha = rotation.x
    # alpha = angle
    alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

    print("alpha lectura", alpha)

    #alpha = np.fmod(alpha + np.pi, 2 * np.pi) - np.pi
    #print("Z real", np.array([[rho], [alpha]]))

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

    pose_cov = np.zeros((6, 6)) # Pose covariance for Odometry message
    twist_cov = np.zeros((6, 6)) # Twist covariance for Odometry message

    # Landmark position
    # TODO: Cambiar esto?
    # m1 = np.array([[1], [2]]) # 18
    # m2 = np.array([[2], [5]]) # 12
    # m3 = np.array([[-1], [9]]) # 5
    # M = [m1] 

    M = {18:(1, 2),
         12:(2, 5),
         5:(-1, 9)}
    
    # Assuming following conditions remain constant

    Q = np.zeros((3, 3)) # Motion model covariance matrix
    
    R = np.array([[0.002, 0], 
                  [0, 0.002]]) # Observation model covariance matrix
    
    # Assumed measurement at each step k using the LiDAR
    # z_i = np.array([[4.87], [0.8]])
    # z = []

    f1 = r/l # To save some calculations
    f2 = 0.5*r

    v = 0.0
    w = 0.0

    current_time = rospy.get_time()
    last_time = current_time
    t0 = rospy.Time.now()

    while not rospy.is_shutdown():

        
        # Robot pose
        x, y, th = np.squeeze(miu)
        modelPose = PoseWithCovariance()
        modelPose.pose.position = Point(x, y, 0)
        qRota = tf.transformations.quaternion_from_euler(0, 0, th)
        modelPose.pose.orientation = Quaternion(qRota[0], qRota[1], qRota[2], qRota[3])

        pose_cov[0][0] = sigma[0][0]
        pose_cov[1][0] = sigma[1][0]
        pose_cov[1][1] = sigma[1][1]
        pose_cov[0][5] = sigma[0][2]
        pose_cov[0][1] = sigma[0][1]
        pose_cov[1][5] = sigma[1][2]
        pose_cov[5][0] = sigma[2][0]
        pose_cov[5][1] = sigma[2][1]
        pose_cov[5][5] = sigma[2][2]
        modelPose.covariance[:] = np.ravel(pose_cov)

        # Robot twist
        modelTwist = TwistWithCovariance()
        modelTwist.twist.linear.x = v
        modelTwist.twist.angular.z = w
        modelTwist.covariance[:] = np.ravel(twist_cov)

        # Odometry
        odom = Odometry()
        cTime = rospy.Time.now()
        odom.header.stamp = cTime
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose = modelPose
        odom.twist = modelTwist
        pOdom.publish(odom)

        # Transform
        tb.sendTransform([x, y, 0], qRota, cTime, "base_link", "map")

        # Joint state
        js = JointState()
        js.name = ['base_to_right_w', 'base_to_left_w']
        t = cTime - t0
        js.position = [wr*t.to_sec(), wl*t.to_sec()]
        js.header.stamp = cTime
        pJS.publish(js)

        v = (wr + wl) * f2 # Compute of linear velocity
        w = (wr - wl) * f1 # Compute of angular velocity

        u = np.array([[v], [w]])

        current_time = rospy.get_time()
        dt = current_time - last_time
        last_time = current_time       


        miu, sigma = kalman(M, miu, sigma, u, z, Q, R, dt)
        


        # Sleep
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('ekf_localisation')

        tb = tf.TransformBroadcaster()

        pOdom = rospy.Publisher('/odom', Odometry, queue_size=10)
        pJS = rospy.Publisher('/joint_states', JointState, queue_size=10)

        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, arucos_callback)
        rospy.Subscriber('/wl', Float32, wl_callback)
        rospy.Subscriber('/wr', Float32, wr_callback)

        rate = rospy.Rate(90)

        main()
    except rospy.ROSInitException:
        pass