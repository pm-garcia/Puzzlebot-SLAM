#!/usr/bin/env python
import tf, rospy
import time
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose2D, Twist, PoseStamped
import numpy as np
import matplotlib.pyplot as plt


class Nodo():
    
    #Constructor
    def __init__(self):
        
        # Inicializar nodo
        rospy.init_node("nodo_minireto1")
        self.rate = rospy.Rate(90)

        # Publicadores
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.robot_cmd = Twist()

        # Suscriptores 
        rospy.Subscriber("/pose", PoseStamped, self.poseCallback)
        self.q = None
        self.x = None
        self.y = None

        # On shutdown
        rospy.on_shutdown(self.endCallback)

    # Callback para detener el robot
    def endCallback(self):
        self.robot_cmd.linear.x = 0.0
        self.robot_cmd.angular.z = 0.0
        self.cmd_pub.publish(self.robot_cmd)
    
    def poseCallback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        (_, _, self.q) = tf.transformations.euler_from_quaternion([
        msg.pose.orientation.x, msg.pose.orientation.y,
        msg.pose.orientation.z, msg.pose.orientation.w])

    def reset_client(self):
        try:
            print('Starting reset...')
            rospy.wait_for_service('/reset')
            reset_bot = rospy.ServiceProxy('/reset', Empty)
            reset_bot()
            print('End reset.')
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    # Funcion que realiza el movimiento lineal del robot y publica su velocidad 
    def move(self, fwd_speed, seconds):
        t0 = rospy.get_rostime().to_sec()
        while (rospy.get_rostime().to_sec() - t0 <= seconds):
            #print(rospy.get_rostime().to_sec() - t0)
            self.robot_cmd.linear.x = fwd_speed
            self.robot_cmd.angular.z = 0.0
            self.cmd_pub.publish(self.robot_cmd)
            self.rate.sleep()
        self.robot_cmd.linear.x = 0.0
        self.cmd_pub.publish(self.robot_cmd)


    def rotate(self, ang_speed, seconds):
        t0 = rospy.get_rostime().to_sec()
        while (rospy.get_rostime().to_sec() - t0 <= seconds):
            #print(rospy.get_rostime().to_sec() - t0)
            self.robot_cmd.linear.x = 0.0
            self.robot_cmd.angular.z = ang_speed
            self.cmd_pub.publish(self.robot_cmd)
            self.rate.sleep()
        self.robot_cmd.linear.z = 0.0
        self.cmd_pub.publish(self.robot_cmd)

    #def statistics(self, x, y, q):
    def statistics(self, data):

        #bins = 3
        #x, x_bins = np.histogram(x, bins=bins)
        #y, y_bins = np.histogram(y, bins=bins)
        #th1, th_bins = np.histogram(th1, bins=bins)

        # Graficar los histogramas con Matplotlib
        #fig, axs = plt.subplots(3, 1, sharex=True, figsize=(6, 8))

        #axs[0].hist(x, bins=x_bins)
        #axs[0].set_title("Posiciones finales en X")

        #axs[1].hist(y, bins=y_bins)
        #axs[1].set_title("Posiciones finales en Y")

        #axs[2].hist(th1, bins=th_bins)
        #axs[2].set_title("Posiciones finales en theta")

        #plt.tight_layout()
        #plt.show()
        # Crear tres graficas de histogramas, una para cada dimension

        print('Statistics of this run: ')

        # mean_x, std_x = np.mean(x), np.std(x)
        # mean_y, std_y = np.mean(y), np.std(y)
        # mean_q, std_q = np.mean(y), np.std(y)
        # print('For X: ' + str(mean_x) + ' and Std = ' + str(std_x))
        # print('For Y: ' + str(mean_y) + ' and Std = ' + str(std_y))
        # print('For Q: ' + str(mean_q) + ' and Std = ' + str(std_q))

        for key in data:
            x, y, q = data[key]
            print('For ' + str(key) + ': mean = ' + str(np.mean(x)) + ' and Std = ' + str(np.std(x)))
            print('For ' + str(key) + ': mean = ' + str(np.mean(y)) + ' and Std = ' + str(np.std(y)))
            print('For ' + str(key) + ': mean = ' + str(np.mean(q)) + ' and Std = ' + str(np.std(q)))

        print('Computing covariance matrix for this run 3: ')
        print(data["experiment 2"])
        cov = np.cov(data["experiment 2"])
        #data =  np.array([x, y, q])
        #cov = np.cov(data)

        # Compute eigenvalues and eigenvectors
        eigvals, eigvecs = np.linalg.eig(cov)
        print('Eigen Values and Eigen Vectors: ')
        print('S = ' + str(eigvals))
        print('V = ' + str(eigvecs))

        # Compute rotation matrix
        rot_matrix = np.matrix(eigvecs)
        print('Rotation Matrix: ')
        print(rot_matrix)

        # compute ellipsoid of covariance at 95% confidence
        confidence = 0.95
        eigvals_sort_idx = np.argsort(eigvals)[::-1]
        sigma_x = np.sqrt(eigvals[eigvals_sort_idx][0]) * np.sqrt(-2 * np.log(1 - confidence))
        sigma_y = np.sqrt(eigvals[eigvals_sort_idx][1]) * np.sqrt(-2 * np.log(1 - confidence))
        yaw = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0]) * 180 / np.pi
        print('Ellipsoid of covariance at ' + str(confidence*100) + ' confidence: ')
        print('sigmaX = ' + str(sigma_x) + ' sigmaY = ' + str(sigma_y) + ' Yaw = ' + str(yaw))

        # # center all distributions to zero
        # data -= np.mean(data, axis=1, keepdims=True)

        for key in data:
            data[key] -= np.mean(data[key], axis=1, keepdims=True)

        # plot uncertainties vs. runs
        # plt.figure()
        # plt.scatter(0, np.std(x), color="r")
        # plt.scatter(1, np.std(y), color="g")
        # plt.scatter(2, np.std(q), color="b")
        # plt.xlabel("Run")
        # plt.ylabel("Uncertainty")
        # plt.title("Uncertainties vs. Runs")
        # plt.show()

        plt.figure()
        for key in data:
            x, y, z = data[key]
            plt.scatter(int(key.split()[-1]), np.std(x), color="r")
            plt.scatter(int(key.split()[-1]), np.std(y), color="g")
            plt.scatter(int(key.split()[-1]), np.std(z), color="b")
        plt.xlabel("Run")
        plt.ylabel("Uncertainty")
        plt.title("Uncertainties vs. Runs")
        plt.show()


        # end of all experiments
        print("End of ALL experiments of this run")


        

        # fig, axs = plt.subplots(1, 3, figsize=(10, 4))

        # # Histograma de la dimension X
        # axs[0].hist(x, bins=10, color='r')
        # axs[0].set_xlabel('X')
        # axs[0].set_ylabel('Frecuencia')

        # # Histograma de la dimension Y
        # axs[1].hist(y, bins=10, color='g')
        # axs[1].set_xlabel('Y')
        # axs[1].set_ylabel('Frecuencia')

        # # Histograma de la dimension Z
        # axs[2].hist(th1, bins=10, color='b')
        # axs[2].set_xlabel('Z')
        # axs[2].set_ylabel('Frecuencia')

        # # Mostrar la figura
        # plt.show()

        # pass

    # Funcion que ejecuta el movimiento del robot
    def main(self):
        #Lista de posiciones
        x = np.array([])
        y = np.array([])
        q = np.array([])
        data = {}
        #th2 = np.array([])
        # Segundos que durara cada iteracion
        sec_exp = [1, 3, 5]
        cont_exp = 0
        # Numero de iteraciones por experimento 
        iteration_exp = 5
        while not rospy.is_shutdown():
            for i in sec_exp:
                self.reset_client()
                for j in range(0, iteration_exp):
                    print('Starting experiment ' + str(j+1))
                    self.move(0.25, i)
                    # x = np.append(x, self.x)
                    # y = np.append(y, self.y)
                    # q = np.append(q, np.degrees(self.q))
                    data["experiment {}".format(cont_exp)] = np.array([self.x, self.y, np.degrees(self.q)])
                    self.reset_client()
                cont_exp += 1
            print('End of all experiment of this run')
            #self.statistics(x, y, q)
            print(data)
            #self.statistics(data)
            # print(x)
            # print(y)
            # print(q)
            #for i in sec_exp:
            #    for j in range(0, iteration_exp):
            #        self.rotate(0.1, i)
            #        theta = math.degrees(self.th)
            #        z2.append(theta)
                
            break
        #self.move(0.25, 1)
        #self.rotate(0.1, 1)
            


if __name__ == '__main__':

    try:
        nodo = Nodo()
        nodo.main()
    except rospy.ROSInterruptException:
        pass