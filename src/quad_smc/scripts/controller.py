# !/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, asin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

##################################
##################################
#TUNING PARAMETERS FOR CONTROLLER#
LAMBDA = [7 , 12, 12, 8]
K = [10, 120, 120, 10]

Kp = 90
Kd = 10
##################################
##################################

class Quadrotor():

    def __init__(self):

        # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)

        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry", Odometry, self.odom_callback)

        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.mutex_lock_on = False

        self.ohm = 0

        rospy.on_shutdown(self.save_data)

    def traj_evaluate(self):
        t = self.t
        if t <=65:
            if self.t <=5:
                t0, tf = 0,5
                x0,y0,z0 = 0,0,0
                xf,yf,zf = 0,0,1

            elif t <= 20:
            
                    t0, tf = 5, 20
                    x0, y0, z0 = 0, 0, 1
                    xf, yf, zf = 1, 0, 1

            elif t <= 35:
             
                t0, tf = 20, 35
                x0, y0, z0 = 1, 0, 1
                xf, yf, zf = 1, 1, 1

            elif t <= 50:
       
                t0, tf = 35, 50
                x0, y0, z0 = 1, 1, 1
                xf, yf, zf = 0, 1, 1

            else:
               
                t0, tf = 50, 65
                x0, y0, z0 = 0, 1, 1
                xf, yf, zf = 0, 0, 1
        

            A = np.array([[1, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5)],
                                [0, 1, 2 * t0, 3 * pow(t0,2), 4 * pow(t0,3), 5 * pow(t0,4)],
                                [0, 0, 2, 6 * t0, 12 * pow(t0,2), 20 * pow(t0,3)],
                                [1, tf, pow(tf,2), pow(tf,3), pow(tf,4), pow(tf,5)],
                                [0, 1, 2 * tf, 3 * pow(tf,2), 4 * pow(tf,3), 5 * pow(tf,4)],
                                [0, 0, 2, 6 * tf, 12 * pow(tf,2), 20 * pow(tf,3)]]) 
            
            B  = np.array([[x0,0,0,xf,0,0],
                [y0,0,0,yf,0,0],
                [z0,0,0,zf,0,0]])
            
            ax = np.linalg.solve(A,B[0])
            ay = np.linalg.solve(A,B[1])
            az = np.linalg.solve(A,B[2])

            #Desired Trajectories
            x_d = ax[0] + ax[1] * t + ax[2] * pow(t,2) + ax[3] * pow(t,3) + ax[4] * pow(t,4) + ax[5] * pow(t,5)
            y_d = ay[0] + ay[1] * t + ay[2] * pow(t,2) + ay[3] * pow(t,3) + ay[4] * pow(t,4) + ay[5] * pow(t,5)
            z_d = az[0] + az[1] * t + az[2] * pow(t,2) + az[3] * pow(t,3) + az[4] * pow(t,4) + az[5] * pow(t,5)
           
            # Velocity
            x_d_dot = ax[1] + 2 * ax[2]*t + 3 * ax[3] * pow(t,2) + 4*ax[4] * pow(t,3) + 5 * ax[5] * pow(t,4)
            y_d_dot = ay[1] + 2 * ay[2]*t + 3 * ay[3] * pow(t,2) + 4*ay[4] * pow(t,3) + 5 * ay[5] * pow(t,4)
            z_d_dot = az[1] + 2 * az[2]*t + 3 * az[3] * pow(t,2) + 4*az[4] * pow(t,3) + 5 * az[5] * pow(t,4)
            
            # Acceleration
            x_d_ddot = 2 * ax[2] + 6 * ax[3] * t + 12 * ax[4] * pow(t,2) + 20 * ax[5] * pow(t,3)
            y_d_ddot = 2 * ay[2] + 6 * ay[3] * t + 12 * ay[4] * pow(t,2) + 20 * ay[5] * pow(t,3)
            z_d_ddot = 2 * az[2] + 6 * az[3] * t + 12 * az[4] * pow(t,2) + 20 * az[5] * pow(t,3)
            return x_d,y_d,z_d,x_d_dot,y_d_dot,z_d_dot,x_d_ddot,y_d_ddot,z_d_ddot
        else:
            return 0,0,0,0,0,0,0,0,0
        

    def sign_function(self, S):
        #sign function to restrict the value to -1, 0, 1 depending upon the value of S
        sign = min(max(S/1, -1), 1)
        return sign


    def limit_angle(self, angle):
        return (angle+np.pi) % (2 * np.pi)-np.pi

    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):

        ##################################
        ##################################
        ##Given Parameters for the Drone##
        m = 0.027
        g = 9.81
        Ix = 16.571710*10**(-6)
        Iy = 16.571710*10**(-6)
        Iz = 29.261652*10**(-6)
        Ip = 12.65625*10**(-8)
        kF = 1.28192*10**(-8)
        kM = 5.964552*10**(-3)
        l = 0.046
        ##################################
        
        #Obtaining the desired positions, velocities and acceleration from the traj evaluate function
        xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = self.traj_evaluate()
        #Actual values of position and velocity being passed from the odm_callback
        x,y,z = xyz
        x_dot, y_dot, z_dot = xyz_dot
        phi, theta, psi = rpy
        phi_dot, theta_dot, psi_dot = rpy_dot

        #declaring an empty list for the U 
        u = ([0, 0, 0, 0])

        #for u1, designing surface s1
        z_err = z - zd
        z_err_dot = z_dot - zd_dot
        #surface s1
        s1 = z_err_dot + LAMBDA[0] * z_err

        sign_s1 = self.sign_function(s1)

        u[0] = m*(g + zd_ddot - (LAMBDA[0] * z_err_dot) -(K[0] * sign_s1))/((np.cos(phi) * np.cos(theta)))

        #Given
        Fx = m*(-Kp*(x-xd) - Kd*(x_dot-xd_dot) + xd_ddot)
        Fy = m*(-Kp*(y-yd) - Kd*(y_dot-yd_dot) + yd_ddot)
        phi_dot_d = 0
        theta_dot_d = 0
        psi_d = 0
        psi_dot_d = 0

        #for the desired phi and theta
        res_Fx = max(min(Fx / u[0],1),-1)
        res_Fy = max(min(Fy / u[0],1),-1) 
        theta_d = np.arcsin(res_Fx)
        phi_d = np.arcsin(-res_Fy)
        

        #for u2, desiging surface s2:
        
        phi_err = self.limit_angle(phi - phi_d)
        phi_err_dot = phi_dot - phi_dot_d
        #surface s2
        s2 = phi_err_dot + LAMBDA[1] * phi_err
        
        sign_s2 = self.sign_function(s2)
        
        u[1] = (Ip * self.ohm* theta_dot) - (Ix*LAMBDA[1]*phi_err_dot) - (theta_dot * psi_dot * (Iy - Iz)) - (K[1] * sign_s2 * Ix)
        

        #for u3, designing surface s3:

        theta_err = self.limit_angle(theta - theta_d)
        theta_err_dot = theta_dot - theta_dot_d
        #surface s3
        s3 = theta_err_dot + LAMBDA[2] * theta_err
        
        sign_s3 = self.sign_function(s3)
        
        u[2] = -(LAMBDA[2] * Iy * theta_err_dot) - (Ip * self.ohm * phi_dot) - (phi_dot * theta_dot * (Iz - Ix)) - (K[2] * Iy * sign_s3)


        #for u4, designing surface s4:

        psi_err = self.limit_angle(psi - psi_d)
        psi_err_dot = psi_dot - psi_dot_d
        #surface s4
        s4 = psi_err_dot + LAMBDA[3] * psi_err
        
        sign_s4 = self.sign_function(s4)
        
        u[3] = -(phi_dot * theta_dot * (Ix - Iy)) - (Iz * LAMBDA[3] * psi_err_dot) - (K[3] * sign_s4 * Iz)

        #Given Allocation Matrix 
        allocation_matrix = np.array(([1/(4 * kF), -sqrt(2)/(4 * kF * l), -sqrt(2)/(4 * kF * l), -1/(4 * kM * kF)],
                              [1/(4 * kF), -sqrt(2)/(4 * kF * l), sqrt(2)/(4 * kF * l),  1/(4 * kM * kF)],
                              [1/(4 * kF),  sqrt(2)/(4 * kF * l), sqrt(2)/(4 * kF * l), -1/(4 * kM * kF)],
                              [1/(4 * kF),  sqrt(2)/(4 * kF * l), -sqrt(2)/(4 * kF * l),  1/(4 * kM * kF)]))
       
        #To check if the square of the angular velocity is negative and making it 0 if found true

        ang_vel_sqr = allocation_matrix @ u
        for i in range(len(ang_vel_sqr)):
            if np.sign(ang_vel_sqr[i]) == -1:
                ang_vel_sqr[i] = 0

        #angular velocity
        ang_vel = ang_vel_sqr**0.5

        #Capping the angular velocity of the motors in between 0 to 2618
        ang_vel = [min(max(x,0),2618) for x in ang_vel]

        self.ohm = ang_vel[0] - ang_vel[1] + ang_vel[2] - ang_vel[3]

        #Publishing the speeds to the motors
        motor_speed = Actuators()
        motor_speed.angular_velocities = [ang_vel[0], ang_vel[1], ang_vel[2], ang_vel[3]]
        self.motor_speed_pub.publish(motor_speed)       

    def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0
        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],[0, np.cos(rpy[0]), -np.sin(rpy[0])], [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]]), w_b)
        rpy = np.expand_dims(rpy, axis=1)
        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])

        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)

    # save the actual trajectory data
    def save_data(self):
        #Caution: Change the directory
        with open("/home/anuj/rbe502_project/src/quad_smc/scripts/log.pkl", "wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series,self.x_series,self.y_series,self. z_series], fp)


if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
       rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        

        

