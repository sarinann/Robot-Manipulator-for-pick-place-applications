#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

def manipulator_control(q1, q2, q3, q4, q5, q6):
    rospy.init_node('manipulator_control') 
    motor_1 = rospy.Publisher('/manipulator/shoulder_pan_trans/command', Float64, queue_size=10)
    motor_2 = rospy.Publisher('/manipulator/shoulder_lift_trans/command', Float64, queue_size=10)
    motor_3 = rospy.Publisher('/manipulator/elbow_trans/command', Float64, queue_size=10)
    motor_4 = rospy.Publisher('/manipulator/elbow_motor/command', Float64, queue_size=10)
    motor_5 = rospy.Publisher('/manipulator/wrist_2_trans/command', Float64, queue_size=10)
    motor_6 = rospy.Publisher('/manipulator/wrist_3_trans/command', Float64, queue_size=10)

    rate = rospy.Rate(4) 

    rospy.loginfo("Data is being sent")  
    rospy.loginfo(q1) 
    rospy.loginfo(q2) 
    rospy.loginfo(q3) 
    rospy.loginfo(q4) 
    rospy.loginfo(q5) 
    rospy.loginfo(q6) 
    rospy.loginfo("-----------------------") 

    # while not rospy.is_shutdown():
    twist = Float64()

    #Initial pose
    twist.data = q1
    motor_1.publish(twist)
    twist.data = q2
    motor_2.publish(twist)
    twist.data = q3
    motor_3.publish(twist)
    twist.data = q4
    motor_4.publish(twist)
    twist.data = q5
    motor_5.publish(twist)
    twist.data = q6
    motor_6.publish(twist)
    rate.sleep()


a, d, alpha, theta, theta1, theta2, theta3, theta4, theta5, theta6, t = sp.symbols('a d alpha theta theta1 theta2 theta3 theta4 theta5 theta6 t')    

#Defining the transformation matrices
T = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
              [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
              [0, sp.sin(alpha), sp.cos(alpha), d],
              [0, 0, 0, 1]])

T01 = sp.Matrix([[sp.cos(theta1), -sp.sin(theta1)*sp.cos(alpha), sp.sin(theta1)*sp.sin(alpha), a*sp.cos(theta1)],
              [sp.sin(theta1), sp.cos(theta1)*sp.cos(alpha), -sp.cos(theta1)*sp.sin(alpha), a*sp.sin(theta1)],
              [0, sp.sin(alpha), sp.cos(alpha), d],
              [0, 0, 0, 1]])
T01 = T01.subs([(d, 0.089159), (a, 0), (alpha, sp.pi/2)])


T12 = sp.Matrix([[sp.cos(theta2), -sp.sin(theta2)*sp.cos(alpha), sp.sin(theta2)*sp.sin(alpha), a*sp.cos(theta2)],
              [sp.sin(theta2), sp.cos(theta2)*sp.cos(alpha), -sp.cos(theta2)*sp.sin(alpha), a*sp.sin(theta2)],
              [0, sp.sin(alpha), sp.cos(alpha), d],
              [0, 0, 0, 1]])
T12 = T12.subs([(d, 0), (a, -0.425), (alpha, 0)])


T23 = sp.Matrix([[sp.cos(theta3), -sp.sin(theta3)*sp.cos(alpha), sp.sin(theta3)*sp.sin(alpha), a*sp.cos(theta3)],
              [sp.sin(theta3), sp.cos(theta3)*sp.cos(alpha), -sp.cos(theta3)*sp.sin(alpha), a*sp.sin(theta3)],
              [0, sp.sin(alpha), sp.cos(alpha), d],
              [0, 0, 0, 1]])
T23 = T23.subs([(d, 0), (a, -0.39225), (alpha, 0)]) 


T34 = sp.Matrix([[sp.cos(theta4), -sp.sin(theta4)*sp.cos(alpha), sp.sin(theta4)*sp.sin(alpha), a*sp.cos(theta4)],
              [sp.sin(theta4), sp.cos(theta4)*sp.cos(alpha), -sp.cos(theta4)*sp.sin(alpha), a*sp.sin(theta4)],
              [0, sp.sin(alpha), sp.cos(alpha), d],
              [0, 0, 0, 1]])
T34 = T34.subs([(d, 0.10915), (a, 0), (alpha, sp.pi/2)])


T45 = sp.Matrix([[sp.cos(theta5), -sp.sin(theta5)*sp.cos(alpha), sp.sin(theta5)*sp.sin(alpha), a*sp.cos(theta5)],
              [sp.sin(theta5), sp.cos(theta5)*sp.cos(alpha), -sp.cos(theta5)*sp.sin(alpha), a*sp.sin(theta5)],
              [0, sp.sin(alpha), sp.cos(alpha), d],
              [0, 0, 0, 1]])
T45 = T45.subs([(d, 0.09465), (a, 0), (alpha, -1*sp.pi/2)])


T56 = sp.Matrix([[sp.cos(theta6), -sp.sin(theta6)*sp.cos(alpha), sp.sin(theta6)*sp.sin(alpha), a*sp.cos(theta6)],
              [sp.sin(theta6), sp.cos(theta6)*sp.cos(alpha), -sp.cos(theta6)*sp.sin(alpha), a*sp.sin(theta6)],
              [0, sp.sin(alpha), sp.cos(alpha), d],
              [0, 0, 0, 1]])
T56 = T56.subs([(d, 0.0823), (a, 0), (alpha, 0)])


#Final homogeneous transformation matrix from frame 2 to frame 0
T02 =T01*T12
#Final homogeneous transformation matrix from frame 3 to frame 0
T03 =T02*T23
#Final homogeneous transformation matrix from frame 4 to frame 0
T04 =T03*T34
#Final homogeneous transformation matrix from frame 5 to frame 0
T05 =T04*T45
#Final homogeneous transformation matrix from frame 6 to frame 0
T06 = T05*T56
#T06.simplify()
#print("The final transformation matrix is given by:")
#sp.pprint(T06)

#Extracring the elements to find the Jacobian matrix
#The end effector coordinates can be determined from the last column of the final transformation matrix H07
#To find the z vector take the 3rd column of the Transformation matrix

J0 = sp.Matrix([[sp.diff(T06[0,3],theta1),sp.diff(T06[0,3],theta2),sp.diff(T06[0,3],theta3),sp.diff(T06[0,3],theta4),sp.diff(T06[0,3],theta5),sp.diff(T06[0,3],theta6)],
               [sp.diff(T06[1,3],theta1),sp.diff(T06[1,3],theta2),sp.diff(T06[1,3],theta3),sp.diff(T06[1,3],theta4),sp.diff(T06[1,3],theta5),sp.diff(T06[1,3],theta6)],
               [sp.diff(T06[2,3],theta1),sp.diff(T06[2,3],theta2),sp.diff(T06[2,3],theta3),sp.diff(T06[2,3],theta4),sp.diff(T06[2,3],theta5),sp.diff(T06[2,3],theta6)],
               [T01[0,2],T02[0,2],T04[0,2],T05[0,2],T06[0,2],T06[0,2]],
               [T01[1,2],T02[1,2],T04[1,2],T05[1,2],T06[1,2],T06[1,2]],
               [T01[2,2],T02[2,2],T04[2,2],T05[2,2],T06[2,2],T06[2,2]]])
J0.simplify()
#print("The Jacobian matrix is given by:")
#sp.pprint(J0)

#Finding angular velocity in terms of angular velocity and time
omega=(2*3.14)/5
dt=0.125
time = np.arange(0, 5, dt)

#Calculating end effector velocity
x_dot=sp.Matrix([[0],
                [omega*0.1*sp.cos(omega*t)],
                [-omega*0.1*sp.sin(omega*t)],
                [0],
                [0],
                [0]])

#Initial joint angle configuration at t=0 
q1=-0.215031984094669 + sp.pi/2
q2=0.215031984094669 - sp.pi/2
q3=-8.06081303635207e-19 + sp.pi/2
q4=3.3227320700992 - sp.pi
q5=-6.08412812358377 - sp.pi/2
q6=2.76139605348457


for i in range(0,40):
    
    if(i!=0):
        manipulator_control(q1, q2, q3, q4, q5, q6)

    T06_=T06.subs([(theta1, q1), (theta2, q2), (theta3, q3), (theta4, q4), (theta5, q5), (theta6, q6)])
    #if(i>1): #For calibration
        #ax.scatter3D(T06_[0,-1], T06_[1,-1], T06_[2,-1], color = "red")
    jbn=J0.subs([(theta1, q1), (theta2, q2), (theta3, q3), (theta4, q4), (theta5, q5), (theta6, q6)])
    A_inv=(jbn.evalf()).inv()                                                                
    x_dot_ = x_dot.subs([(t,time[i])]).evalf()
    q_dot=(A_inv * x_dot_).evalf()
    q1 = q1+(q_dot[0,0])*dt
    q2 = q2+(q_dot[1,0])*dt
    q3 = q3+(q_dot[2,0])*dt
    q4 = q4+(q_dot[3,0])*dt
    q5 = q5+(q_dot[4,0])*dt
    q6 = q6+(q_dot[5,0])*dt
    #plt.pause(0.1)

if __name__ == '__main__':
    try:
        0
    except rospy.ROSInterruptException: 
        pass
