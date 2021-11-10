import numpy as np
from numpy import linalg as LA
from scipy.linalg import logm
import math
import copy

# Robot Informations

def Screws():
    w_1 = np.array([-1.0, 0.0, 0.0])
    w_2 = np.array([1.0, 0.0, 0.0])
    w_3 = np.array([1.0, 0.0, 0.0])
    w_4 = np.array([-1.0, 0.0, 0.0])
    w_5 = np.array([0.5774,0.5774,0.5774])
    w_6 = np.array([0.3574,0.3574,0.8629])
    w_7 = np.array([-1.0, 0.0, 0.0])

    p_1 = np.array([0.0, 0.0, 0.333])
    p_2 = np.array([0.0, 0.0, 0.0])
    p_3 = np.array([0.0, -0.316, 0.0])
    p_4 = np.array([0.0825, 0.0, 0.0])
    p_5 = np.array([-0.0825, 0.384, 0.0])
    p_6 = np.array([0.0, 0.0, 0.0])
    p_7 = np.array([0.088, 0.0, 0.0])

    q_1 = p_1
    q_2 = p_1+p_2
    q_3 = p_1+p_2+p_3
    q_4 = p_1+p_2+p_3+p_4
    q_5 = p_1+p_2+p_3+p_4+p_5
    q_6 = p_1+p_2+p_3+p_4+p_5+p_6
    q_7 = q_6+p_7;
    
    S = np.array([7,6])
    S[:,0] = np.concatenate([w_1, -np.cross(w_1, q_1)])
    S[:,1] = np.concatenate([w_2, -np.cross(w_2, q_2)])
    S[:,2] = np.concatenate([w_3, -np.cross(w_3, q_3)])
    S[:,3] = np.concatenate([w_4, -np.cross(w_4, q_4)])
    S[:,4] = np.concatenate([w_5, -np.cross(w_5, q_5)])
    S[:,5] = np.concatenate([w_6, -np.cross(w_6, q_6)])
    S[:,6] = np.concatenate([w_7, -np.cross(w_7, q_7)])
    return S

def JointLimit():
    lb = np.zeros(7)
    ub = np.zeros(7)

    lb[0] = -2.8973
    lb[1] = -1.7628
    lb[2] = -2.8973
    lb[3] = -3.0718
    lb[4] = -2.8973
    lb[5] = -0.0175
    lb[6] = -2.8973

    ub[0] = 2.1750
    ub[1] = 2.1750
    ub[2] = 2.1750
    ub[3] = 2.1750
    ub[4] = 2.6100
    ub[5] = 2.6100
    ub[6] = 2.6100

    return [lb, ub]

# Functions for robotics

def Omega_to_SO3(w, theta):  
    skeww = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    R = np.eye((3)) + skeww.dot(np.sin(theta)) + skeww.dot(skeww).dot(1-np.cos(theta))

    return R

def SO3_to_Omega(R):
    w = np.zeros((3,1))
    cos_theta = (np.trace(R)-1)/2
    theta = np.arccos(cos_theta)
    if np.abs(theta) < np.power(0.1,6) :
        w = np.zeros((3,1))
    else :
        if np.abs(theta-np.pi) < np.power(0.1,6) : 
            for k in range(3) :
                if abs(1 + R[k,k]) > np.power(0.1,6) :
                    break
            w = copy.deepcopy(R[:,k])
            w[k] = w[k] + 1
            w = w / np.sqrt(2*(1+R[k,k])) * theta
        else : 
            w_hat = (R-np.transpose(R)) / (2*np.sin(theta)) * theta
            w[0] = w_hat[2,1]
            w[1] = w_hat[0,2]
            w[2] = w_hat[1,0]
    return w

def SO3_to_EulerZYX(R):
    a = math.atan2(R[1,0],R[0,0])
    b = np.arcsin(-R[2,0])
    c = math.atan2(R[2,1],R[2,2])
    # output EulerZYX angle in degree 
    # a = a * 180 / np.pi
    # b = b * 180 / np.pi
    # c = c * 180 / np.pi
    return [a,b,c] 

def EulerZYX_to_SO3(a,b,c):
    Rz = np.array([[np.cos(a), -np.sin(a), 0.0],[np.sin(a), np.cos(a), 0.0],[0.0, 0.0, 1.0]])
    Ry = np.array([[np.cos(b), 0.0, np.sin(b)],[0.0, 1.0, 0.0],[-np.sin(b), 0.0, np.cos(b)]])
    Rx = np.array([[1.0, 0.0, 0.0],[0.0, np.cos(c), -np.sin(c)],[0.0, np.sin(c), np.cos(c)]])
    R = ( Rz.dot(Ry) ).dot(Rx)
    return R

# def Screw_to_SE3(Screw, theta):
#     w = Screw[0:3]
#     v = Screw[3:6]

#     skeww = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
#     G = np.eye((3)).dot(theta) + skeww.dot(1 - np.cos(theta)) + skeww.dot(skeww).dot(theta - np.sin(theta))

#     T = np.eye(4)
#     T[0:3,0:3] = Omega_to_SO3(w, theta)
#     T[0:3,3] = G.dot(v)

#     return T

def Screw_to_SE3(Screw, theta):
    w = Screw[0:3]
    v = Screw[3:6]
    T = np.eye(4)

    if LA.norm(w) < 1e-20:   # w == 0, |v| == 1
        T[0:3,0:3] = np.eye(3)
        T[0:3,3] = v * theta
        return T
    
    else:     # |w| != 0
        if (LA.norm(w)-1) > 1e-8: # |w| != 1
            v = v / LA.norm(w)
            theta *= LA.norm(w)
            w = w / LA.norm(w)  # should be normalized (|w| == 1)
            
        skeww = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        G = np.eye((3)).dot(theta) + skeww.dot(1 - np.cos(theta)) + skeww.dot(skeww).dot(theta - np.sin(theta))
        T = np.eye(4)
        T[0:3,0:3] = Omega_to_SO3(w, theta)
        T[0:3,3] = G.dot(v)
        return T


def SE3_to_CartesianCoordinate(T):
    x = np.zeros((6,1))
    x[0:3] = SO3_to_Omega(T[0:3,0:3])
    x[3:6] = T[0:3,3].reshape(3,1)

    return x

def Adjoint_T(T):
    R = T[0:3,0:3]
    p = T[0:3,3]
    skewp = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    
    AdT = np.eye(6)
    AdT[0:3][:,0:3] = R
    AdT[0:3][:,3:6] = np.zeros((3,3))
    AdT[3:6][:,0:3] = skewp.dot(R)
    AdT[3:6][:,3:6] = R

    return AdT

def Tinv(T):
    R=T[0:3,0:3]
    p=T[0:3,3]
    Tinv=np.eye(4)
    Tinv[0:3,0:3]=np.transpose(R)
    Tinv[0:3,3]=-np.transpose(R).dot(p)
    return Tinv

# def GG(W,theta):
#     if (W.dot(W)-1)>0.0000001:
#         print ("warning! GG!!")
#     w=np.zeros((3,3))
#     w[0,1]=-W[2]
#     w[0,2]=W[1]
#     w[1,2]=-W[0]
#     w[1,0]=W[2]
#     w[2,0]=-W[1]
#     w[2,1]=W[0]
#     return np.eye(3).dot(theta)+(1-np.cos(theta)).dot(w)+(theta-np.sin(theta)).dot(w).dot(w)

# def SE3_to_OmegaV(T):
#     R=T[0:3,0:3]
#     p=T[0:3,3]
#     w=SO3_to_Omega(R).reshape(3)
#     if LA.norm(w) < 0.00000001:
#         v=p
#     else:
#         w_hat=w/LA.norm(w)
#         theta=LA.norm(w)
#         v=LA.inv(GG(w_hat,theta)).dot(p)
#     OmegaV=np.zeros((6))
#     OmegaV[0:3]=w
#     OmegaV[3:6]=v
#     return OmegaV

def Jac_s(Screws, theta):
    Js = np.eye((6))
    M = np.eye((4))
    for i in range(np.size(Screws,axis=1)):
        Js[:,i] = Adjoint_T(M).dot(Screws[:, i])
        M = M.dot(Screw_to_SE3(Screws[:,i], theta[i,0]))
    return Js

def Jac_b(Screws, theta, M_EF):
    Tsb = Screws_to_SE3(Screws, theta).dot(M_EF)
    Tbs = Tinv(Tsb)
    Jb = Adjoint_T(Tbs).dot(Jac_s(Screws, theta))
    return Jb

def Jac_c(Screws, theta, M_EF):
    Tsb = Screws_to_SE3(Screws, theta).dot(M_EF)
    Rsb = Tsb[0:3,0:3]
    Jc = np.eye((6))
    Jc[0:3,:] = Jac_s(Screws, theta)[0:3,:]
    Jc[3:6,:] = Rsb.dot(Jac_b(Screws, theta, M_EF)[3:6,:])
    return Jc

def Jac_c_dot(Screws, theta, theta_dot, M_EF, timestep):
    
    Jc_dot = np.zeros((6,6))
    delta = 0.001

    for i in range(6):
        delta_theta = np.zeros((6,1))
        delta_theta[i,0] = delta
        theta_prime = theta + delta_theta
        Jc_dot = Jc_dot + (Jac_c(Screws, theta_prime, M_EF)-Jac_c(Screws, theta, M_EF)).dot(theta_dot[i,0]) / timestep

    return Jc_dot

def Screws_to_SE3(Screws, theta):
    T = np.eye((4))
    for i in range(np.size(Screws,axis=1)):
        T = T.dot(Screw_to_SE3(Screws[:,i], theta[i,0]))
    return T

def ForwardKinematics(Screws, theta, M_EF):
    T=Screws_to_SE3(Screws, theta)
    return T.dot(M_EF)

def InvKinematics(Screws, q_init, M_EF, T_desired):
    # Stepsize
    k = 0.5

    # Tolerance for stopping iteration
    Tolerance = 0.001

    # Initialization
    q_iteration = q_init
    T = Screws_to_SE3(Screws, q_iteration).dot(M_EF)
    Iteration = 1
    V = np.zeros((6,1))
    # Iteration algorithm
    while LA.norm(T - T_desired, 'fro') > Tolerance:
        K = logm(T_desired.dot(Tinv(T)))
        # print(T_desired)
        V[0:3,0] = [K[2,1], K[0,2], K[1,0]]
        V[3:6,0] = K[0:3,3]
        delta_q = k * LA.pinv(Jac_s(Screws, q_iteration)).dot(V)
        q_iteration = q_iteration + delta_q
        T = Screws_to_SE3(Screws, q_iteration).dot(M_EF)
        Iteration = Iteration + 1
        if Iteration > 500:
            break
    # print("iteration in InvKinematics: %d" %Iteration)

    return q_iteration

def InvKinematics2(Screws, q_init, M_EF, T_desired):
    # Stepsize
    k = 0.1

    # Tolerance for stopping iteration
    Tolerance = 0.00001

    # Initialization
    q_iteration = q_init
    T = Screws_to_SE3(Screws, q_iteration).dot(M_EF)
    Iteration = 1
    V = np.zeros((6,1))
    # Iteration algorithm
    while LA.norm(T - T_desired, 'fro') > Tolerance:
        K = logm(T_desired.dot(Tinv(T)))
        # print(T_desired)
        V[0:3,0] = [K[2,1], K[0,2], K[1,0]]
        V[3:6,0] = K[0:3,3]
        delta_q = k * LA.pinv(Jac_s(Screws, q_iteration)).dot(V)
        q_iteration = q_iteration + delta_q
        T = Screws_to_SE3(Screws, q_iteration).dot(M_EF)
        Iteration = Iteration + 1
        if Iteration > 500:
            break

    # Simplify results' value to [0,2*pi)
    # InvKinematics2 is different to InvKInematics with this part 
    q_iteration = np.mod(q_iteration,2 * np.pi)

    return q_iteration

def ModifyingJointValue(q_init, q_final):
    for i in range(6):
        if np.abs(q_final[i]-q_init[i]) > np.pi :
            q_final[i] = q_final[i] - 2 * np.pi
        else:
            pass

    return q_final

def Joint_Limit_Check(ub,lb,joint):
    for i in range(np.size(joint)):

        if (joint[i]<lb[i]):
            n=np.int((lb[i]-joint[i])/(2*np.pi))+1
            joint[i]=joint[i]+2*np.pi*n

        elif lb[i] <= joint[i] <= ub[i]:
            pass

        else:
            n=np.int((joint[i]-ub[i])/(2*np.pi))+1
            joint[i]=joint[i]-2*n*np.pi

    for i in range(np.size(joint)):
        if lb[i] <= joint[i] <= ub[i]:
            pass
        else:
            print("joint is out of the limit")

    return joint

# def Projector(T,V_b,T_d,a,b,B_a,B_v,delta_t):
#     # T: current SE(3)
#     # V_b: current body velocity
#     # T_d: desired SE(3)
#     # a,b: metric weight
#     # B_a: radious of accer
#     # B_v: radious of vel
#     # delta_t

#     G=np.eye(6)
#     G[0:3,0:3]=a*np.eye(3)
#     G[3:6,3:6]=b*np.eye(3)

#     v=SE3_to_OmegaV(Tinv(T).dot(T_d))/delta_t

#     if np.transpose(V_b-v).dot(G).dot(V_b-v) < B_a*B_a:
#         v_a=v
#     else:
#         v_a=V_b+B_a*(v-V_b)/np.sqrt(np.transpose(v-V_b).dot(G).dot(v-V_b))

#     if np.transpose(v).dot(G).dot(v) < B_v*B_v:
#         v_ba=v_a
#     else:
#         v_ba=B_v*v_a/np.sqrt(np.transpose(v).dot(G).dot(v))
#     T_projection=T.dot(Screw_to_SE3(v_ba, delta_t))
#     return T_projection

def distance_btw_SE3(T1 , T2 , a, b):
    T_1=T1.reshape(4,4)
    T_2=T2.reshape(4,4)
    R1=T_1[0:3,0:3]
    R2=T_2[0:3,0:3]
    p1=T_1[0:3,3:4]
    p2=T_2[0:3,3:4]
    d_p1p2 = LA.norm(p1-p2)
    d_R1R2 = LA.norm(SO3_to_Omega(R1.dot(np.transpose(R2))))
    return a * d_p1p2 + b * d_R1R2

