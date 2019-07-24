import numpy as np
from Coordinate_Transformations import Transforms

quad_dim, K_sensor_to_pix, T_cam_to_sensor, _, _, _, _ = Transforms()
alpha_f_x = np.squeeze(K_sensor_to_pix[0,0])
alpha_f_y = np.squeeze(K_sensor_to_pix[1,1]) #alpha*f of the y-axis
K = K_sensor_to_pix[:3,:3]

l, w, h, half_length, half_width, half_height = quad_dim

height = h #m
width = w #m

R_t = 0.1*np.eye(3)
Q_t = 0.1*np.eye(4)
Q_t[2][2] = 50000
Q_t[3][3] = 50000

def return_u_t(quad_pose_now,quad_pose_old,dt):
    u_t = np.array([[0.,0.,0.]]).T
    return u_t

def g(u_t,mu_t0):
    mu_bar_t = mu_t0 + u_t
    return mu_bar_t

def G(u_t,mu_t0):
    return np.eye(3)

def h(mu_bar_t):
    z_bar_t = mu_bar_t
    z_bar_t_2 = alpha_f_x*width/np.sqrt(z_bar_t[0]**2 + z_bar_t[1]**2 + z_bar_t[2]**2)
    z_bar_t_3 = alpha_f_y*height/np.sqrt(z_bar_t[0]**2 + z_bar_t[1]**2 + z_bar_t[2]**2)
    z_bar_t = np.matmul(K,z_bar_t)
    for i in range(2):
        z_bar_t[i][0] = z_bar_t[i][0]/z_bar_t[2][0]
    z_bar_t = np.append(z_bar_t[0:2],np.array([z_bar_t_2]),axis=0)
    z_bar_t = np.append(z_bar_t     ,np.array([z_bar_t_3]),axis=0)
    return z_bar_t #[[x_center_pixel,y_center_pixel,x_width_pixel,y_height_pixel]].T

def H(mu_bar_t):
    x = mu_bar_t[0][0]
    y = mu_bar_t[1][0]
    z = mu_bar_t[2][0]
    J_xyz1 = np.array([\
                [1./z, 0.  , -x/(z**2)],\
                [0.  , 1./z, -y/(z**2)],\
                [0.  , 0.  , 0.       ]])
    temp = np.array([\
                [1.,0.,0.],\
                [0.,1.,0.]])
    J = np.matmul(K,J_xyz1)
    J = np.matmul(temp,J)
    s_h = alpha_f_y*height/((x**2+y**2+z**2)**1.5)
    s_w = alpha_f_x*width/((x**2+y**2+z**2)**1.5)
    J_s_w = np.array([[-x*s_w, -y*s_w, -z*s_w]])
    J_s_h = np.array([[-x*s_h, -y*s_h, -z*s_h]])
    J = np.append(J,J_s_w,axis=0)
    J = np.append(J,J_s_h,axis=0)
    return J


def EKF_v1(mu_t0,Sig_t0,u_t,z_t,msg_pose_RealSense):
    mu_bar_t = g(u_t,mu_t0)
    G_mat = G(u_t,mu_t0)
    Sig_bar_t = np.matmul(np.matmul(G_mat,Sig_t0),G_mat.T) + R_t
    H_mat = H(mu_bar_t)
    K_t_1 = np.matmul(Sig_bar_t,H_mat.T)

    K_t_2 = np.matmul(np.matmul(H_mat,Sig_bar_t),H_mat.T) + Q_t
    K_t = np.matmul(K_t_1,np.linalg.inv(K_t_2))

    temp = z_t-h(mu_bar_t)

    mu_t = mu_bar_t + np.matmul(K_t,(z_t - h(mu_bar_t)))
    Sig_t = np.matmul((np.eye(3) - np.matmul(K_t,H_mat)),Sig_bar_t)

    return mu_t, Sig_t

def EKF_v1_noMeasureUpdate(mu_t0,Sig_t0,u_t):

    mu_bar_t = g(u_t,mu_t0)
    G_mat = G(u_t,mu_t0)
    Sig_bar_t = np.matmul(np.matmul(G_mat,Sig_t0),G_mat.T) + R_t

    return mu_bar_t, Sig_bar_t
