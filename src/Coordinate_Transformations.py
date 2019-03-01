import numpy as np
import tf

#Given poses --> return rectangle coordinates.
def vertex_coordinates(msg_pose_Quad,msg_pose_RealSense):
    #40x40x20cm
    half_length = 0.2 #m
    half_width = 0.2 #m
    half_height = 0.1 #m
    ori_vertex_array = np.array([[ half_length, half_width, half_height],
                                 [ half_length, half_width,-half_height],
                                 [ half_length,-half_width,-half_height],
                                 [ half_length,-half_width, half_height],
                                 [-half_length,-half_width, half_height],
                                 [-half_length,-half_width,-half_height],
                                 [-half_length, half_width,-half_height],
                                 [-half_length, half_width, half_height]])

    quaternion = (
        msg_pose_Quad.pose.orientation.x,
        msg_pose_Quad.pose.orientation.y,
        msg_pose_Quad.pose.orientation.z,
        msg_pose_Quad.pose.orientation.w)
    R_Quad = tf.transformations.quaternion_matrix(quaternion)
    R_Quad = R_Quad[:3,:3]
    C_Quad = np.array(
        [[msg_pose_Quad.pose.position.x],
         [msg_pose_Quad.pose.position.y],
         [msg_pose_Quad.pose.position.z]])
    C_Quad = C_Quad + np.array([[0],[0],[0]])

    quaternion = (
        msg_pose_RealSense.pose.orientation.x,
        msg_pose_RealSense.pose.orientation.y,
        msg_pose_RealSense.pose.orientation.z,
        msg_pose_RealSense.pose.orientation.w)
    R_camera = tf.transformations.quaternion_matrix(quaternion)
    R_camera = R_camera[:3,:3].T #inverse(transpose) rotation matrix
    C_camera = np.array(
        [[msg_pose_RealSense.pose.position.x],
         [msg_pose_RealSense.pose.position.y],
         [msg_pose_RealSense.pose.position.z]])
    t_camera = np.dot(R_camera,-C_camera)

    K_camera = np.array(
        [[617.2744140625, 0.0,              324.1011047363281],
         [0.0,            617.335693359375, 241.5790557861328],
         [0.0,            0.0,              1.0]]) #Intrinsic Parameters of Camera

    #Tuning
    Angle = np.pi*90./180.
    R_deltax = np.array([[1,0,0],
                        [0,np.cos(Angle),-np.sin(Angle)],
                        [0,np.sin(Angle),np.cos(Angle)]])

    R_deltay = np.array([[-1,0,0],
                        [0,1,0],
                        [0,0,-1]])

    R_delta = np.dot(R_deltay,R_deltax)
    R_delta = R_delta.T
    # R_delta = np.array([[1,0,0],
                      # [0,0,-1],
                      # [0,1,0]])


    C_delta = np.array([[0.03],[-0.026],[-0.019]])
    t_delta = np.dot(R_delta,-C_delta)
    # R_delta = np.array([[-0.99152125,  0.11922477,  0.05168243],
    #                     [-0.01577827,  0.28432205, -0.95859899],
    #                     [-0.1289832,  -0.95128672, -0.28003019]])
    # t_delta = np.array([[ 0.93766943],
    #                     [-0.24947798],
    #                     [ 0.0724625 ]])

    vertex = np.zeros((3,1))
    vertex_pixels_array = np.zeros((8,2))
    for index in range(0,8):
        vertex[:3,0] = ori_vertex_array[index] #column vector, (x,y,z)
        vertex = np.dot(R_Quad,vertex)+C_Quad
        # vertex_pixels = np.dot(R_delta,np.dot(R_camera,vertex)+t_camera)+t_delta
        # vertex_pixels = np.true_divide(vertex_pixels,vertex_pixels[2][0])
        # vertex_pixels = np.dot(K_camera,vertex_pixels)
        vertex_pixels = np.dot(K_camera,np.dot(R_delta,np.dot(R_camera,vertex)+t_camera)+t_delta)
        vertex_pixels = np.true_divide(vertex_pixels,vertex_pixels[2][0])
        vertex_pixels_array[index,:2] = vertex_pixels[:2,0]

    vertex_pixels_array = np.squeeze(vertex_pixels_array)
    max_coords = np.amax(vertex_pixels_array,axis=0)
    min_coords = np.amin(vertex_pixels_array,axis=0)

    return vertex_pixels_array, max_coords, min_coords
