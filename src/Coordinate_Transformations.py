import numpy as np
import tf

def Transforms():
    #NOTE: Tune dimensions of the bounding box
    l = 0.44*1.2
    w = 0.44*1.2
    h = 0.2*1.2
    #NOTE: Manual Calibration: camera->image transformation
    Angle_x = np.pi*89.2/180.
    Angle_y = np.pi*90./180.
    Angle_z = np.pi*90./180.
    #NOTE: Manual Calibration: Center offset of Quad rigid body
    translation_offset = np.array([[0],[0],[0.03]])



    half_length = l/2.
    half_width  = w/2.
    half_height = h/2.
    quad_dim = [l, w, h, half_length, half_width, half_height]

    R_deltax = np.array([[ 1.              , 0.              , 0.               ],
                         [ 0.              , np.cos(Angle_x) ,-np.sin(Angle_x)  ],
                         [ 0.              , np.sin(Angle_x) , np.cos(Angle_x)  ]])
    R_deltay = np.array([[ np.cos(Angle_y) , 0.              , np.sin(Angle_y)  ],
                         [ 0.              , 1.              , 0                ],
                         [-np.sin(Angle_y) , 0.              , np.cos(Angle_y)  ]])
    R_deltaz = np.array([[ np.cos(Angle_z) ,-np.sin(Angle_z) , 0.               ],
                         [ np.sin(Angle_z) , np.cos(Angle_z) , 0.               ],
                         [ 0.              , 0.              , 1.               ]])
    R_delta = np.dot(R_deltax,R_deltaz)
    C_delta = np.array([[0.026],[0.03],[-0.019]])
    t_cam_to_image = np.dot(R_delta.T,-C_delta)
    T_cam_to_image = np.eye(4)
    T_cam_to_image[0:3,0:3] = R_delta[0:3,0:3]
    T_cam_to_image[0:3,3] = t_cam_to_image[0:,0]

    K_image_to_pix = np.array([\
                         [617.2744140625, 0.0,              324.1011047363281, 0.],
                         [0.0,            617.335693359375, 241.5790557861328, 0.],
                         [0.0,            0.0,              1.0              , 0.]]) #Intrinsic Parameters of Camera

    ori_vertex_array = np.array([[ half_length, half_width, half_height, 1.],
                                 [ half_length, half_width,-half_height, 1.],
                                 [ half_length,-half_width,-half_height, 1.],
                                 [ half_length,-half_width, half_height, 1.],
                                 [-half_length,-half_width, half_height, 1.],
                                 [-half_length,-half_width,-half_height, 1.],
                                 [-half_length, half_width,-half_height, 1.],
                                 [-half_length, half_width, half_height, 1.]])

    return quad_dim, K_image_to_pix, T_cam_to_image, R_delta, C_delta, \
            ori_vertex_array, translation_offset

def Matrix_from_Pose(msg_pose,offset=0.):
    quaternion = (
        msg_pose.pose.orientation.x,
        msg_pose.pose.orientation.y,
        msg_pose.pose.orientation.z,
        msg_pose.pose.orientation.w)
    R = tf.transformations.quaternion_matrix(quaternion)
    C = np.array(
        [[msg_pose.pose.position.x],
         [msg_pose.pose.position.y],
         [msg_pose.pose.position.z]])
    T = np.eye(4)
    T[:3,:3] = R[:3,:3]
    T[:3, 3:] = C + offset
    return T

#Given poses --> return rectangle coordinates.
def vertex_coordinates(msg_pose_Quad,msg_pose_RealSense):
    quad_dim, K_image_to_pix, T_cam_to_image, R_delta, C_delta, \
            ori_vertex_array, translation_offset = Transforms()
    l, w, h, half_length, half_width, half_height = quad_dim

    #Homogeneous Transformation: Quad -> World frame
    T_quad_to_world = Matrix_from_Pose(msg_pose_Quad,translation_offset)

    #Homogeneous Transformation: World -> Camera frame
    T_cam_to_world = Matrix_from_Pose(msg_pose_RealSense)
    T_world_to_cam = np.linalg.inv(T_cam_to_world)

    vertex = np.zeros((4,1))
    vertex_pixels_array = np.zeros((8,2))
    for index in range(0,8):
        vertex[:4,0] = ori_vertex_array[index] #column vector, (x,y,z,1)
        vertex = np.matmul(T_quad_to_world ,vertex)
        vertex = np.matmul(T_world_to_cam  ,vertex)
        vertex = np.matmul(T_cam_to_image  ,vertex)
        vertex_pixels = np.matmul(K_image_to_pix ,vertex)
        vertex_pixels = np.true_divide(vertex_pixels ,vertex_pixels[2][0])
        vertex_pixels_array[index,:2] = vertex_pixels[:2,0]

    vertex_pixels_array = np.squeeze(vertex_pixels_array)
    max_coords = np.amax(vertex_pixels_array,axis=0)
    min_coords = np.amin(vertex_pixels_array,axis=0)

    return vertex_pixels_array, max_coords, min_coords

def vertex_coordinates_EKF(quad_pose,quad_orientation,msg_pose_RealSense):
    quad_dim, K_image_to_pix, T_cam_to_image, R_delta, C_delta, \
            ori_vertex_array, translation_offset = Transforms()
    l, w, h, half_length, half_width, half_height = quad_dim

    #Homogeneous Transformation: World -> Camera frame
    T_cam_to_world = Matrix_from_Pose(msg_pose_RealSense)
    T_world_to_cam = np.linalg.inv(T_cam_to_world)

    #Non-homogeneous Transformation: Quad -> World frame
    R_Quad = tf.transformations.euler_matrix(quad_orientation[0][0],quad_orientation[1][0],quad_orientation[2][0])
    R_Quad = R_Quad[:3,:3]
    #To find the Quad -> World frame translation vector, need to derive from the estimated pose (which is in image frame)
    t_delta = np.dot(R_delta,-C_delta)
    t_camera = T_world_to_cam[:3,3:]

    C_Quad = quad_pose[:3,0:] - t_delta - np.dot(R_delta,t_camera)
    C_Quad2 = np.dot(T_world_to_cam[:3,:3].T,R_delta.T)
    C_Quad = np.dot(C_Quad2,C_Quad)
    #This (below) is the equivalent homogeneous form.
    T_quad_to_world = np.eye(4)
    T_quad_to_world[:3,:3] = R_Quad
    T_quad_to_world[:3,3:] = C_Quad

    vertex = np.zeros((4,1))
    vertex_pixels_array = np.zeros((8,2))
    for index in range(0,8):
        vertex[:4,0] = ori_vertex_array[index] #column vector, (x,y,z,1)
        vertex = np.matmul(T_quad_to_world ,vertex)
        vertex = np.matmul(T_world_to_cam  ,vertex)
        vertex = np.matmul(T_cam_to_image  ,vertex)
        vertex_pixels = np.matmul(K_image_to_pix ,vertex)
        vertex_pixels = np.true_divide(vertex_pixels ,vertex_pixels[2][0])
        vertex_pixels_array[index,:2] = vertex_pixels[:2,0]

    vertex_pixels_array = np.squeeze(vertex_pixels_array)

    return vertex_pixels_array

def ground_truth_center(msg_pose_Quad,msg_pose_RealSense):

    quad_dim, K_image_to_pix, T_cam_to_image, R_delta, C_delta, \
            ori_vertex_array, translation_offset = Transforms()
    l, w, h, half_length, half_width, half_height = quad_dim

    #Homogeneous Transformation: Quad -> World frame
    T_quad_to_world = Matrix_from_Pose(msg_pose_Quad,translation_offset)

    #Homogeneous Transformation: World -> Camera frame
    T_cam_to_world = Matrix_from_Pose(msg_pose_RealSense)
    T_world_to_cam = np.linalg.inv(T_cam_to_world)

    quad_position = np.zeros((4,1))
    quad_position[3] = 1.
    quad_position = np.matmul(T_quad_to_world ,quad_position)
    quad_position = np.matmul(T_world_to_cam  ,quad_position)
    quad_position = np.matmul(T_cam_to_image  ,quad_position)

    R = np.eye(4)
    R[:3,:3] = T_quad_to_world[:3,:3]
    ai,aj,ak = tf.transformations.euler_from_matrix(R)
    quad_orientation = np.array([[ai,aj,ak]]).T
    return quad_position, quad_orientation
