import numpy as np
import modern_robotics as mr
import math as m

def converter(array1, array2, array3): 
    a1, a2, a3 = (
        np.ndarray.tolist(array1),
        np.ndarray.tolist(array2),
        np.ndarray.tolist(array3),
    )
    a1.extend(a2)
    a1.extend(a3)
    a1 = str(a1)
    a1 = a1.replace("[", "")
    a1 = a1.replace("]", "")

    return a1

def tj_csv(segment, csv_file, gripper_state, file_operation):

    for j in segment:
        x = []
        y = []
        j = np.ndarray.tolist(j)
        j.pop(3)
        for k in j:
            x.append(k.pop(3))
            y.append(k)
            ystr = str(y)
            ystr = ystr.replace("[", "")
            ystr = ystr.replace("]", "")
            xstr = str(x)
            xstr = xstr.replace("[", "")
            xstr = xstr.replace("]", "")
        string = ystr + "," + xstr
        traj_file = open(csv_file, file_operation)
        traj_file.write(string + "," + str(gripper_state) + "\n")
        traj_file.close()

    return None


def csv_to_ndarray(csv_input):
    final_array = []
    split_flat_mat = csv_input.split(",")
    split_flat_mat.pop(12)
    split_flat_mat.append("0")
    split_flat_mat = [float(i) for i in split_flat_mat]
    element1 = split_flat_mat[0:3]
    element1.append(split_flat_mat[9])
    final_array.append(element1)
    element2 = split_flat_mat[3:6]
    element2.append(split_flat_mat[10])
    final_array.append(element2)
    element3 = split_flat_mat[6:9]
    element3.append(split_flat_mat[11])
    final_array.append(element3)
    final_array.append([0, 0, 0, 1])
    final_array = np.array(final_array)
    return final_array


def grip_state_change(csv_file, is_open):
    if is_open is True:
        file = open(csv_file, "r")
        last_line = file.readlines()[-1]
        last_line = last_line.replace(last_line[len(last_line) - 2], "1", 1)
        file.close()
        file = open(csv_file, "a")
        file.write(last_line)
        file.close()
    else:
        file = open(csv_file, "r")
        last_line = file.readlines()[-1]
        last_line = last_line.replace(last_line[len(last_line) - 2], "0", 1)
        file.close()
        file = open(csv_file, "a")
        file.write(last_line)
        file.close()

    return None

def NextStep(vector_12, vector_9,gripper_state,write_file):
    
    timestep = 0.01
    l, w,r = 0.47 / 2, 0.3 / 2,0.0475
    new_joint_angles_0 = np.array(vector_12[3:8]).T + (
        np.array([vector_9[4:9]]) * timestep
    )
    new_wheel_angles_0 = np.array(vector_12[8 : len(vector_12)]).T + (
        np.array([vector_9[0 : 4]]) * timestep
    )
    F = np.array(
        [
            [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
            [1, 1, 1, 1],
            [-1, 1, -1, 1],
        ] 
    )
    F=r/4*F
    Twist = np.matmul(F, np.array(vector_9[0 : 4]))
    omg, vx, vy = Twist
    config_0 = np.array(vector_12[0:3]).T
    if np.isnan(omg):
        omg = 0
    if int(omg) is not 0:
        del_q = np.array([omg, ((vx * m.sin(omg)) + (vy * (m.cos(omg) - 1))) / omg,((vy * m.sin(omg)) + (vx * (-m.cos(omg) + 1))) / omg]).T /100
    else:
        del_q = np.array([omg, vx, vy]).T /100

    phi_k=np.ndarray.tolist(config_0)[0]
    del_q = np.matmul( np.array([[1, 0, 0], [0, m.cos(phi_k), -m.sin(phi_k)], [0, m.sin(phi_k), m.cos(phi_k)]]),del_q) 
    file = open("Milestone3.csv", "a")
    new_joint_angles = new_joint_angles_0 + (np.array([vector_9[4:9]]) * timestep)
    new_wheel_angles = new_wheel_angles_0 + (
        np.array([vector_9[0:4]]) * timestep
    )
    config = config_0 + del_q

    new_wheel_angles[abs(new_wheel_angles) < 1e-5] = 0
    new_joint_angles[abs(new_joint_angles) < 1e-5] = 0
    config[abs(config) < 1e-5] = 0
    
    if write_file:
        file.write(converter(config, new_joint_angles, new_wheel_angles) +"," + str(gripper_state) + "\n")
    
    '''#this block of code was used to generate the csv file for Milestone  1
    i = 1
    while i < 100:
        phi_k=np.ndarray.tolist(config)[0]
        del_q = np.matmul( np.array([[1, 0, 0], [0, m.cos(phi_k), -m.sin(phi_k)], [0, m.sin(phi_k), m.cos(phi_k)]]),del_q)
        new_wheel_angles =new_wheel_angles + (np.array([vector_9[5 : len(vector_9)]]) * timestep)
        new_joint_angles = new_joint_angles + (np.array([vector_9[0:5]]) * timestep)
        config = config + del_q

        file.write(converter(config, new_joint_angles, new_wheel_angles) + ",0" + "\n")

        i += 1
    '''
    file.close()
    return new_joint_angles[0]

def TrajectoryGenerator(
    initial_ee_config,
    initial_cube_config,
    final_cube_config,
    grasp_config,
    initial_standoff_config,
    final_standoff_config,
    k):

    Tf = 5
    N = Tf * k / 0.01

    """THE FINAL TRAJECTORY IS DIVIDED INTO SEGMENTS """
    file = open("Milestone2.csv", "w")

    """Segment 1 : moving from initial position to the initial-standoff position """
    segment1 = mr.CartesianTrajectory(initial_ee_config, initial_standoff_config, Tf, N, 5)
    tj_csv(segment1, "Milestone2.csv", 0, "a")

    """Segment 2 : moving from initial standoff to grasping position"""
    segment2 = mr.CartesianTrajectory(initial_standoff_config, grasp_config, Tf, N, 5)
    tj_csv(segment2, "Milestone2.csv", 0, "a")

    """Segment3 : closing the gripper"""
    grip_state_change("Milestone2.csv", True)

    """Segment4 : moving back to standoff position"""
    Segment4 = mr.CartesianTrajectory(grasp_config, initial_standoff_config, Tf, N, 5)
    tj_csv(Segment4, "Milestone2.csv", 1, "a")

    """Segment5 : moving to final standoff config"""
    Segment5 = mr.CartesianTrajectory(
        initial_standoff_config, final_standoff_config, Tf, N, 5
    )
    tj_csv(Segment5, "Milestone2.csv", 1, "a")

    """Segment6 : moving cube to its final config"""
    segment6 = mr.CartesianTrajectory(final_standoff_config, final_cube_config, Tf, N, 5)
    tj_csv(segment6, "Milestone2.csv", 1, "a")

    """Segment7: Opening the gripper"""
    grip_state_change("Milestone2.csv", False)

    """Segment8 : Back to final standoff config"""
    Segment8 = mr.CartesianTrajectory(final_cube_config, final_standoff_config, Tf, N, 5)
    tj_csv(Segment8, "Milestone2.csv", 0, "a")

    file.close()
    return None

def FeedbackControl(current_config, desired_config, next_config, Kp, Ki, timestep):
    dsrd_twist = (
        mr.MatrixLog6(np.matmul(mr.TransInv(desired_config), next_config))
        * 1
        / timestep
    )
    dsrd_twist = mr.se3ToVec(dsrd_twist)
    adjoint = mr.Adjoint(np.matmul(mr.TransInv(current_config), desired_config))
    err_config = mr.MatrixLog6(np.matmul(mr.TransInv(current_config), desired_config))
    V = mr.se3ToVec(err_config)
    #err_integral = V * timestep
    twist = np.matmul(adjoint, dsrd_twist) + np.matmul(Kp, V) + np.matmul(Ki, V)
    err_file = open("ERROR.csv","a")
    twist_for_file = np.ndarray.tolist(twist)
    twist_for_file= str(twist_for_file)
    twist_for_file = twist_for_file.replace("[", "")
    twist_for_file = twist_for_file.replace("]", "")
    err_file.write(twist_for_file + "\n")
    return twist

def last_line(csv_file):
    file = open(csv_file,"r")
    reader = file.readlines()
    last_line = reader[-1]
    last_line = last_line.split(",")
    last_line = [float(i) for i in last_line]
    return last_line
   
def T0_e(last_line,home_cfg,Blist):
    theta_list=last_line[3:8]
    theta_list = np.array(theta_list).T
    T0e= mr.FKinBody(home_cfg,Blist,theta_list)
    return T0e

def act_curr_config(T0E,last_line):

    config = last_line[0:3]
    phi,x,y = config[0],config[1],config[2]
    Tsb = np.array([[m.cos(phi), -m.sin(phi), 0, x],[m.sin(phi), m.cos(phi), 0, y],[0, 0, 1, 0.0963],[0, 0, 0, 1]])
    Tb0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]]) 
    Tse= Tsb @ Tb0 @ T0E
    return Tse   

def jacobian(last_line,B_list,Home_config):
    l, w,r = 0.47 / 2, 0.3 / 2,0.0475
    F6 = np.array([[0,0,0,0],[0,0,0,0],[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],[1, 1, 1, 1],[-1, 1, -1, 1],[0,0,0,0]])
    F6=F6*r/4  
    T0e=T0_e(last_line,Home_config,B_list)
    Tb0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])
    J_base = mr.Adjoint(mr.TransInv(T0e) @ mr.TransInv(Tb0)) @F6
    theta_list=np.array(last_line[3:8]).T
    J_body= mr.JacobianBody(B_list,theta_list)
    Je= np.concatenate((J_base, J_body),1)

    
    return Je


def csv_curr_cfg(csv_file):
    file = open(csv_file,"r")
    f_r = file.readlines()
    last_cfg = f_r[-1]
    last_cfg=last_cfg.split(",")
    last_cfg.pop(12)
    last_cfg= [float(i) for i in last_cfg]
    return last_cfg

def test_joint_limits(config_vector,gripper_state, Jacobian_matrix,err_twist):
    Jacobian_matrix_inv = np.linalg.pinv(Jacobian_matrix)
    Jacobian_matrix_inv[abs(Jacobian_matrix_inv) < 1e-3] = 0
    control_vector =np.ndarray.tolist( Jacobian_matrix_inv @ err_twist)
    joint_angles = NextStep(config_vector,control_vector,gripper_state,False)
    for i in range(4):
        if i is 0:
            if not  -2.93 <= joint_angles[i] <= 0:
                Jacobian_matrix[:,i+4] = 0
       # elif i is 1:
         #   if not  - <= joint_angles[i] <= 1.3:
           #     Jacobian_matrix[:,i+4] = 0
             
        elif i is 2 :
            if not -9 <= joint_angles[i] <= -0.2:
                Jacobian_matrix[:,i+4] = 0
        elif i is 3 :
            if not  -8 <= joint_angles[i] <= -0.2 :
                Jacobian_matrix[:,i+4] = 0
                
    
    Jacobian_matrix_inv = np.linalg.pinv(Jacobian_matrix)
    Jacobian_matrix_inv[abs(Jacobian_matrix_inv) < 1e-3] = 0
    control_vector = np.ndarray.tolist(Jacobian_matrix_inv @ err_twist)
    NextStep(config_vector,control_vector,gripper_state,True)
    return None


if __name__ == "__main__":

    """Initialization of values"""
    B1=np.array([0,0,1,0,0.033,0])
    B2=np.array([0,-1,0,-0.5076,0,0])
    B3=np.array([0,-1,0,-0.3526,0,0])
    B4=np.array([0,-1,0,-0.2176,0,0])
    B5=np.array([0,0,1,0,0,0])
    B_list = np.vstack((B1,B2,B3,B4,B5)).T
    phi, x, y, z = 0, 0, 0, 0.0963  # input values for chassis config
    j1, j2, j3, j4, j5 = 0.0, -0.265, -0.556, -0.3, 0.214  # arm joint angles
    w1, w2, w3, w4 = 0, 0, 0, 0  # wheel angles
    Tsb = np.array([[m.cos(phi), -m.sin(phi), 0, x],[m.sin(phi), m.cos(phi), 0, y],[0, 0, 1, 0.0963],[0, 0, 0, 1]])
    Tb0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]]) 
    M0e_home = np.array(
        [[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]]
    )  # home config, e-e wrt to arm base
    Tse = np.array([[0.425,-0.092,0.901,0.535],[0.212,0.977,0.0,0.00],[-0.880,0.191,0.435,0.582],[0,0,0,1]])

    Tscini = np.array(
        [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]
    )  # initial cube configuration
    Tscdes = np.array(
       [[0.044, 0.998, -0.041, 0], [0.726, -0.060, -0.685, -1], [-0.686, 0, -0.727, 0.025], [0, 0, 0, 1]]
    )  # final cube configuration  z = 0.025

    T_grasp = np.array([[-0.888,0,0.461,1],[0,1,0,0],[-0.461,0,-0.888,0.025],[0,0,0,1]])

    Tscini_standoff = np.array([[-0.888,0,0.461,1],[0,1,0,0],[-0.461,0,-0.888,0.070],[0,0,0,1]])

    Tscdes_standoff =   np.array(
        [[0.044, 0.998, -0.041, 0], [0.726, -0.060, -0.685, -1], [-0.686, 0, -0.727, 0.070], [0, 0, 0, 1]]
    ) 
    #TrajectoryGenerator(Tse,Tscini,Tscdes,T_grasp,Tscini_standoff,Tscdes_standoff,1)
    err_file= open("ERROR.csv","w")
    KP = np.eye(6) * 2
    KI = np.eye(6) * 2
    cfg = [phi,x,y,j1,j2,j3,j4,j5,w1,w2,w3,w4]
    controls = [ 0,0,0,0,0,0,0,0,0]
    cfg_file = open("Milestone3.csv","w")
    NextStep(cfg,controls,0,True)
    curr_cfg = act_curr_config(T0_e(last_line("Milestone3.csv"),M0e_home,B_list),last_line("Milestone3.csv"))
    traj_file = open("Milestone2.csv","r")
    tj_r= traj_file.readlines()                                               #csv_curr_cfg("Milestone3.csv")
    dsrd_cfg = tj_r[0]
    g_s=dsrd_cfg[-2]
    dsrd_cfg = csv_to_ndarray(dsrd_cfg)
    nxt_cfg = tj_r[1]
    nxt_cfg = csv_to_ndarray(nxt_cfg)
    J = jacobian(last_line("Milestone3.csv"),B_list,M0e_home)
    test_joint_limits(csv_curr_cfg("Milestone3.csv"),g_s,J,FeedbackControl(curr_cfg,dsrd_cfg,nxt_cfg,KP,KI,0.01))
    curr_cfg = act_curr_config(T0_e(last_line("Milestone3.csv"),M0e_home,B_list),last_line("Milestone3.csv"))
    

    i=1

 
    while i < 3001: 
        curr_cfg = act_curr_config(T0_e(last_line("Milestone3.csv"),M0e_home,B_list),last_line("Milestone3.csv"))
        traj_file = open("Milestone2.csv","r")
        tj_r= traj_file.readlines()
        dsrd_cfg = tj_r[i]
        g_s=dsrd_cfg[-2]
        dsrd_cfg = csv_to_ndarray(dsrd_cfg)
        nxt_cfg = tj_r[i+1]
        nxt_cfg = csv_to_ndarray(nxt_cfg)
        J = jacobian(last_line("Milestone3.csv"),B_list,M0e_home)
        test_joint_limits(csv_curr_cfg("Milestone3.csv"),g_s,J,FeedbackControl(curr_cfg,dsrd_cfg,nxt_cfg,KP,KI,0.01))    
        i+=1

