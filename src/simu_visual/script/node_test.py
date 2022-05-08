#!/usr/bin/env python
import delta_define
import codos_tm1_adams
import rospy
import std_msgs.msg
import trans_rot_ls_adams
import linear_speed_f_adams
import delta_kinematics_t1m_adams

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from simu_visual.msg import linear_speed_xyz

pi = delta_define.pi()
dtr = delta_define.dtr()
mmtm = delta_define.mmtm()

def node():
    #######  callback_linear_speed_xyz(data)  #######
    global call_xo_2
    global call_yo_2
    global call_zo_2

    global call_xf_2
    global call_yf_2
    global call_zf_2

    global call_vmax_2
    global call_amax_2
    global gripper

    global permiso_2

    ######  Home Node  ######
    rospy.init_node("node_c", anonymous=False)
    rate = rospy.Rate(7.8125)  # Hz

    #######  Subscriber  ##################
    rospy.Subscriber("input_ls_final", linear_speed_xyz, callback_linear_speed_xyz) # Topics

    #######  Publisher  ##################
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    pub_1 = rospy.Publisher("status_delta", String, queue_size=10)

    juntas = JointState()

    #######  Variables verify incoming message  #######
    call_pas1_2 = 50
    call_pas2_2 = 150

    permiso_2 = False

    while not rospy.is_shutdown():
        if ((permiso_2 == True) and ((call_xo_2 != call_xf_2) or (call_yo_2 != call_yf_2) or (call_zo_2 != call_zf_2)) and ((call_vmax_2 > 0) and (call_amax_2 > 0))):

            rospy.loginfo("  v_max= " + str(call_vmax_2) + "  a_max=" + str(call_amax_2))

            #######   End and start point rotation  #######
            results_2 = trans_rot_ls_adams.path_linear_speed(call_xo_2, call_yo_2, call_zo_2, call_xf_2, call_yf_2, call_zf_2)
                                                            #   x0         y0        z0          x1         y1         z1

            #######   Trapezoidal velocity profile  #######
            results_3 = linear_speed_f_adams.ls_v_a_total(0, results_2[0], call_vmax_2, call_amax_2, call_pas1_2, call_pas2_2)
                                                        #p0       p1            v_max         a_max         50          150

            ########   Reverse rotation end point, start point and trajectory #######
            results_4 = trans_rot_ls_adams.path_linear_speed_inv(results_3[0],results_3[1],results_3[2],results_3[3],results_2[1],results_2[2],results_2[3],results_2[4],results_2[5])
                                                            #  mytempo           pos         vel          acel        rot_z         rot_y     theta_y         theta_z   rot_tras
            
            #######   Save in Matrix Path Cartesian space XYZ  #######
            res_1=[results_4[0], results_4[1], results_4[4], results_4[7], results_4[2], results_4[5], results_4[8], results_4[3], results_4[6], results_4[9]]
                #   mytempo        x            vel_x         acel_x           y          vel_y         acel_y         z          vel_z           acel_z
                #     0            1              2             3              4            5              6           7            8                9
            
            #######   Inverse kinematics  #######
            res_2 = delta_kinematics_t1m_adams.inverse_m(res_1[1], res_1[4], res_1[7])

            #######  Rviz Trajectory Visualization #######
            rospy.loginfo("Creating Linear Path RVIZ!")
            largo = len(res_1[0])

            # Starting point
            juntas = angulos_eulerianos(1, res_1[1][0], res_1[4][0], res_1[7][0], res_2[0][0], res_2[1][0], res_2[2][0], gripper)
            delta = 1
            rospy.sleep(delta)
            pub.publish(juntas)

            for i in range(1, largo):
                #juntas = angulos_eulerianos(t_m[i], x_m[i], y_m[i], z_m[i], theta1_m[i], theta2_m[i], theta3_m[i])
                juntas = angulos_eulerianos(res_1[0][i] * 10.0, res_1[1][i], res_1[4][i], res_1[7][i], res_2[0][i], res_2[1][i], res_2[2][i], gripper)
                delta = res_1[0][i] * 10.0 - res_1[0][i-1] * 10.0
                rospy.sleep(delta)
                pub.publish(juntas)

            #######  Reset variable incoming message  ######
            status_delta = "DONE"
            pub_1.publish(status_delta)

            permiso_2 = False

        rate.sleep()

    return

def callback_linear_speed_xyz(data):
    global permiso_2

    global call_xo_2
    global call_yo_2
    global call_zo_2

    global call_xf_2
    global call_yf_2
    global call_zf_2

    global call_vmax_2
    global call_amax_2
    global gripper

    permiso_2 = True

    call_xo_2 = data.xo
    call_yo_2 = data.yo
    call_zo_2 = data.zo

    call_xf_2 = data.xf
    call_yf_2 = data.yf
    call_zf_2 = data.zf

    call_vmax_2 = data.vmax
    call_amax_2 = data.amax
    gripper = data.gripper

# |--------------------------------------|
# |----------- Joint Angles -------------| 
# |--------------------------------------|
def angulos_eulerianos(ti, xi, yi, zi, th1, th2, th3, gripper):
    # Rviz interior angles in Radians
    joint = JointState()
    punto = [-yi * mmtm, -xi * mmtm, -zi * mmtm]

    c2 = codos_tm1_adams.punto_codo(th2)
    p2 = codos_tm1_adams.punto_ee(punto, 2)
    [a2_a, a2_b] = codos_tm1_adams.angulos_codo(c2, p2, 2)

    c3 = codos_tm1_adams.punto_codo(th3)
    c3 = codos_tm1_adams.rotacion120(c3)
    p3 = codos_tm1_adams.punto_ee(punto, 3)
    [a3_a, a3_b] = codos_tm1_adams.angulos_codo(c3, p3, 3)

    c1 = codos_tm1_adams.punto_codo(th1)
    c1 = codos_tm1_adams.rotacion120(c1)
    c1 = codos_tm1_adams.rotacion120(c1)
    p1 = codos_tm1_adams.punto_ee(punto, 1)
    [a1_a, a1_b] = codos_tm1_adams.angulos_codo(c1, p1, 1)

    # Data to publish in Rviz
    joint.header = std_msgs.msg.Header()
    joint.header.stamp = rospy.Time.now()

    joint.name = ["base_brazo1",    "base_brazo2",  "base_brazo3",
                  "codo1_a",        "codo1_b",
                  "codo2_a",        "codo2_b",
                  "codo3_a",        "codo3_b",
                  "act_x",          "act_y",         "act_z",       "gripper"]

    joint.position = [th1 * dtr, th2 * dtr, th3 * dtr,
                      th1 * dtr + a1_a, a1_b,
                      th2 * dtr + a2_a, a2_b,
                      th3 * dtr + a3_a, a3_b,
                      xi / 1000, yi / 1000, zi / 1000, gripper]

    joint.velocity = []
    joint.effort = []

    return joint

if __name__ == "__main__":
    try:
        node()
    except rospy.ROSInterruptException:
        pass
