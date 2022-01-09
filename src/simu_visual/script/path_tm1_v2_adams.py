#!/usr/bin/env python
import numpy as np
import random
import plot_delta
import rospy
import trans_rot_ls_adams
import linear_speed_f_adams
import delta_kinematics_t1m_adams
import delta_kinematics_Paderborn_tm1_adams
import jacobian_tm1_adams
import jacobian_Paderborn_tm1_v2_adams
import torque_m1_adams
import torque_m1_Paderborn_v2_adams

from simu_visual.msg import matriz_path_ls
from simu_visual.msg import linear_speed_xyz

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

   global call_pas1_2
   global call_pas2_2
   
   global permiso_2
   global id_call_2
   global num_tray_2

   #######  Variables verify incoming message  #######
   permiso_2 = False
   id_call_2 = 0
   id_permiso_2 = 0

   #######  Start ROS node  ##################
   rospy.init_node("torque_metodo_1", anonymous=False)
   rate = rospy.Rate(7.8125)  # Hz

   #######  Publisher  ##################
   pub1 = rospy.Publisher("m_txyzth123", matriz_path_ls, queue_size=10)
   rospy.Subscriber("input_ls_final", linear_speed_xyz, callback_linear_speed_xyz) # Topics

   while not rospy.is_shutdown():

      if ((permiso_2 == True) and (id_call_2 != id_permiso_2) and ((call_xo_2 != call_xf_2) or (call_yo_2 != call_yf_2) or (call_zo_2 != call_zf_2)) and ((call_vmax_2 > 0) and (call_amax_2 > 0))):

         rospy.loginfo("ID: " + str(id_call_2) + "  v_max= " + str(call_vmax_2) + "  a_max=" + str(call_amax_2))

         #######   End and start point rotation  #######
         results_2 = trans_rot_ls_adams.path_linear_speed(call_xo_2, call_yo_2, call_zo_2, call_xf_2, call_yf_2, call_zf_2)

         #######   Trapezoidal velocity profile  #######
         results_3 = linear_speed_f_adams.ls_v_a_total(0, results_2[0], call_vmax_2, call_amax_2, call_pas1_2, call_pas2_2)

         ########   Reverse rotation end point, start point and trajectory #######
         results_4 = trans_rot_ls_adams.path_linear_speed_inv(results_3[0],
	      results_3[1], results_3[2], results_3[3],
	      results_2[1], results_2[2],
	      results_2[3], results_2[4],
	      results_2[5])

         #######   Save in Matrix Path Cartesian space XYZ  #######
         res_1 = [results_4[0],
			results_4[1], results_4[4], results_4[7],
			results_4[2], results_4[5], results_4[8],
			results_4[3], results_4[6], results_4[9]]

         # ************************************
         # ******  Method A *******************
         # ************************************

         #######   Inverse kinematics  #######
         res_2 = delta_kinematics_t1m_adams.inverse_m(res_1[1], res_1[4], res_1[7])

         #######  Rviz Trajectory Visualization #######
         enviar = matriz_path_ls()
         enviar.x = res_1[1]
         enviar.y = res_1[4]
         enviar.z = res_1[7]
         enviar.th1 = res_2[0]
         enviar.th2 = res_2[1]
         enviar.th3 = res_2[2]
         enviar.tiempo = res_1[0] * 10.0
         enviar.permiso = bool(1)
         enviar.id_call = random.randrange(10)
         pub1.publish(enviar)

         permiso_2 = False
         id_permiso_2 = id_call_2

      rate.sleep()
   return


def callback_linear_speed_xyz(data):
   global permiso_2
   global id_call_2
   global call_xo_2
   global call_yo_2
   global call_zo_2
   global call_xf_2
   global call_yf_2
   global call_zf_2
   global call_vmax_2
   global call_amax_2
   global call_pas1_2
   global call_pas2_2
   global num_tray_2

   call_xo_2 = data.xo
   call_yo_2 = data.yo
   call_zo_2 = data.zo
   call_xf_2 = data.xf
   call_yf_2 = data.yf
   call_zf_2 = data.zf
   call_vmax_2 = data.vmax
   call_amax_2 = data.amax
   call_pas1_2 = data.paso1
   call_pas2_2 = data.paso2
   permiso_2 = data.ls_fin
   id_call_2 = data.idcall
   num_tray_2 = data.num_tray

if __name__ == "__main__":
   try:
      node()
   except rospy.ROSInterruptException:
      pass
