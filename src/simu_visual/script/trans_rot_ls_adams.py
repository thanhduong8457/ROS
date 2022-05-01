import math
import delta_define
import numpy as np
import matplotlib.pyplot as plt

rtd = delta_define.rtd()
mmtm = delta_define.mmtm()
mtmm = delta_define.mtmm()
dtr = delta_define.dtr()

def path_linear_speed(xo, yo, zo, xf, yf, zf):
  ######  International units change  ##########
  xoo = xo * mmtm
  yoo = yo * mmtm
  zoo = zo * mmtm
  xff = xf * mmtm
  yff = yf * mmtm
  zff = zf * mmtm

  res_final_1 = system_linear(xoo, yoo, zoo, xff, yff, zff)

  return res_final_1


# ******************************************
# ****  Rotations XYZ to X'Y'Z '  *****
# ******************************************
def system_linear(xo, yo, zo, xf, yf, zf):  # ,theta_z,theta_y) :
  ######  Creacion de Matrices  ######
  rot_z = np.zeros((3, 3))
  rot_y = np.zeros((3, 3))
  rot_tras = np.zeros((4, 4))

  pf = np.zeros((3, 1))
  pf_trans = np.zeros((4, 1))

  ######  Traslacion  ######
  # Traslacion (Xo,Yo,Zo)
  rot_tras[0, 0] = 1
  rot_tras[1, 1] = 1
  rot_tras[2, 2] = 1
  rot_tras[3, 3] = 1
  rot_tras[0, 3] = -xo
  rot_tras[1, 3] = -yo
  rot_tras[2, 3] = -zo
  # Traslacion (Xf,Yf,Zf)
  pf_trans[0, 0] = xf
  pf_trans[1, 0] = yf
  pf_trans[2, 0] = zf
  pf_trans[3, 0] = 1

  x_trans = (rot_tras).dot(pf_trans)

  ######  Angles theta-y theta-z  ######
  NN = unit_vector(0, 0, 0, x_trans[0, 0], x_trans[1, 0], x_trans[2, 0])
  angles = angulos_rot(NN[0], NN[1], NN[2])
  theta_y = angles[1]
  theta_z = angles[0]

  cos_axisz = math.cos(theta_y)
  sin_axisz = math.sin(theta_y)

  cos_axisy = math.cos(theta_z)
  sin_axisy = math.sin(theta_z)

  ######  Rotacion Z  ######
  # Plano xy ya trasladado
  pf[0, 0] = x_trans[0, 0]  # xf - x0
  pf[1, 0] = x_trans[1, 0]  # yf - y0
  pf[2, 0] = x_trans[2, 0]  # zf - z0
  # Rotacion respecto a eje Z
  rot_z[0, 0] = cos_axisz
  rot_z[0, 1] = sin_axisz
  rot_z[1, 0] = -1 * sin_axisz
  rot_z[1, 1] = cos_axisz
  rot_z[2, 2] = 1

  m_rot1 = rot_z.dot(pf)

  ######  Rotacion Y  ######
  # Rotacion respecto a eje Y
  rot_y[0, 0] = cos_axisy
  rot_y[0, 2] = sin_axisy
  rot_y[1, 1] = 1
  rot_y[2, 0] = -sin_axisy
  rot_y[2, 2] = cos_axisy

  m_rot2 = rot_y.dot(m_rot1)

  return [(m_rot2[0, 0]) * mtmm, rot_z, rot_y, theta_y * rtd, theta_z * rtd, rot_tras]


# ******************************************
# ******  Unit Vector  ***********
# ******************************************
def unit_vector(xo, yo, zo, xf, yf, zf): # this point caculate unit vector from 2 point percific A and B 
 delta_x = xf - xo
 delta_y = yf - yo
 delta_z = zf - zo
 modulo = math.sqrt(((delta_x) ** 2) + ((delta_y) ** 2) + ((delta_z) ** 2)) # caculate modulo of vector
 nx = delta_x / modulo
 ny = delta_y / modulo
 nz = delta_z / modulo
 return [nx, ny, nz]


# ******************************************
# ******  Theta Y ; Theta Z  *********
# ******************************************
def angulos_rot(nx, ny, nz):
  if (nz < 0):
      theta_z = (360 * dtr) + math.asin(nz)
  else:
      theta_z = math.asin(nz)

  if (nx < 0):
      theta_y = (180 * dtr) + (math.atan(ny / nx))
  elif (nx == 0):
      if (ny < 0):
          theta_y = (270 * dtr)
      elif (ny == 0):
          theta_y = 0
      else:
          theta_y = (90 * dtr)
  else:
      if (ny < 0):
          theta_y = (360 * dtr) + (math.atan(ny / nx))
      else:
          theta_y = (math.atan(ny / nx))

  return [theta_z, theta_y]


def path_linear_speed_inv(m_tiempo, m_pos, m_vel, m_acel, rot_z, rot_y, theta_y, theta_z, m_trans):
  ######  Cambiar unidades a SI  ##########
  m_tiempo = m_tiempo[0]
  m_pos = mmtm * m_pos[0]
  m_vel = mmtm * m_vel[0]
  m_acel = mmtm * m_acel[0]
  theta_y = theta_y * dtr
  theta_z = theta_z * dtr

  res_final_2 = system_linear_matrix(m_tiempo, m_pos, m_vel, m_acel, rot_z, rot_y, theta_y, theta_z, m_trans)

  return res_final_2


# ******************************************
# ****  Trayectoria Lineal XYZ  ******
# ******************************************
def system_linear_matrix(m_tiempo, m_pos, m_vel, m_acel, rot_z, rot_y, theta_y, theta_z, m_trans):

  tamano = len(m_tiempo)

  m_pos_x = np.zeros((1, tamano))
  m_pos_y = np.zeros((1, tamano))
  m_pos_z = np.zeros((1, tamano))

  m_vel_x = np.zeros((1, tamano))
  m_vel_y = np.zeros((1, tamano))
  m_vel_z = np.zeros((1, tamano))

  m_acel_x = np.zeros((1, tamano))
  m_acel_y = np.zeros((1, tamano))
  m_acel_z = np.zeros((1, tamano))

  for i in range(0, tamano):
    ######  Posicion xyz  ##########
    xyz_ls = system_linear_inv(m_pos[i], rot_z, rot_y, theta_y, theta_z, m_trans)

    m_pos_x[0, i] = xyz_ls[0, 0]
    m_pos_y[0, i] = xyz_ls[1, 0]
    m_pos_z[0, i] = xyz_ls[2, 0]

    ######  Velocidad xyz  ##########
    xyz_ls_vel = system_linear_inv(m_vel[i],rot_z, rot_y,theta_y, theta_z, m_trans)
    m_vel_x[0, i] = xyz_ls_vel[0, 0]
    m_vel_y[0, i] = xyz_ls_vel[1, 0]
    m_vel_z[0, i] = xyz_ls_vel[2, 0]

    ######  Aceleracion xyz  ##########
    xyz_ls_acel = system_linear_inv(m_acel[i],rot_z, rot_y,theta_y, theta_z,m_trans)
    m_acel_x[0, i] = xyz_ls_acel[0, 0]
    m_acel_y[0, i] = xyz_ls_acel[1, 0]
    m_acel_z[0, i] = xyz_ls_acel[2, 0]
  
  return [m_tiempo, mtmm * m_pos_x[0], mtmm * m_pos_y[0], mtmm * m_pos_z[0], mtmm * m_vel_x[0], mtmm * m_vel_y[0], mtmm * m_vel_z[0], mtmm * m_acel_x[0], mtmm * m_acel_y[0], mtmm * m_acel_z[0]]
  #          0             1                   2                  3                    4               5                   6                 7                     8                  9

# ******************************************
# ******  Rotacion Inversa ***********
# ******************************************
def system_linear_inv(xprima, rot_z, rot_y, theta_y, theta_z, m_trans):
  
 ######   Creacion de Matrices  ######
 rot_tras_inv = np.zeros((4, 4))
 pf_inv = np.zeros((3, 1))
 pf_trans_inv = np.zeros((4, 1))
 m_trans_inv = np.zeros((4, 4))

 ######   Rotacion Y  ######
 pf_inv[0, 0] = xprima
 pf_inv[1, 0] = 0
 pf_inv[2, 0] = 0

 rot_y_tras = rot_y.transpose()

 m_rot1_inv = rot_y_tras.dot(pf_inv)

 ######   Rotacion Z  ######
 rot_z_tras = rot_z.transpose()
 m_rot2_inv = rot_z_tras.dot(m_rot1_inv)

 ######   Traslacion ######
 pf_trans_inv[0, 0] = m_rot2_inv[0, 0]
 pf_trans_inv[1, 0] = m_rot2_inv[1, 0]
 pf_trans_inv[2, 0] = m_rot2_inv[2, 0]
 pf_trans_inv[3, 0] = 1

 m_trans_inv[0, 3] = -1 * m_trans[0, 3]
 m_trans_inv[1, 3] = -1 * m_trans[1, 3]
 m_trans_inv[2, 3] = -1 * m_trans[2, 3]
 m_trans_inv[0, 0] = 1
 m_trans_inv[1, 1] = 1
 m_trans_inv[2, 2] = 1
 m_trans_inv[3, 3] = 1

 xyz_res = (m_trans_inv).dot(pf_trans_inv)

 return xyz_res
