import math
import delta_define
import numpy as np
from numpy.linalg import inv

###############################################
########### [Parametros] ######################
###############################################
ra = delta_define.ra()
rb = delta_define.rb()
l1 = delta_define.l1()
l2 = delta_define.l2()
rdif = (ra - rb)

# Constantes Trigonometricas
m1 = delta_define.m1()
m_elbow = delta_define.m_elbow()
m2 = delta_define.m2()
i_motor = delta_define.inercia_m()
g = delta_define.gg()
mp = delta_define.mp()

# Trigonometric constants
sqrt3 = delta_define.sqrt3()
pi = delta_define.pi()
sin120 = sqrt3 / 2.0
cos120 = delta_define.cos120()
tan60 = sqrt3
sin30 = delta_define.sin30()
tan30 = delta_define.tan30()

dtr = delta_define.dtr()  # degrees to radians
mtmm = delta_define.mtmm()  # Metros a Milimetros
mmtm = delta_define.mmtm()  # Milimetros a Metros
kgm2tgrmm2 = delta_define.kgm2tgrmm2()
rtd = delta_define.rtd()  # degrees to radians

###############################################
########## [Torque Menu Input] ################
###############################################
# |------------------------------------------|
# |------- Torque TOTAL (puntual) -----------|
# |------------------------------------------|
def ti(theta11_pp_deg, theta12_pp_deg, theta13_pp_deg,
       theta11_deg, theta12_deg, theta13_deg,
       xp_mm, yp_mm, zp_mm,
       xp_pp_mm, yp_pp_mm, zp_pp_mm,
       fpx, fpy, fpz,
       m_playload):
    # Cambiar unidades a SI
    # Cambiar orden angulos y coordenadas cartesianas a sistema referencia LOCAL
    phi = [0 * dtr, 120 * dtr, 240 * dtr]
    theta1i_pp = [theta12_pp_deg * dtr, theta11_pp_deg * dtr, theta13_pp_deg * dtr]
    theta1i = [theta12_deg * dtr, theta11_deg * dtr, theta13_deg * dtr]
    punto = [-yp_mm * mmtm, -xp_mm * mmtm, -zp_mm * mmtm]
    punto_pp = [-yp_pp_mm * mmtm, -xp_pp_mm * mmtm, -zp_pp_mm * mmtm]
    fp = [-fpy, -fpx, -fpz]

    lambdai = system_tq(phi, theta1i, punto, fp, punto_pp, m_playload)

    t1 = ti_puntual(phi[0], theta1i_pp[0], theta1i[0],
        punto[0], punto[1], punto[2], lambdai[0, 0])
    t2 = ti_puntual(phi[1], theta1i_pp[1], theta1i[1],
        punto[0], punto[1], punto[2], lambdai[1, 0])
    t3 = ti_puntual(phi[2], theta1i_pp[2], theta1i[2],
        punto[0], punto[1], punto[2], lambdai[2, 0])

    # Cambio orden sistema referencia GLOBAL
    return [t2, t1, t3]

###############################################
##### [Torque puntual motor i] ################
###############################################
def ti_puntual(phi,
               theta1i_pp, theta1i,
               xp, yp, zp,
               lambdai):
    A = (((m1 / 3.0) + (m2)) * (l1 ** 2.0) * (theta1i_pp))
    B = (((m1 / 2.0) + (m2)) * (g * l1) * (math.cos(theta1i)))  # g

    C1 = (((xp) * (math.cos(phi))) + ((yp) * (math.sin(phi))) - (rdif)) * (math.sin(theta1i))
    C2 = zp * (math.cos(theta1i))
    C = C1 - C2

    torq_2 = (A)  # Inercia BRazo	#0x00
    torq_3 = (-B)  # Peso Brazo-Antebrazo
    torq_1 = ((-1) * 2.0 * lambdai * l1 * C)  # Peso efector +   #0x00

    # Efector + Inercia BRazo + Peso brazo-antebrazo
    torq = (torq_1 * (1)) + (torq_2 * (1)) + (torq_3 * (1))
    return torq

###############################################
##### [Multiplicadores de Lagrange] ###########
###############################################
# |------------------------------------------|
# |--------Sistema Ecuaciones ---------------| 
# |------------------------------------------|
def system_tq(phi, theta1i, punto, fp, punto_pp, m_playload):
    # Creacion Matrices
    m_A = np.zeros((3, 3))
    m_B = np.zeros((3, 1))
    m_lamda = np.zeros((3, 1))

    fpx = fp[0]
    fpy = fp[1]
    fpz = fp[2]
    xp_pp = punto_pp[0]
    yp_pp = punto_pp[1]
    zp_pp = punto_pp[2]

    # Matriz A
    # x
    m_A[0, 0] = maxx(phi[0], theta1i[0], punto[0])
    m_A[0, 1] = maxx(phi[1], theta1i[1], punto[0])
    m_A[0, 2] = maxx(phi[2], theta1i[2], punto[0])
    # y
    m_A[1, 0] = mayy(phi[0], theta1i[0], punto[1])
    m_A[1, 1] = mayy(phi[1], theta1i[1], punto[1])
    m_A[1, 2] = mayy(phi[2], theta1i[2], punto[1])
    # z
    m_A[2, 0] = mazz(theta1i[0], punto[2])
    m_A[2, 1] = mazz(theta1i[1], punto[2])
    m_A[2, 2] = mazz(theta1i[2], punto[2])

    # Matriz B
    m_B[0, 0] = ((mnt(m_playload) + (3.0 * m2)) * ((1) * (xp_pp))) - (fpx)
    m_B[1, 0] = ((mnt(m_playload) + (3.0 * m2)) * ((1) * (yp_pp))) - (fpy)
    m_B[2, 0] = ((mnt(m_playload) + (3.0 * m2)) * ((1) * (zp_pp - g))) - (fpz)

    m_lamda = (inv(m_A)).dot(m_B)
    return m_lamda

# |-----------------------------|
# |-------- M_AX ---------------| 
# |-----------------------------|
def maxx(phi, theta1i, xp):
    value = 2.0 * ((xp) - (rdif * (math.cos(phi))) -
                   (l1 * (math.cos(theta1i)) * (math.cos(phi))))
    return value

# |-----------------------------|
# |-------- M_AY ---------------| 
# |-----------------------------|
def mayy(phi, theta1i, yp):
    value = 2.0 * ((yp) - (rdif * (math.sin(phi))) -
                   (l1 * (math.cos(theta1i)) * (math.sin(phi))))
    return value

# |-----------------------------|
# |-------- M_AZ ---------------| 
# |-----------------------------|
def mazz(theta1i, zp):
    value = 2.0 * (zp - (l1 * (math.sin(theta1i))))
    return value

###############################################
######### [Torque Matricial] ##################
###############################################
# |----------------------------------------|
# |-------- Torque TOTAL ------------------| 
# |----------------------------------------|
def ti_matriz(theta11_pp_deg, theta12_pp_deg, theta13_pp_deg,
              theta11_deg, theta12_deg, theta13_deg,
              xp_mm, yp_mm, zp_mm,
              xp_pp_mm, yp_pp_mm, zp_pp_mm,
              fpx, fpy, fpz,
              m_playload):
    tamano = len(xp_mm)
    tm1 = np.zeros(tamano)
    tm2 = np.zeros(tamano)
    tm3 = np.zeros(tamano)

    for i in range(0, tamano):
        t231 = ti(theta11_pp_deg[i], theta12_pp_deg[i], theta13_pp_deg[i],
                  theta11_deg[i], theta12_deg[i], theta13_deg[i],
                  xp_mm[i], yp_mm[i], zp_mm[i],
                  xp_pp_mm[i], yp_pp_mm[i], zp_pp_mm[i],
                  fpx[i], fpy[i], fpz[i],
                  m_playload)
        tm1[i] = t231[0]
        tm2[i] = t231[1]
        tm3[i] = t231[2]
    return [tm1, tm2, tm3]

# |------------------------------------------|
# |-------  masa efector TOTAL    -----------| 
# |------------------------------------------|
# masa efector + masa objeto a levantar + masa dividida antebrazo Lb
def mnt(m_playload):
    mass_mnt = mp + m_playload
    return mass_mnt

############################# FIN ###############################
