import math

#Geometric Dimensioning Robot Delta

def l2():
 value = (465.0) * ((10.0) ** (-3))  # Longitud Antebrazo [m]
 return value


def l1():
 value = (200.0) * ((10.0) ** (-3))  # Longitud Brazo [m]
 return value

def e():
 value = (86.5) * ((10.0) ** (-3))  # [m]
 return value


def f():
 value = (346.4) * ((10.0) ** (-3))  # [m]
 return value

def rb():
 value = e() * tan30/2  # Radio plataforma movil[m]
 return value

def ra():
 value = f() * tan30/2# Radio base fija [m]
 return value



def hf():
 value = math.sqrt(0.75 * (f() ** 2))  # [m]
 return value


def he():
 value = math.sqrt(0.75 * (e() ** 2))  # [m]
 return value

 #############################################

def dtr():
 value = math.pi / 180.0  # Degrees to Radians
 return value


def rtd():
 value = 180.0 / math.pi  # Radians to Degrees
 return value


def mmtm():
 value = ((10.0) ** (-3))  # Millimeters to Meters
 return value


def mtmm():
 value = ((10.0) ** (3))  # Meters to Millimeters
 return value


def kgm2tgrmm2():
 value = ((10.0) ** (3 + 6))  # Kilogram meter ^ 2 to gram mm ^ 2
 return value


#Trigonometry

def sqrt3():
 value = math.sqrt(3.0)
 return value

def pi():
 value = math.pi  # [rad]
 return value

def sin120():
 value = sqrt3() / 2.0
 return value

def cos120():
 value = -0.5
 return value

def tan60():
 value = sqrt3()
 return value

def sin30():
 value = 0.5
 return value

def tan30():
 value = 1.0 / sqrt3()
 return value

def cos30():
 value = (sqrt3()) / 2.0
 return value

#Masas e Inercias

def m1():
 value = (2213.0) * ((10.0) ** (-3))  # Masa Brazo [kg]
 return value


def m_elbow():
 value = (0.0) * ((10.0) ** (-3))  # Masa juntas [kg]
 return value


def m2():
 value = (1315.0 / 2.0) * ((10.0) ** (-3))  # Masa de una varilla del antebrazo [kg]
 return value


def mp():
 value = (510.0) * ((10.0) ** (-3))  # Masa plataforma movil [kg]
 return value


def inercia_m():
 value = (0.0) * ((10.0) ** (-3))  # Inercia actuadores [kg * m2]
 return value


def gg():
 value = (9.81)  # Gravedad [m/s2]
 return value


def r_mass():
 value = (0.5)  # Relacion de masas del antebrazo
 return value


def com():
 value = (l1()) * (((0.5 * (m1())) + (m_elbow()) + ((2) * (r_mass()) * (m2()))) /
				   ((1.0 * (m1())) + (m_elbow()) + ((2) * (r_mass()) * (m2()))))
 # Centro de masas brazo
 return value


######################################################
##########  Restricciones Espacio de Trabajo #########
######################################################
# Restricciones limines de angulos de motores [grados]
def res_ang_min():
 value = int(-90)
 # value=int(-30)
 return value


def res_ang_max():
 value = ((res_ang_min()) * -1) + 1
 # value=90
 return value


# Restriction of angles 2 and 3 of each arm [degrees]
def theta2i_min():
 value = 5
 return value


def theta2i_max():
 value = 180 - theta2i_min()
 return value


def theta3i_min():
 value = 45
 return value


def theta3i_max():
 value = 180 - theta3i_min()
 return value


# Restricciones Singularidad Jacobiano
def err_jxx():
 value = (6) * (10 ** (-1))
 return value


def err_jinv():
 value = (4) * (10 ** (-3))
 return value


# Restriccion del fabricante (caja) [mm]
def px_max_ws():
 value = 400
 return value


def px_min_ws():
 value = (-1) * px_max_ws()
 return value


def py_max_ws():
 value = 400
 return value


def py_min_ws():
 value = (-1) * py_max_ws()
 return value


def pz_max_ws():
 value = -300
 return value


def pz_min_ws():
 value = -750
 return value
