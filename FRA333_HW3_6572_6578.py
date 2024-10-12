# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.พิมพ์ณภัทร_6572
2.ชนัญญ์ทิชา_6578
'''
import numpy as np 
import sympy as sp 
import math
import roboticstoolbox as rtb

from spatialmath import SE3
from math import pi
#=============================================<function>======================================================#
# Transformation matrix function
def transform(a, alpha, d, theta):
    return sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0, a],
                      [sp.sin(theta) * sp.cos(alpha), sp.cos(theta) * sp.cos(alpha), -sp.sin(alpha), -sp.sin(alpha) * d],
                      [sp.sin(theta) * sp.sin(alpha), sp.cos(theta) * sp.sin(alpha), sp.cos(alpha), sp.cos(alpha) * d],
                      [0, 0, 0, 1]])
#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q_1, q_2, q_3):
    z = []
    p = []
    Jv = []
    Jw = []
    d_1 = 0.0892
    a_2 = 0.425
    a_3 = 0.39243
    d_4 = 0.109
    d_5 = 0.093
    d_6 = 0.082
    
    T0_1 = transform(0, 0, d_1, q_1)
    T1_2 = transform(0, sp.pi/2, 0, q_2) 
    T2_3 = transform(-a_2, 0, 0, q_3)
    T3_e = transform(-a_3, 0, 0, 0)*transform(0, 0, d_4, 0)*transform(-(sp.pi/2), 0, d_5, sp.pi/2)*transform(-(sp.pi/2), 0, d_6, 0)
    
    T_total = T0_1 * T1_2 * T2_3 * T3_e
    
    p_end = T_total[:3, 3]
    
    T_matrices = [T0_1, T0_1 * T1_2, T0_1 * T1_2 * T2_3]
    for T in T_matrices:
        z.append(T[:3, 2])
        p.append(T[:3, 3])
        
    for i in range(3):
        # Jacobian เชิงเส้น
        Jv_i = z[i].cross(p_end - p[i])
        Jv.append(Jv_i)
    
        # Jacobian เชิงมุม
        Jw_i = z[i]
        Jw.append(Jw_i)

    # Jacobian matrix
    Jv_matrix = sp.Matrix.hstack(*Jv)
    Jw_matrix = sp.Matrix.hstack(*Jw)
    Jacobian = sp.Matrix.vstack(Jv_matrix, Jw_matrix)
    J_e = sp.simplify(Jacobian) 
    return J_e                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    return flag
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    return tau
#==============================================================================================================#