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


from math import pi
from HW3_utils import FKHW3


#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q):
    R,P,R_e,p_e = FKHW3(q)

    p0_1 = sp.Matrix(P[:,0]) # ตำแหน่งเฟรม 1 ที่สัมพัทธ์กับเฟรม 0
    p0_2 = sp.Matrix(P[:,1]) # ตำแหน่งเฟรม 2 ที่สัมพัทธ์กับเฟรม 0
    p0_3 = sp.Matrix(P[:,2]) # ตำแหน่งเฟรม 3 ที่สัมพัทธ์กับเฟรม 0
    p0_e = sp.Matrix(p_e)    # ตำแหน่งของ end-effector ที่สัมพัทธ์กับเฟรม 0
    p = [p0_1, p0_2, p0_3]
    
    # แกนหมุนของแต่ละข้อต่อ
    z1 = sp.Matrix([0, 0, 1])
    z2 = sp.Matrix([sp.sin(q[0]), -sp.cos(q[0]), 0])
    z3 = sp.Matrix([sp.sin(q[0]), -sp.cos(q[0]), 0])
    z = [z1, z2, z3]
    
    Jv = []
    Jw = []
    
    for i in range(3):
        
        # Jacobian เชิงเส้น
        Jv_i = z[i].cross(p0_e - p[i])
        Jv.append(Jv_i)
    
        # Jacobian เชิงมุม
        Jw_i = z[i]
        Jw.append(Jw_i)

    # Jacobian matrix
    Jv_matrix = sp.Matrix.hstack(*Jv)
    Jw_matrix = sp.Matrix.hstack(*Jw)
    Jacobian = sp.Matrix.vstack(Jv_matrix, Jw_matrix)
    
    # ลดรูป Jacobian
    J_e = sp.simplify(Jacobian) 
    J = np.array(J_e).astype(np.float64)
    return J                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q):
    # Jacobian matrix
    J_e = endEffectorJacobianHW3(q)
    
    # กำหนดค่า ε
    E = 0.001
    
    # แปลง sympy เป็น numpy
    J = np.array(J_e).astype(np.float64)
    
    # หา det ของ Jacobian
    det_J = np.linalg.det(J[:3, :])
    print("det(J*(q)) = ", det_J)
    
    # เช็กว่าเกิดสถาวะ Singularity หรือไม่
    if(abs(det_J)<E):
        print("Singularity:")
        return True
    elif(abs(det_J)>E):
        print("Singularity:")
        return False
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q, w):
    # Jacobian matrix
    J_e = endEffectorJacobianHW3(q)
    
    # แปลง sympy เป็น numpy
    J = np.array(J_e).astype(np.float64)
    
    # Jacobian Transpose 
    J_T = np.transpose(J)
    
    # หา effort ของแต่ละข้อต่อเมื่อมี wrench มากระทำกับ end-effector
    tau = J_T @ w
   
    return tau
#==============================================================================================================#