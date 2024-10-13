# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.พิมพ์ณภัทร_6572
2.ชนัญญ์ทิชา_6578
'''
import numpy as np
import roboticstoolbox as rtb
import FRA333_HW3_6572_6578 as hw3

from math import pi
from spatialmath import SE3
#=============================================================================================================#
# input value
q = [180, 0, 0]
w = [1, 1, 5, 1, 2, 1]

# joint parameter
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082

# สร้าง model robot จาก DH-table
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(a = 0, alpha = 0, d_1 = 0), 
        rtb.RevoluteMDH(a = 0, alpha = pi/2, d = 0), 
        rtb.RevoluteMDH(a = -a_2, alpha = 0, d = 0),
        # rtb.RevoluteMDH(a = -a_3, alpha = 0, d = 0, offset = 0),
        # rtb.RevoluteMDH(a= 0, alpha = 0, d = d_4, offset = 0),
        # rtb.RevoluteMDH(a = 0, alpha = pi/2, d = d_5, offset = pi/2),
        # rtb.RevoluteMDH(a = 0, alpha = -pi/2, d = d_6, offset = 0)
    ],
    name = "Robot")
T3_4 = SE3(-a_3, 0.0, 0.0) @ SE3.Rx(0, 'deg') @ SE3(0.0, 0.0, 0.0) @ SE3.Rz(0, 'deg')
T4_5 = SE3(0.0, 0.0, 0.0) @ SE3.Rx(0, 'deg') @ SE3(0.0, 0.0, d_4) @ SE3.Rz(0, 'deg')
T5_6 = SE3(0, 0.0, 0.0) @ SE3.Rx(90, 'deg') @ SE3(0.0, 0.0, d_5) @ SE3.Rz(90, 'deg')
T6_e = SE3(0, 0.0, 0.0) @ SE3.Rx(-90, 'deg') @ SE3(0.0, 0.0, d_6) @ SE3.Rz(0, 'deg')
T3_e = T3_4 @ T4_5 @ T5_6 @ T6_e
robot.tool = T3_e
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
def test01(q):
    # หา Jacobian โดยใช้ Robotics Toolbox
    Jacobian = robot.jacob0(q)

    # ค่า Jacobian จาก function ที่สร้าง
    J = hw3.endEffectorJacobianHW3(q)

    # ตรวจสอบความถูกต้อง
    for i in range(6):
        for j in range(3):
            error = abs(Jacobian[i][j]) - abs(J[i][j])
            if(abs(error) > 0.0001): # ปรับค่า error ขั้นต่ำ
                return "Test1", False
            else:
                continue
    return "Test1", True

print(test01(q))   
print(robot.jacob0(q))
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
def test02(q):
    # หา Jacobian โดยใช้ Robotics Toolbox
    Jacobian = robot.jacob0(q)
    E = 0.001
    # หา det ของ Jacobian
    det_J = np.linalg.det(Jacobian[:3, :])
    if(abs(det_J)<E):
        test2 = True
    elif(abs(det_J)>E):
        test2 = False
    flag = hw3.checkSingularityHW3(q)
    print("Singularity: test2, flag")
    print(test02, flag)
    if(test2 == flag):
        return "Test2", True
    else:
        return "Test2", False
print(test02(q)) 
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
def test03(q,w):
    # หา Jacobian โดยใช้ Robotics Toolbox
    Jacobian = robot.jacob0(q)
    
    # Jacobian Transpose
    J_T = np.transpose(Jacobian)
    # หา effort ของแต่ละข้อต่อเมื่อมี wrench มากระทำกับ end-effector
    tau0 = J_T @ w
    tau = hw3.computeEffortHW3(q,w)
    print("Effort: test3, tau")
    print(tau0)
    print(tau)
    for i in range(3):
        error = abs(tau0[i]) - abs(tau[i])
        if(abs(error) > 1): # ปรับค่า error ขั้นต่ำ
            return "Test3", False
        else:
                continue
    return "Test3", True
            
print(test03(q,w))
#==============================================================================================================#