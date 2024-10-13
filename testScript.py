# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.พิมพ์ณภัทร_6572
2.ชนัญญ์ทิชา_6578
'''
import roboticstoolbox as rtb
import FRA333_HW3_6572_6578 as hw3

from math import pi
from spatialmath import SE3
#===========================================<Input>====================================================#
q = [180, 0, 0]
w = [1, 1, 5, 1, 2, 1]

d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082

# DH-table
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
    name = "Robot"
)
T3_4 = SE3(-a_3, 0.0, 0.0) @ SE3.Rx(0, 'deg') @ SE3(0.0, 0.0, 0.0) @ SE3.Rz(0, 'deg')
T4_5 = SE3(0.0, 0.0, 0.0) @ SE3.Rx(0, 'deg') @ SE3(0.0, 0.0, d_4) @ SE3.Rz(0, 'deg')
T5_6 = SE3(0, 0.0, 0.0) @ SE3.Rx(90, 'deg') @ SE3(0.0, 0.0, d_5) @ SE3.Rz(90, 'deg')
T6_e = SE3(0, 0.0, 0.0) @ SE3.Rx(-90, 'deg') @ SE3(0.0, 0.0, d_6) @ SE3.Rz(0, 'deg')
T3_e = T3_4 @ T4_5 @ T5_6 @ T6_e

robot.tool = T3_e
print(robot)
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
# q0 = [0, 0, 0]
# for i in range(3):
#     q0[i] = q[i]
# print(q0)
print(robot.jacob0(q))
J = hw3.endEffectorJacobianHW3(q)
print(J)
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
flag = hw3.checkSingularityHW3(q)
print(flag)
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
tau = hw3.computeEffortHW3(q, w)
print("Effort:", tau)
#==============================================================================================================#