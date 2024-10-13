#!/usr/bin/python3
import numpy as np
import math

def FKHW3(q):
    [q1,q2,q3] = q                                              
    d_1 = 0.0892
    a_2 = -0.425
    a_3 = -0.39243
    d_4 = 0.109
    d_5 = 0.093
    d_6 = 0.082
    p1_14 = 0
    p1_24 = 0
    p1_34 = 223/2500
    r1_11 = - math.cos(q1) - (4967757600021511*math.sin(q1))/40564819207303340847894502572032
    r1_12 = math.sin(q1) - (4967757600021511*math.cos(q1))/40564819207303340847894502572032
    r1_13 = 0
    r1_21 = (4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)
    r1_22 = - math.cos(q1) - (4967757600021511*math.sin(q1))/40564819207303340847894502572032
    r1_23 = 0
    r1_31 = 0
    r1_32 = 0
    r1_33 = 1
    p2_14 = 0
    p2_24 = 0
    p2_34 = 223/2500
    r2_11 = - math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) - math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)
    r2_12 = math.sin(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032) - math.cos(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064)
    r2_13 = (4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)
    r2_21 = math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) - math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)
    r2_22 = - math.sin(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) - math.cos(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)
    r2_23 = math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032
    r2_31 = math.sin(q2)
    r2_32 = math.cos(q2)
    r2_33 = 4967757600021511/81129638414606681695789005144064
    p3_14 = (17*math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064))/40 + (17*math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032))/40
    p3_24 = (17*math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048))/40 - (17*math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)))/40
    p3_34 = 223/2500 - (17*math.sin(q2))/40
    r3_11 = - math.cos(q3)*(math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) + math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)) - math.sin(q3)*(math.cos(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) - math.sin(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032))
    r3_12 = math.sin(q3)*(math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) + math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)) - math.cos(q3)*(math.cos(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) - math.sin(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032))
    r3_13 = (4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)
    r3_21 = math.cos(q3)*(math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) - math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)) - math.sin(q3)*(math.sin(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) + math.cos(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048))
    r3_22 = - math.cos(q3)*(math.sin(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) + math.cos(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)) - math.sin(q3)*(math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) - math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048))
    r3_23 = math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032
    r3_31 = math.cos(q2)*math.sin(q3) + math.cos(q3)*math.sin(q2)
    r3_32 = math.cos(q2)*math.cos(q3) - math.sin(q2)*math.sin(q3)
    r3_33 = 4967757600021511/81129638414606681695789005144064
    p4_14 = (541485578402344699*math.cos(q1))/40564819207303340847894502572032000 - (109*math.sin(q1))/1000 + (47443*math.cos(q3)*(math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) + math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)))/100000 + (93*math.cos(q3)*(math.cos(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) - math.sin(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)))/1000 - (93*math.sin(q3)*(math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) + math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)))/1000 + (47443*math.sin(q3)*(math.cos(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) - math.sin(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)))/100000 + (17*math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064))/40 + (17*math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032))/40
    p4_24 = (109*math.cos(q1))/1000 + (541485578402344699*math.sin(q1))/40564819207303340847894502572032000 - (17*math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)))/40 + (93*math.cos(q3)*(math.sin(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) + math.cos(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)))/1000 - (47443*math.cos(q3)*(math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) - math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)))/100000 + (47443*math.sin(q3)*(math.sin(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) + math.cos(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)))/100000 + (93*math.sin(q3)*(math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) - math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)))/1000 + (17*math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048))/40
    p4_34 = (93*math.sin(q2)*math.sin(q3))/1000 - (93*math.cos(q2)*math.cos(q3))/1000 - (47443*math.cos(q2)*math.sin(q3))/100000 - (47443*math.cos(q3)*math.sin(q2))/100000 - (17*math.sin(q2))/40 + 36183818732914582743749788305976039/405648192073033408478945025720320000
    r4_11 = (4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1) - (4967757600021511*math.cos(q3)*(math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) + math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)))/81129638414606681695789005144064 - (4967757600021511*math.cos(q3)*(math.cos(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) - math.sin(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)))/81129638414606681695789005144064 + (4967757600021511*math.sin(q3)*(math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) + math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)))/81129638414606681695789005144064 - (4967757600021511*math.sin(q3)*(math.cos(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) - math.sin(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)))/81129638414606681695789005144064
    r4_12 = (4967757600021511*math.sin(q1))/81129638414606681695789005144064 - (24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 + (4967757600021511*math.cos(q3)*(math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) + math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)))/81129638414606681695789005144064 - math.cos(q3)*(math.cos(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) - math.sin(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)) + math.sin(q3)*(math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) + math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)) + (4967757600021511*math.sin(q3)*(math.cos(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) - math.sin(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)))/81129638414606681695789005144064
    r4_13 = (24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064 + math.cos(q3)*(math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) + math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)) + (4967757600021511*math.cos(q3)*(math.cos(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) - math.sin(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)))/81129638414606681695789005144064 - (4967757600021511*math.sin(q3)*(math.sin(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) + math.cos(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032)))/81129638414606681695789005144064 + math.sin(q3)*(math.cos(q2)*((24678615572571482867467662723121*math.cos(q1))/3291009114642412084309938365114701009965471731267159726697218048 - (4967757600021511*math.sin(q1))/81129638414606681695789005144064) - math.sin(q2)*(math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032))
    r4_21 = math.cos(q1) + (4967757600021511*math.sin(q1))/40564819207303340847894502572032 - (4967757600021511*math.cos(q3)*(math.sin(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) + math.cos(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)))/81129638414606681695789005144064 + (4967757600021511*math.cos(q3)*(math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) - math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)))/81129638414606681695789005144064 - (4967757600021511*math.sin(q3)*(math.sin(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) + math.cos(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)))/81129638414606681695789005144064 - (4967757600021511*math.sin(q3)*(math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) - math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)))/81129638414606681695789005144064
    r4_22 = (4967757600021511*math.sin(q3)*(math.sin(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) + math.cos(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)))/81129638414606681695789005144064 - (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048 - math.cos(q3)*(math.sin(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) + math.cos(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)) - (4967757600021511*math.cos(q3)*(math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) - math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)))/81129638414606681695789005144064 - (4967757600021511*math.cos(q1))/81129638414606681695789005144064 - math.sin(q3)*(math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) - math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048))
    r4_23 = (4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048 + (4967757600021511*math.cos(q3)*(math.sin(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) + math.cos(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)))/81129638414606681695789005144064 - math.cos(q3)*(math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) - math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)) + math.sin(q3)*(math.sin(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) + math.cos(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)) + (4967757600021511*math.sin(q3)*(math.cos(q2)*((4967757600021511*math.cos(q1))/40564819207303340847894502572032 - math.sin(q1)) - math.sin(q2)*((4967757600021511*math.cos(q1))/81129638414606681695789005144064 + (24678615572571482867467662723121*math.sin(q1))/3291009114642412084309938365114701009965471731267159726697218048)))/81129638414606681695789005144064
    r4_31 = (4967757600021511*math.cos(q2)*math.cos(q3))/81129638414606681695789005144064 + (4967757600021511*math.cos(q2)*math.sin(q3))/81129638414606681695789005144064 + (4967757600021511*math.cos(q3)*math.sin(q2))/81129638414606681695789005144064 - (4967757600021511*math.sin(q2)*math.sin(q3))/81129638414606681695789005144064 + 4967757600021511/81129638414606681695789005144064
    r4_32 = math.cos(q2)*math.cos(q3) - (4967757600021511*math.cos(q2)*math.sin(q3))/81129638414606681695789005144064 - (4967757600021511*math.cos(q3)*math.sin(q2))/81129638414606681695789005144064 - math.sin(q2)*math.sin(q3) - 24678615572571482867467662723121/6582018229284824168619876730229402019930943462534319453394436096
    r4_33 = (4967757600021511*math.sin(q2)*math.sin(q3))/81129638414606681695789005144064 - math.cos(q2)*math.sin(q3) - math.cos(q3)*math.sin(q2) - (4967757600021511*math.cos(q2)*math.cos(q3))/81129638414606681695789005144064 + 24678615572571482867467662723121/6582018229284824168619876730229402019930943462534319453394436096

    R = np.empty((3,3,4),dtype=np.float32)
    R[:,:,0] = np.array([r1_11, r1_12, r1_13, r1_21, r1_22, r1_23, r1_31, r1_32, r1_33]).reshape(3,3)
    R[:,:,1] = np.array([r2_11, r2_12, r2_13, r2_21, r2_22, r2_23, r2_31, r2_32, r2_33]).reshape(3,3)
    R[:,:,2] = np.array([r3_11, r3_12, r3_13, r3_21, r3_22, r3_23, r3_31, r3_32, r3_33]).reshape(3,3)
    R[:,:,3] = np.array([r4_11, r4_12, r4_13, r4_21, r4_22, r4_23, r4_31, r4_32, r4_33]).reshape(3,3)

    P = np.empty((3,4));     
    P[:,0] = [p1_14, p1_24, p1_34]
    P[:,1] = [p2_14, p2_24, p2_34]
    P[:,2] = [p3_14, p3_24, p3_34]
    P[:,3] = [p4_14, p4_24, p4_34]
    R_e = R[:,:,3]
    p_e = P[:,3]

    return R,P,R_e,p_e
# R = FKHW3([180, 0, 0])
# print(R)



