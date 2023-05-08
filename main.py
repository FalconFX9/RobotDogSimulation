import math
import matplotlib.pyplot as plt
import numpy as np

# All measurements are in mm

ULL = 150
LLL = 120
mass = 1


def forward_kinematics(theta1, theta2):
    xf = - math.cos(theta1) * ULL + math.sin(theta2) * LLL
    yf = math.sin(theta1) * ULL + math.cos(theta2) * LLL
    return [xf, yf]


def get_angles_from_length(l):
    theta1 = math.pi/2 - math.acos(((l ** 2) + (ULL ** 2) - (LLL ** 2))/(2 * l * ULL))
    theta2 = math.acos((LLL**2 + l**2 - ULL**2)/(2 * LLL * l))
    return theta1, theta2


def inverse_kinematics(xf, yf):
    new_leg_length = math.sqrt(xf**2 + yf**2)
    added_angle = math.atan2(xf, yf)
    t1, t2 = get_angles_from_length(new_leg_length)
    t1 += added_angle
    # t2 += added_angle  # whether or not this is added depends on the placement of the knee joint motor (add if on chassis, do not add if on leg)
    return t1, t2, added_angle


t1a, t2a, aa = inverse_kinematics(50, 150)
# xf, yf = forward_kinematics(t1, t2 + aa)
print(t1a, t2a)
t1b, t2b, aa = inverse_kinematics(-50, 150)
print(t1b, t2b)
print(t1b-t1a, t2b-t2a)

foot_pos = []
t1 = []
t1tan = []
t2 = []
t2tan = []
x = -50
TARGET_SPEED = 1000  # mm/s
max_speeds = []
height = []
tm_t = []
bm_t = []
max_tms = []
max_bms = []
for j in range(100, int(math.sqrt((ULL+LLL)**2 - 50**2)-1), 5):
    x = -50
    for i in range(1000):
        t1t, t2t, aa = inverse_kinematics(x, j)
        tm_torque = mass * (x/10)
        bm_torque = mass * math.sin(t2t + aa) * (LLL/10)
        foot_pos.append(x)
        x += 0.1
        t1.append(t1t)
        t2.append(t2t)
        tm_t.append(tm_torque)
        bm_t.append(bm_torque)
        if i != 0:
            t1tan.append((t1[-1] - t1[-2])/((100/TARGET_SPEED)/1000))
            t2tan.append((t2[-1] - t2[-2])/((100/TARGET_SPEED)/1000))

    max_t1 = max(t1tan)
    max_t2 = max(t2tan)
    t1tan = []
    t2tan = []

    max_tm = max(tm_t)
    max_bm = max(bm_t)
    tm_t = []
    bm_t = []
    # print(max(max_t1, max_t2), "Rad/s", max(max_tm, max_bm), "Nm")
    max_speeds.append(max(max_t1, max_t2))
    #max_torques.append(max(max_tm, max_bm))
    max_tms.append(max_tm)
    max_bms.append(max_bm)
    height.append(j)
    # Typical RC Servo No-Load Max: 8 Rad/s


plt.plot(height, max_speeds, height, max_tms, height, max_bms)
plt.legend(["Min Req. Motor Speed", "Min Req. Top Motor Torque", "Min Req. Bottom Motor Torque"])
plt.show()
plt.plot(foot_pos, t1)
plt.show()
plt.plot(foot_pos, t2)
plt.show()

plt.plot(foot_pos[1:], t1tan, foot_pos[1:], t2tan)
plt.show()
