import roboticstoolbox as rtb
from manipulator_utils import KR4R600, KR3R540

robot = KR4R600()

Te = robot.fkine(robot.qz)  # forward kinematics
print(Te)

from spatialmath import SE3

Tep = SE3.Trans(0, 0, 0) * SE3.OA([0, 1, 0], [0, 0, 1])
print(Tep)
sol = robot.ik_lm_chan(Tep)         # solve IK
print(sol)

q_pickup = sol[0]
print(robot.fkine(q_pickup))

qt = rtb.jtraj(robot.qz, robot.home, 50)

robot.plot(qt.q, backend='pyplot')