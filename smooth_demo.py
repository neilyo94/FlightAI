#######################################
# This script will generate the smooth
# control figure in the report
#######################################

import FGI
import PIDController
from PIDController import DiscretePIDControllerDriver as DPD
from time import sleep
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# create FGI object and connect with communicator
plane = FGI.FGI(OS="Ubn")
plane.waitstart()
communicator = PIDController.FGCommunicator(plane)
communicator.communicate()
roll_ctrl = DPD.createRollStablizer(communicator, 0.02, 0.0005, 0.003)

# start controlling
roll_ctrl.stablize()
# wait for the plane to get initial state
sleep(10)

# add new target value
roll_ctrl.stablizer.newTarget(35)
sleep(50)

# end communication
roll_ctrl.stopStablizing()
communicator.stopCommunicate()
plane.termFG()


plt.figure()
plt.plot(roll_ctrl.controlVal)
plt.ylim((-1, 1))
plt.title("elevator signal")
plt.savefig("smooth/elevator_signal" + str(roll_ctrl.stablizer.smoother) + ".png")

plt.figure()
plt.plot(roll_ctrl.targetVal)
plt.ylim((-5, 40))
plt.title("roll signal")
plt.savefig("smooth/roll_signal" + str(roll_ctrl.stablizer.smoother) + ".png")



