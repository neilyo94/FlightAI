import threading
import numpy as np
import time

class Pilot(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.active_set = set()
        self.aileron = 0.0
        self.throttle = 0.0
        self.elevator = 0.0


    def stop(self):
        threading.Thread.stop(self)

    def report(self):
        for item in self.active_set:
            print(item)

    def run(self):
        while True:
            # store the results of all behaviors
            ctl_elevator = []
            ctl_aileron = []
            ctl_throttle = []

            # make a copy for delete
            active_set_copy = self.active_set.copy()
            for beh in self.active_set:
                fgi = beh.fgi
                state, ctls = beh.behave()

                # remove if success or fail
                if (state in ("success", "failure")):
                    active_set_copy.remove(beh)
                    print("Behavior was removed: %s" % beh)
                else:
                    elevator, aileron, throttle = ctls
                    if (elevator[1]):
                        ctl_elevator.append(elevator[0])
                    if (aileron[1]):
                        ctl_aileron.append(aileron[0])
                    if (throttle[1]):
                        ctl_throttle.append(throttle[0])

            self.active_set = active_set_copy

            # set the control variable to be the mean of all feedbacks from behaviors
            if (len(ctl_elevator) != 0):
                el =  np.mean(ctl_elevator)
                fgi.setFGProp("elevator",el)
                self.elevator = el

            if (len(ctl_aileron) != 0):
                ail = np.mean(ctl_aileron)
                fgi.setFGProp("aileron", ail)
                self.aileron = ail

            if (len(ctl_throttle) != 0):
                thr = np.mean(ctl_throttle)
                fgi.setFGProp("throttle", np.mean(ctl_throttle))
                self.throttle = thr

            time.sleep(0.25)

    def addBeh(self, beh, warmP=False):
        beh.reinit(warmP)
        f = beh.fgi
        prob_of_success = beh.pc((f.look(), [self.elevator, self.aileron, self.throttle]))
        if prob_of_success >= 0.8:
            self.active_set.add(beh)
        else:
            print("Behavior could not be added: %s" % beh)
