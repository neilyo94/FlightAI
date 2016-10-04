import FGI, behavior, pilot

p = pilot.Pilot()
a = FGI.FGI(OS="OSX", collect=4)

a.waitstart()
p.start()
b = behavior.RecoveryBehavior(a)
p.addBeh(b)
