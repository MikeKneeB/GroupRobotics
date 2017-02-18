import HSMSim as h

test = h.HorizontalSlidingMassSimulation()
test2 = h.HorizontalSlidingMassSimulation()
print test.length
test.sim()
print test.t
test.t = test.t + 2
print test.t

for n in range(0, 5):

    print test2.iterate()
