#!/usr/bin/python

from cProfile import Profile
from cStringIO import StringIO
from pstats import Stats
from time import sleep

from altimu import AltIMU

imu = AltIMU()
imu.enable()

profile = Profile()

cumTime = []
for profileRun in range(3):
    profile.enable()
    for x in range(1000):
        print "Gyro Angles:", imu.trackGyroAngle()
    profile.disable()

    stream = StringIO()
    stats = Stats(profile, stream = stream).sort_stats('cumulative')
    stats.print_stats(0)

    stream.seek(0)
    cumTime[profileRun] = float(stream.readline().split(' ')[-2])

looptime = float(sum(cumTime)) / (float(len(cumTime)) * 1000.0)

print "Average loop time:", looptime

imu.calibrateGyroAngles()

for x in range(1000):
    print "Gyro Angles:", imu.trackGyroAngle(deltaT = looptime)
