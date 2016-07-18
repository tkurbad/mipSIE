#!/usr/bin/python

from cProfile import Profile
from cStringIO import StringIO
from pstats import Stats
from time import sleep

from altimu import AltIMU

imu = AltIMU()
imu.enable()

profile = Profile()

profile.enable()
x = 0
while x < 2000:
    print "Gyro Angles:", imu.trackGyroAngle()
    x += 1
profile.disable()

stream = StringIO()
stats = Stats(profile, stream = stream).sort_stats('cumulative')
stats.print_stats()
print stream.getvalue()
