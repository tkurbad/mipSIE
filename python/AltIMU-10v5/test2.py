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
for x in range(1000):
    print "Gyro Angles:", imu.trackGyroAngle()
profile.disable()

stream = StringIO()
stats = Stats(profile, stream = stream).sort_stats('cumulative')
stats.print_stats(1)
import pdb; pdb.set_trace()
