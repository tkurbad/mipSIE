#!/usr/bin/env python
# coding: Latin-1

# Simple example of a motor sequence script

# Import library functions we need
import PicoBorgRev
import time

# Setup the PicoBorg Reverse
PBR = PicoBorgRev.PicoBorgRev()     # Create a new PicoBorg Reverse object
PBR.Init()                          # Set the board up (checks the board is connected)
PBR.ResetEpo()                      # Reset the stop switch (EPO) state
                                    # if you do not have a switch across the two pin header then fit the jumper

# Set our sequence, pairs of motor 1 and motor 2 drive levels
sequence = [
            [+0.2, +0.2],
            [+0.4, +0.4],
            [+0.6, +0.6],
            [+0.8, +0.8],
            [+1.0, +1.0],
            [+0.6, +1.0],
            [+0.2, +1.0],
            [-0.2, +1.0],
            [-0.6, +1.0],
            [-1.0, +1.0],
            [-0.6, +0.6],
            [-0.2, +0.2],
            [+0.2, -0.2],
            [+0.6, -0.6],
            [+1.0, -1.0],
            [+0.6, -0.6],
            [+0.3, -0.3],
            [+0.1, -0.1],
            [+0.0, +0.0],
           ]
stepDelay = 1.0                     # Number of seconds between each sequence step

# Loop over the sequence until the user presses CTRL+C
print 'Press CTRL+C to finish'
try:
    while True:
        # Go through each entry in the sequence in order
        for step in sequence:
            PBR.SetMotor1(step[0])                  # Set the first motor to the first value in the pair
            PBR.SetMotor2(step[1])                  # Set the second motor to the second value in the pair
            print '%+.1f %+.1f' % (step[0], step[1])
            time.sleep(stepDelay)                   # Wait between steps
except KeyboardInterrupt:
    # User has pressed CTRL+C
    PBR.MotorsOff()                 # Turn both motors off
    print 'Done'
