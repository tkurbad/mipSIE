#!/usr/bin/env python
# coding: latin-1

# Import libary functions we need
import PicoBorgRev
import time

# Tell the system how to drive the stepper
sequence = [[1.0, 1.0], [1.0, -1.0], [-1.0, -1.0], [-1.0, 1.0]] # Order for stepping 
stepDelay = 0.002                                               # Delay between steps
degPerStep= 1.8                                                 # Number of degrees moved per step

# Name the global variables
global step
global PBR

# Setup the PicoBorg Reverse
PBR = PicoBorgRev.PicoBorgRev()
#PBR.i2cAddress = 0x44                   # Uncomment and change the value if you have changed the board address
PBR.Init()
if not PBR.foundChip:
    boards = PicoBorgRev.ScanForPicoBorgReverse()
    if len(boards) == 0:
        print 'No PicoBorg Reverse found, check you are attached :)'
    else:
        print 'No PicoBorg Reverse at address %02X, but we did find boards:' % (PBR.i2cAddress)
        for board in boards:
            print '    %02X (%d)' % (board, board)
        print 'If you need to change the I²C address change the setup line so it is correct, e.g.'
        print 'PBR.i2cAddress = 0x%02X' % (boards[0])
    sys.exit()
#PBR.SetEpoIgnore(True)                 # Uncomment to disable EPO latch, needed if you do not have a switch / jumper
PBR.ResetEpo()
step = -1

# Function to perform a sequence of steps as fast as allowed
def MoveStep(count):
    global step
    global PBR

    # Choose direction based on sign (+/-)
    if count < 0:
        dir = -1
        count *= -1
    else:
        dir = 1

    # Loop through the steps
    while count > 0:
        # Set a starting position if this is the first move
        if step == -1:
            drive = sequence[-1]
            PBR.SetMotor1(drive[0])
            PBR.SetMotor2(drive[1])
            step = 0
        else:
            step += dir

        # Wrap step when we reach the end of the sequence
        if step < 0:
            step = len(sequence) - 1
        elif step >= len(sequence):
            step = 0

        # For this step set the required drive values
        if step < len(sequence):
            drive = sequence[step]
            PBR.SetMotor1(drive[0])
            PBR.SetMotor2(drive[1])
        time.sleep(stepDelay)
        count -= 1

# Function to move based on an angular distance
def MoveDeg(angle):
    count = int(angle / float(degPerStep))
    MoveStep(count)

try:
    # Start by turning all drives off
    PBR.MotorsOff()
    # Loop forever
    while True:
        # Rotate forward 10 turns then wait
        MoveDeg(360 * 10)
        time.sleep(1)
        # Rotate reverse 10 turns then wait
        MoveDeg(-360 * 10)
        time.sleep(1)
        # Move forward 1/4 turn 16 times with waits between
        for i in range(16):
            MoveDeg(90)
            time.sleep(1)
except KeyboardInterrupt:
    # CTRL+C exit, turn off the drives and release the GPIO pins
    PBR.MotorsOff()
    print 'Terminated'


