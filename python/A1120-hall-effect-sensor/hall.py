#!/usr/bin/env python

import atexit
import pigpio
import time

#
# OH3144E or equivalent Hall effect sensor
#
# Pin 1 - 5V
# Pin 2 - Ground
# Pin 3 - gpio (here P1-8, gpio 14, TXD is used)
#
# The internal gpio pull-up is enabled so that the sensor
# normally reads high.  It reads low when a magnet is close.
#

SERVO = 17
HALL=27

pi = pigpio.pi('10.168.16.22') # connect to local Pi

pi.set_servo_pulsewidth(SERVO, 0)

pi.set_mode(HALL, pigpio.INPUT)
pi.set_pull_up_down(HALL, pigpio.PUD_UP)

def move_servo(gpio, level, tick):
    if level == 1:
        pi.set_servo_pulsewidth(SERVO, 2500)
        time.sleep(0.3)
        pi.set_servo_pulsewidth(SERVO, 0)
        return
    pi.set_servo_pulsewidth(SERVO, 500)
    time.sleep(0.3)
    pi.set_servo_pulsewidth(SERVO, 0)

def cleanup():
    servo_callback.cancel()
    pi.stop()

servo_callback = pi.callback(HALL, pigpio.EITHER_EDGE, move_servo)
atexit.register(cleanup)

while True:
    time.sleep(20)


#!/usr/bin/env python

# servo_key.py
# 2015-04-10
# Public Domain

#import time
#import curses
#import atexit

#import pigpio 

#

#MIN_PW = 500
#MID_PW = 1500
#MAX_PW = 2500

#NONE        = 0
#LEFT_ARROW  = 1
#RIGHT_ARROW = 2
#UP_ARROW    = 3
#DOWN_ARROW  = 4
#HOME        = 5
#QUIT        = 6

#def getch():
   #global in_escape, in_cursor
   #c = stdscr.getch()

   #key = NONE

   #if c == 27:
      #in_escape = True
      #in_cursor = False
   #elif c == 91 and in_escape:
      #in_cursor = True
   #elif c == 68 and in_cursor:
      #key = LEFT_ARROW
      #in_escape = False
   #elif c == 67 and in_cursor:
      #key = RIGHT_ARROW
      #in_escape = False
   #elif c == 65 and in_cursor:
      #key = UP_ARROW
      #in_escape = False
   #elif c == 66 and in_cursor:
      #key = DOWN_ARROW
      #in_escape = False
   #elif c == 72 and in_cursor:
      #key = HOME
      #in_escape = False
   #elif c == 113 or c == 81:
      #key = QUIT
   #else:
      #in_escape = False
      #in_cursor = False

   #return key

#def cleanup():
   #curses.nocbreak()
   #curses.echo()
   #curses.endwin()
   #pi.set_servo_pulsewidth(SERVO, 0)
   #pi.stop()

#pi = pigpio.pi('10.168.16.22')

#stdscr = curses.initscr()
#curses.noecho()
#curses.cbreak()

#atexit.register(cleanup) # Ensure original screen state is restored.

#in_escape = False
#in_cursor = False

#pulsewidth = MID_PW

#pi.set_servo_pulsewidth(SERVO, pulsewidth)

#while True:

   #time.sleep(0.01)

   #c = getch()

   #if c == QUIT:
      #break

   #pw = pulsewidth

   #if c == HOME:
      #pw = MID_PW # Stop.
   #elif c == UP_ARROW:
      #pw = MAX_PW # Fastest clockwise.
   #elif c == DOWN_ARROW:
      #pw = MIN_PW # Fastest anti-clockwise
   #elif c == LEFT_ARROW:
      #pw = pw - 100 # Shorten pulse.
      #if pw < MIN_PW:
         #pw = MIN_PW
   #elif c == RIGHT_ARROW:
      #pw = pw + 100 # Lengthen pulse.
      #if pw > MAX_PW:
         #pw = MAX_PW

   #if pw != pulsewidth:
      #pulsewidth = pw
      #pi.set_servo_pulsewidth(SERVO, pulsewidth)
