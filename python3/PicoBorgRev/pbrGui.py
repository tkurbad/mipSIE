#!/usr/bin/env python
# coding: latin-1

# Import library functions we need 
import PicoBorgRev
import Tkinter

# Setup the PicoBorg Reverse
global PBR
PBR = PicoBorgRev.PicoBorgRev()     # Create a new PicoBorg Reverse object
PBR.Init()                          # Set the board up (checks the board is connected)
PBR.ResetEpo()                      # Reset the stop switch (EPO) state
                                    # if you do not have a switch across the two pin header then fit the jumper

# Class respresenting the GUI dialog
class PicoBorgRev_tk(Tkinter.Tk):
    # Constructor (called when the object is first created)
    def __init__(self, parent):
        Tkinter.Tk.__init__(self, parent)
        self.parent = parent
        self.protocol("WM_DELETE_WINDOW", self.OnExit) # Call the OnExit function when user closes the dialog
        self.Initialise()

    # Initialise the dialog
    def Initialise(self):
        global PBR
        self.title('PicoBorg Reverse Example GUI')
        # Setup a grid of 2 sliders which command each motor output, plus a stop button for both motors
        self.grid()
        self.sld1 = Tkinter.Scale(self, from_ = +100, to = -100, orient = Tkinter.VERTICAL, command = self.sld1_move)
        self.sld1.set(0)
        self.sld1.grid(column = 0, row = 0, rowspan = 1, columnspan = 1, sticky = 'NSEW')
        self.sld2 = Tkinter.Scale(self, from_ = +100, to = -100, orient = Tkinter.VERTICAL, command = self.sld2_move)
        self.sld2.set(0)
        self.sld2.grid(column = 1, row = 0, rowspan = 1, columnspan = 1, sticky = 'NSEW')
        self.butOff = Tkinter.Button(self, text = 'All Off', command = self.butOff_click)
        self.butOff['font'] = ("Arial", 20, "bold")
        self.butOff.grid(column = 0, row = 1, rowspan = 1, columnspan = 2, sticky = 'NSEW')
        self.grid_columnconfigure(0, weight = 1)
        self.grid_columnconfigure(1, weight = 1)
        self.grid_rowconfigure(0, weight = 4)
        self.grid_rowconfigure(1, weight = 1)
        # Set the size of the dialog
        self.resizable(True, True)
        self.geometry('200x600')
        # Setup the initial motor state
        PBR.MotorsOff()

    # Called when the user closes the dialog
    def OnExit(self):
        global PBR
        # Turn drives off and end the program
        PBR.MotorsOff()
        self.quit()
  
    # Called when sld1 is moved
    def sld1_move(self, value):
        global PBR
        PBR.SetMotor1(float(value) / 100.0)

    # Called when sld2 is moved
    def sld2_move(self, value):
        global PBR
        PBR.SetMotor2(float(value) / 100.0)

    # Called when butOff is clicked
    def butOff_click(self):
        global PBR
        PBR.MotorsOff()
        self.sld1.set(0)
        self.sld2.set(0)

# if we are the main program (python was passed a script) load the dialog automatically
if __name__ == "__main__":
    app = PicoBorgRev_tk(None)
    app.mainloop()

