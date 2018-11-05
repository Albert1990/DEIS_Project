from utils import EncoderPulses, RobotPos
import math
import numpy as np
import consts
import math

firstRun = True
lastEncoderPulses = EncoderPulses()
def calculatePosition(currentEncoderPulses, currentRobotPos):
    global firstRun
    global lastEncoderPulses
    print(currentEncoderPulses.left, currentEncoderPulses.right)
    if firstRun:
        lastEncoderPulses = currentEncoderPulses
        firstRun = False
        return False
    else:
        ddr = pulsesToMillimeter(currentEncoderPulses.right - lastEncoderPulses.right)
        ddl = pulsesToMillimeter(currentEncoderPulses.left - lastEncoderPulses.left)
        print('ddl: %d, ddr: %d' % (ddl, ddr))
        C = currentRobotPos.C
        newP = updatePosition(currentRobotPos, ddr, ddl, C)
        # newP.C = C
        lastEncoderPulses = currentEncoderPulses
        return newP

def updatePosition(oldP, dDr, dDl, CovM):
    dD= (dDr+dDl)/2
    dA= (dDr-dDl)/consts.wheelBase

    
    dX=dD*math.cos(oldP.Theta+(dA/2))
    dY=dD*math.sin(oldP.Theta+(dA/2))

    newP = RobotPos()
    newP.X=oldP.X+dX
    newP.Y=oldP.Y+dY
    newP.Theta= math.fmod(oldP.Theta+dA,2*math.pi)

    # # calculate the new covM 3*3
    # Cxya_old = np.zeros((3,3))
    # Cxya_old = CovM
    # Cu = np.zeros((2,2))
    # Cu = np.array([[pow(SIGMAr,2)+pow(SIGMAl,2))/4, (pow(SIGMAr,2)-pow(SIGMAl,2))/2*wheelBase], [(pow(SIGMAr,2)-pow(SIGMAl,2))/2*wheelBase, (pow(SIGMAr,2)+pow(SIGMAl,2))/pow(wheelBase,2)]])
    # Axya = np.array([[1, 0, -dD*sin(oldP.Theta+(dA/2))], [0, 1, dD*cos(oldP.Theta+(dA/2))], [0, 0, 1])
    # Au = np.array([[cos(oldP.Theta+(dA/2)), -dD/2*sin(oldP.Theta+(dA/2))], [sin(oldP.Theta+(dA/2)), dD/2*cos(oldP.Theta+(dA/2))], [0, 1]]);
    # CovM = Axya*Cxya_old*Axya.adjoint() + Au*Cu*Au.adjoint();
    return newP

def pulsesToMillimeter(pulsesCount):
	return pulsesCount/consts.Nbr_Puls_MM