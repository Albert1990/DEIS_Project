from utils import EncoderPulses, RobotPos
import math
import numpy as np

firstRun = True
lastEncoderPulses = EncoderPulses()
Nbr_Puls_MM = (128*internalGearRatio*externalGearRatio*4)/(M_PI*wheelDiameter)
def calculatePosition(currentEncoderPulses):
    if firstRun:
        lastEncoderPulses = currentEncoderPulses
        return 0
    ddr = pulsesToMillimeter(currentEncoderPulses.right - lastEncoderPulses.right)
	ddl = pulsesToMillimeter(currentEncoderPulses.left - lastEncoderPulses.left)
    currentRobotPos = robot.getRobotPos()
    C = currentRobotPos.c
    newP = updatePosition(currentRobotPos, ddr, ddl, C)
    newP.C = C
    # robot.setRobotPos(newP)
    lastEncoderPulses = currentEncoderPulses
    return newPos

def updatePosition(oldP, dDr, dDl, CovM):
    dD= (dDr+dDl)/2
    dA=(dDr-dDl)/wheelBase

    
    dX=dD*math.cos(oldP.Theta+(dA/2))
    dY=dD*math.sin(oldP.Theta+(dA/2))

    newP = RobotPos()
    newP.X=oldP.X+dX
    newP.Y=oldP.Y+dY
    newP.Theta=fmod(oldP.Theta+dA,2*M_PI)

    # calculate the new covM 3*3
    Cxya_old = np.zeros((3,3))
    Cxya_old = CovM
    Cu = np.zeros((2,2))
    Cu = np.array([[pow(SIGMAr,2)+pow(SIGMAl,2))/4, (pow(SIGMAr,2)-pow(SIGMAl,2))/2*wheelBase], [(pow(SIGMAr,2)-pow(SIGMAl,2))/2*wheelBase, (pow(SIGMAr,2)+pow(SIGMAl,2))/pow(wheelBase,2)]])
    Axya = np.array([[1, 0, -dD*sin(oldP.Theta+(dA/2))], [0, 1, dD*cos(oldP.Theta+(dA/2))], [0, 0, 1])
    Au = np.array([[cos(oldP.Theta+(dA/2)), -dD/2*sin(oldP.Theta+(dA/2))], [sin(oldP.Theta+(dA/2)), dD/2*cos(oldP.Theta+(dA/2))], [0, 1]]);
    CovM = Axya*Cxya_old*Axya.adjoint() + Au*Cu*Au.adjoint();
    return newP;

def pulsesToMillimeter(pulsesCount):
	return pulsesCount/Nbr_Puls_MM