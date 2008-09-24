# Specific to HRP-2
import hrp
import hstsetup
from hrp import *


ms = findPluginManager("motionsys")
# Connect to plugins on hrp2010c
ms.load("kfplugin")
kf = ms.create("kfplugin","kf","")
ms.load("ZMPsensor")
zmpsensor = ms.create("ZMPsensor","zmpsensor","")
ms.load("seqplay")
seq = seqpluginHelper.narrow(ms.create("seqplay","seq","-ORBconfig../../../../../Common/orb.conf"))
seq.start()

ms.load("WalkGenJRL")
walk = ms.create("WalkGenJRL","walk","PreviewControlParameters.ini /home/stasse/src/OpenHRP/etc/HRP2JRL/ HRP2JRLmain.wrl HRP2Specificities.xml HRP2LinkJointRank.xml")
ms.load("hstabilizer")
st = ms.create("hstabilizer","st","")

ms.load("dynamics")
dyn = ms.create("dynamics","dyn","-ORBconfig../../../../../Common/orb.conf")
dyn.start()

ms.load("logplugin")
log = LoggerPluginHelper.narrow(ms.create("logplugin","log",""))

waitInput("Push [Ok] to go half sitting")
print("Go half sitting")
log.add("kf")
log.add("st")
log.sendMsg(":max-length 80")
log.start()

seq.goHalfSitting(2.0)
seq.waitInterpolation()

#waitInput("Push [Ok] to Start Stabilizer")
hstsetup.stsetup()
kf.start()
zmpsensor.start()
st.start()

#waitInput("Push [Ok] to Start JRL Walking Pattern Generator")
walk.start()

# walk mode : 0 for normal walking ; 1 for walking with waistheight variation ; 2 for walking with obstacle stepover

walk.sendMsg(":parsecmd :omega 0.0")
walk.sendMsg(":parsecmd :stepheight 0.07")
walk.sendMsg(":parsecmd :singlesupporttime 0.78")
walk.sendMsg(":parsecmd :doublesupporttime 0.02")
walk.sendMsg(":parsecmd :armparameters 0.5")
walk.sendMsg(":parsecmd :LimitsFeasibility 0.0")
walk.sendMsg(":parsecmd :ZMPShiftParameters 0.015 0.015 0.015 0.015")
walk.sendMsg(":parsecmd :TimeDistributeParameters 2.0 3.5 1.0 3.0")
walk.sendMsg(":parsecmd :UpperBodyMotionParameters 0.0 -0.5 0.0")
walk.sendMsg(":parsecmd :SetAlgoForZmpTrajectory Kajita")

# Switches for the behaviors.
TalkingIntroduction=0
WesternGreetingsBehavior=0
PrepareVSBehavior=1
PreparePlanification=0
StepOver2Behavior=0
Planification=0
PrepareFin=0
FinishBehavior=1

#VisionSimulation=1
VisionSimulation=0

# Important : go back to half sitting position.
seq.goHalfSitting(2.0)
seq.waitInterpolation()




 #*************** Prepare for Going forward. *********************/


waitInputConfirm("Push [Ok] to prepare for going forward and start on line sequencing")	 
walk.sendMsg(":parsecmd :walkmode 0"); 
#Going to the right.
walk.sendMsg(":parsecmd :StartOnLineStepSequencing 0.0 0.105 0.0 0.2 -0.21 0.0 0.2 0.21 0.0 0.2 -0.21 0.0")
walk.sendMsg(":SendStackToControl")

waitInputConfirm("Push [Ok] to send a step.")	 
walk.sendMsg(":parsecmd :addstandardonlinestep 0.2 0.0 0.0")

waitInputConfirm("Push [Ok] to prepare for stopping on line sequencing")	 
walk.sendMsg(":parsecmd :StopOnLineStepSequencing");


#************* Fin de la demo ***********************

waitInputConfirm("Demo finished - Press [Ok] to log")
log.stop()
log.save("TestOnLineKajita")
walk.sendMsg(":parsecmd :dumpzmpreel ZMPreeldump.dat")
print("Script finished")


