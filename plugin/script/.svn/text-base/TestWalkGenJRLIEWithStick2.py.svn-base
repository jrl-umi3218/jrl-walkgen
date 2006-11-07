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

ms.load("WalkGenJRLIntegrate")
walk = ms.create("WalkGenJRLIntegrate","walk","/home/stasse/src/OpenHRP/PatternGeneratorJRLIntegrateExperiment/src/PreviewControlParameters.ini /home/stasse/src/OpenHRP/etc/HRP2JRL_withbar/ HRP2JRLmain.wrl")
ms.load("hstabilizer")
st = ms.create("hstabilizer","st","")

ms.load("dynamics")
dyn = ms.create("dynamics","dyn","-ORBconfig../../../../../Common/orb.conf")
dyn.start()

ms.load("logplugin")
log = LoggerPluginHelper.narrow(ms.create("logplugin","log",""))

#waitInput("Push [Ok] to go half sitting")
print("Go half sitting")
log.add("kf")
log.add("st")
log.sendMsg(":max-length 80")

seq.goHalfSitting(2.0)
seq.waitInterpolation()

#waitInput("Push [Ok] to Start Stabilizer")
hstsetup.stsetup()
kf.start()
zmpsensor.start()
st.start()

#waitInput("Push [Ok] to Start JRL Walking Pattern Generator")
walk.start()



#waitInputConfirm("Push [Ok]to move the robot to the initial position.")
log.start()
seq.sendMsg(":joint-angles 0 0 -26 50 -24 0 0 0 -26 50 -24 0 0 0 0.0 0.0 12 -10.0 0 -40.0 0 0 8.2 12 10.0 12.78 -40.0 0 0 8.2 -10.0 10.0 -10.0 10.0 -10.0 -10.0 10.0 -10.0 10.0 -10.0 1.0")
seq.waitInterpolation()
#waitInputConfirm("Put the bar in the right hand, and push [Ok] to close the hand.")
seq.sendMsg(":joint-angles 0 0 -26 50 -24 0 0 0 -26 50 -24 0 0 0 0.0 0.0 12 -10.0 0 -40.0 0 0 1.0 12 10.0 12.78 -40.0 0 0 8.2 -1.0 1.0 -1.0 1.0 -1.0 -8.2 8.2 -8.2 8.2 -8.2 1.0")
seq.sendMsg(":wait-interpolation")

# walk mode : 0 for normal walking ; 1 for walking with waistheight variation ; 2 for walking with obstacle stepover
walk.sendMsg(":walkmode 1")



walk.sendMsg(":omega 0.0")
walk.sendMsg(":stepheight 0.07")
walk.sendMsg(":singlesupporttime 0.78")
walk.sendMsg(":doublesupporttime 0.02")
walk.sendMsg(":armparameters 0.5")
walk.sendMsg(":LimitsFeasibility 0.0")
walk.sendMsg(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
walk.sendMsg(":TimeDistributeParameters 2.0 3.5 1.0 3.0")
walk.sendMsg(":UpperBodyMotionParameters 0.0 -0.5 0.0")
walk.sendMsg(":readfilefromkw PartialModel.dat KWBarPath.pth")

#uncomment this part if you want to step over obstacle and give the specific obstacle parameters	
#waitInputConfirm("Push [Ok]to give o bstacle parameters")	 
walk.sendMsg(":obstacleparameters 1.0 0.0 0.0 0.0 0.05 1.0 0.05 1")
#waitInputConfirm("Push [Ok] to start to walk")	 

#uncomment this part if you want the robot to walk with waist height variation
#walk.sendMsg(":stepseq 0.0 -0.095 0.0 0.0 0.2 0.19 0.0 0.0 0.2 -0.19 0.0 0.0 0.2 0.19 0.0 0.0 0.2 -0.19 0.0 0.0 0.2 0.19 0.0 -0.05 0.2 -0.19 0.0 -0.10 0.2 0.19 0.0 -0.15 0.2 -0.19 0.0 -0.2 0.2 +0.19 0.0 -0.2 0.2 -0.19 0.0 -0.2 0.2 +0.19 0.0 -0.2 0.2 -0.19 0.0 -0.15 0.2 +0.19 0.0 -0.07 0.2 -0.19 0.0 0.0 0.0 0.19 0.0 0.0")
walk.sendMsg(":stepseq 0.0 -0.095 0.0 0.0 0.1 0.19 0.0 0.0 0.1 -0.19 0.0 0.0 0.1 0.19 0.0 0.0 0.1 -0.19 0.0 0.0 0.1 0.19 0.0 0.0 0.1 -0.19 0.0 0.0 0.1 0.19 0.0 0. 0.1 -0.19 0.0 0. 0.1 +0.19 0.0 0. 0.1 -0.19 0.0 0. 0.1 +0.19 0.0 0. 0.1 -0.19 0.0 0. 0.1 +0.19 0.0 0.0 0.1 -0.19 0.0 0.0 0.1 0.19 0.0 0.0 0.1 -0.19 0.0 0.0 0.1 0.19 0.0 0.0 0.1 -0.19 0.0 0.0 0.1 0.19 0.0 0.0 0.1 -0.19 0.0 0.0 0.1 0.19 0.0 0. 0.1 -0.19 0.0 0. 0.1 +0.19 0.0 0. 0.1 -0.19 0.0 0. 0.1 +0.19 0.0 0. 0.1 -0.19 0.0 0. 0.1 +0.19 0.0 0.0 0.1 -0.19 0.0 0.0 0.0 0.19 0.0 0.0")
waitInputConfirm("Wait before finishing the script.")

log.stop()
log.save("sim")
walk.sendMsg(":dumpzmpreel ZMPreeldump.dat")
print("Script finished")

