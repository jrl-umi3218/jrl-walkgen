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
walk = ms.create("WalkGenJRLIntegrate","walk","/home/grxuser/src/OpenHRP/PatternGeneratorJRL/src/PreviewControlParameters.ini /home/grxuser/src/OpenHRP/etc/HRP2JRL/ HRP2JRLmain.wrl")
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

seq.goHalfSitting(2.0)
seq.waitInterpolation()

waitInput("Push [Ok] to Start Stabilizer")
hstsetup.stsetup()
kf.start()
zmpsensor.start()
st.start()

waitInput("Push [Ok] to Start Our Great Walking Pattern")
walk.start()



waitInputConfirm("Push [Ok]to move the robot.")
log.start()
#seq.sendMsg(":joint-angles 0 0 -26 50 -24 0 0 0 -26 50 -24 0 0 0 0.0 0.0 14.813 -10.0 0 -30.0 0 0 10.0 14.813 10.0 0 -30.0 0 0 10.0 -10.0 10.0 -10.0 10.0 -10.0 -10.0 10.0 -10.0 10.0 -10.0 5.0")
#seq.sendMsg(":wait-interpolation")

# walk mode : 0 for normal walking ; 1 for walking with waistheight variation ; 2 for walking with obstacle stepover
walk.sendMsg(":walkmode 4")

walk.sendMsg(":omega 0.0")
walk.sendMsg(":stepheight 0.07")
walk.sendMsg(":singlesupporttime 0.78")
walk.sendMsg(":doublesupporttime 0.02")
walk.sendMsg(":armparameters 0.5")
walk.sendMsg(":LimitsFeasibility 0.0")
walk.sendMsg(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
walk.sendMsg(":TimeDistributeParameters 2.0 3.7 1.7 3.0")
walk.sendMsg(":UpperBodyMotionParameters -0.1 -1.0  0.0")

#uncomment this part if you want to step over obstacle and give the specific obstacle parameters	
waitInputConfirm("Push [Ok]to give o bstacle parameters")	 
#structure of this message ("position (frontmiddle at floor) x"   "position y "   "position z " "orientation"    "height"   "width"  "depth"   "boole if obstacle (1) else 0")	
walk.sendMsg(":obstacleparameters 1.0 0.0 0.0 0.0 0.19 1.0 0.025 1")
waitInputConfirm("Push [Ok] to start to walk")	 
#walk.sendMsg(":stepseq 0.0 -0.095 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.0 0.19 0.0")



#uncomment this part if you want the robot to walk with waist height variation
##waitInputConfirm("Push [Ok] to start walking on line")	 
#walk.sendMsg(":stepseq 0.0 -0.095 0.0 0.0 0.2 0.19 0.0 0.0 0.2 -0.19 0.0 -0.01 0.2 0.19 0.0 -0.05 0.2 -0.19 0.0 -0.10 0.2 0.19 0.0 -0.15 0.2 -0.19 0.0 -0.05 0.2 +0.19 0.0 0.0 0.0 -0.19 0.0 0.0")
#uncomment this one to start walking on a circle	
#waitInputConfirm("Push [Ok] to start circle")	 
walk.sendMsg(":stepseq 0.0 -0.095 0.0 0.0 0.2 0.19 10.0 0.0 0.2 -0.19 10.0 -0.01 0.2 0.19 10.0 -0.01 0.2 -0.19 10.0 -0.05 0.2 0.19 10.0 -0.05 0.2 -0.19 10.0 -0.05 0.2 0.19 10.0 -0.01 0.2 -0.19 10.0 -0.01 0.2 +0.19 10.0 0.0 0.2 -0.19 10.0 0.0 0.2 +0.19 10.0 -0.01 0.2 -0.19 10.0 -0.01 0.2 +0.19 10.0 -0.05 0.2 -0.19 10.0 -0.05 0.2 +0.19 10.0 -0.05 0.2 -0.19 10.0 -0.01 0.2 +0.19 10.0 -0.01 0.2 -0.19 10.0 0.0 0.2 0.19 10.0 0.0 0.2 -0.19 10.0 -0.01 0.2 0.19 10.0 -0.01 0.2 -0.19 10.0 -0.05 0.2 0.19 10.0 -0.05 0.2 -0.19 10.0 -0.05 0.2 0.19 10.0 -0.01 0.2 -0.19 10.0 -0.01 0.2 +0.19 10.0 0.0 0.2 -0.19 10.0 0.0 0.2 +0.19 10.0 -0.01 0.2 -0.19 10.0 -0.01 0.2 +0.19 10.0 -0.05 0.2 -0.19 10.0 -0.05 0.2 +0.19 10.0 -0.05 0.2 -0.19 10.0 -0.01 0.2 +0.19 10.0 -0.01 0.2 -0.19 10.0 0.0 0.2 +0.19 0.0 0.0 0.0 -0.19 0.0 0.0")	

#uncomment this part if you want the robot to walk in normal mode
#walk.sendMsg(":stepseq 0.0 -0.095 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.0 0.19 0.0")

waitInputConfirm("Wait before finishing the script.")
#ms.sendMsg(":wait 10")
#dyn.stop()
log.stop()
log.save("sim")
walk.sendMsg(":dumpzmpreel ZMPreeldump.dat")
print("Script finished")

