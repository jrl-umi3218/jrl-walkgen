# Specific to HRP-2
import hrp
import hstsetup
from hrp import *

# Specific to JRL
# import jp.go.aist.jrl.vision as vision

# import jp.go.aist.jrl.sound as sound

ms = findPluginManager("motionsys")
# Connect to plugins on hrp2010c
ms.load("kfplugin")
kf = ms.create("kfplugin","kf","")
ms.load("ZMPsensor")
zmpsensor = ms.create("ZMPsensor","zmpsensor","")
ms.load("seqplay")
seq = seqpluginHelper.narrow(ms.create("seqplay","seq","-ORBconfig ../../../../../Common/orb.conf"))
seq.start()

ms.load("WalkGenJRL")
#walk = ms.create("WalkGenJRL","walk","PreviewControlParameters.ini /home/grxuser/src/OpenHRP/etc/HRP2JRL/ HRP2JRLmain.wrl HRP2Specificities.xml")
walk = ms.create("WalkGenJRL","walk","../../../../../PatternGeneratorJRL/src/PreviewControlParameters.ini /local/data/yoshida/src/OpenHRP/etc/HRP2JRL/ HRP2JRLmain.wrl ../../../../../PatternGeneratorJRL/src/HRP2Specificities.xml")
ms.load("hstabilizer")
st = ms.create("hstabilizer","st","")

ms.load("dynamics")
dyn = ms.create("dynamics","dyn","-ORBconfig ../../../../../Common/orb.conf")
dyn.start()

ms.load("logplugin")
log = LoggerPluginHelper.narrow(ms.create("logplugin","log",""))

# Connect to flite
#try:
#    obj = hrp.findObject("Festival")
#   flite =sound.FestivalForHRP2Helper.narrow(obj)
#except:
#GG    print "exception in finding Festival"


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

walk.sendMsg(":omega 0.0")
walk.sendMsg(":stepheight 0.07")
walk.sendMsg(":singlesupporttime 0.78")
walk.sendMsg(":doublesupporttime 0.02")
walk.sendMsg(":armparameters 0.5")
walk.sendMsg(":LimitsFeasibility 0.0")
walk.sendMsg(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
walk.sendMsg(":TimeDistributeParameters 2.0 3.5 1.0 3.0")
walk.sendMsg(":UpperBodyMotionParameters 0.0 -0.5 0.0")

StepOverBehavior=1
WesternGreetingsBehavior=1
PrepareVSBehavior=1

# Important : go back to half sitting position.
seq.goHalfSitting(2.0)
seq.waitInterpolation()

waitInputConfirm("Push [Ok] to have HRP-2 speaking")

#flite.ToSay("Greetings")
#flite.ToSay("Dear French visitors")
#flite.ToSay("My name is HRP-2.")
#flite.ToSay("but my creators calls me prometee.")

if StepOverBehavior==1:
  #************ Part for Stepping over ***************

  walk.sendMsg(":walkmode 2")
  walk.sendMsg(":TimeDistributeParameters 2.0 3.7 1.5 3.0")
  walk.sendMsg(":UpperBodyMotionParameters 0.0 -0.0 0.0")
  
  walk.sendMsg(":obstacleparameters 1.0 0.0 0.0 0.0 0.12 1.0 0.05 1")
  waitInputConfirm("Push [Ok] to start to walk")	 
  walk.sendMsg(":stepseq 0.0 -0.095 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0  0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.0 0.19 0.0 ")
  walk.sendMsg(":SendStackToControl")
  walk.sendMsg(":synchronize")
  waitInputConfirm("Remove the obstacle and push [Ok] to perform western greetings on the right")

if WesternGreetingsBehavior==1:
  #************************ WESTERN GREETINGS ***********************
  #western greet right

  seq.sendMsg(":joint-angles 0 0 -20 40 -17 0.0 0.0 0.0 -20 40 -17 0.0 28.0 13.0 0.0 0.0 70.0 0.0 50.0 -40.0 20.0 0.0 -10.0  -80.0 20.0 -60.0 -75.0 12.0 -20.0 -10.0 10.0 -10.0 10.0 -10 -10.0 -10.0 10.0 -10.0 10.0 -10.0 5.0")
  seq.sendMsg(":wait-interpolation")

  #flite.ToSay("Please to meet you sir")

  seq.sendMsg(":joint-angles 0 0 -20 40 -17 0.0 0.0 0.0 -20 40 -17 0.0 28.0 13.0 0.0 30.0 70.0 0.0 50.0 -40.0 0.0 0.0 -10.0  -60.0 20.0 -84.0 -100.0 12.0 -20.0 -10.0 10.0 -10.0 10.0 -10 -10.0 -10.0 10.0 -10.0 10.0 -10.0 5.0")
  seq.sendMsg(":wait-interpolation")

  waitInputConfirm("Push [Ok] to go in the half-sitting position")
  #back halfsitting

  seq.sendMsg(":joint-angles 0 0 -20 40 -20 0 0 0 -20 40 -20 0 0 0 0.0 0.0 15.0 -10.0 0 -30.0 0 0 10.0 15.0 10.0 0 -30.0 0 0 -10.0 10.0 -10.0 10.0 -10.0 10.0 10.0 -10.0 10.0 -10.0 10.0 5.0")
  seq.sendMsg(":wait-interpolation")


  ##flite.ToSay(" I can also play music.")
  waitInputConfirm("Wait before western greeting left")
  #western greet left

  seq.sendMsg(":joint-angles 0 0 -20 40 -17 0.0 0.0 0.0 -20 40 -17 0.0 -28.0 13.0 0.0 0.0 -80.0 -20.0 60.0 -75.0 12.0 -20.0 -10.0  70.0 0.0 -50.0 -40.0 0.0 0.0 -10.0 10.0 -10.0 10.0 -10 -10.0 -10.0 10.0 -10.0 10.0 -10.0 5.0")
  seq.sendMsg(":wait-interpolation")


  seq.sendMsg(":joint-angles 0 0 -20 40 -17 0.0 0.0 0.0 -20 40 -17 0.0 -28.0 13.0 0.0 30.0 -60.0 -20.0 84.0 -100.0 12.0 -20.0 -10.0  70.0 0.0 -50.0 -40.0 0.0 0.0 -10.0 10.0 -10.0 10.0 -10 -10.0 -10.0 10.0 -10.0 10.0 -10.0 5.0")
  seq.sendMsg(":wait-interpolation")

  #back halfsitting

  seq.sendMsg(":joint-angles 0 0 -20 40 -20 0 0 0 -20 40 -20 0 0 0 0.0 0.0 15.0 -10.0 0 -30.0 0 0 10.0 15.0 10.0 0 -30.0 0 0 -10.0 10.0 -10.0 10.0 -10.0 10.0 10.0 -10.0 10.0 -10.0 10.0 5.0")
  seq.sendMsg(":wait-interpolation")
	
#*************** Prepare for Visual Servoing *********************/

if PrepareVSBehavior==1:
  waitInputConfirm("Push [Ok] to turn for Visual Servoing")	 

  walk.sendMsg(":walkmode 0"); 
  walk.sendMsg(":stepseq 0.0 -0.095 0.0 -0.2 0.19 0.0 -0.2 -0.19 0.0 -0.2 0.19 0.0 -0.2 -0.19 0.0 -0.1 0.19 0.0 0.0 -0.19 10.0 0.0 0.19 10.0 0.0 -0.19 10.0 0.0 0.19 10.0 0.0 -0.19 10.0 0.0 0.19 10.0 0.0 -0.19 10.0 0.0 0.19 10.0 0.0 -0.19 10.0 0.0 0.19 0.0");
  walk.sendMsg(":SendStackToControl")
  walk.sendMsg(":synchronize")


#************* Fin de la demo ***********************
waitInputConfirm("Demo finished ")
log.stop()
log.save("PartialDemoChambreDeCommerce")
walk.sendMsg(":dumpzmpreel ZMPreeldump.dat")
print("Script finished")

