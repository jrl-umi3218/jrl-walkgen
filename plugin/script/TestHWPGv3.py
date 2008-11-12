# Specific to HRP-2
import hrp
import hstsetup
from hrp import *


ms = findPluginManager("motionsys")
# Connect to plugins on hrp2010c
ms.load("kfplugin")
kf = ms.create("kfplugin","kf","")
ms.load("seqplay")
seq = SequencePlayerHelper.narrow(ms.create("seqplay","seq","-ORBconfig../../../../../Common/orb.conf"))
seq.start()

ms.load("WalkGenJRL")
walk = ms.create("WalkGenJRL","walk","../etc/PreviewControlParameters.ini ../model/ HRP2JRLmain.wrl ../etc/HRP2Specificities.xml ../etc/HRP2LinkJointRank.xml")
ms.load("hstabilizer")
st = ms.create("hstabilizer","st","")


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
hstsetup.stsetup(st)
kf.start()
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


if WesternGreetingsBehavior==1:
  #************************ WESTERN GREETINGS ***********************
  #western greet right
  waitInputConfirm("Push [Ok] to have HRP-2 perform western greetings")
  print("Western Greetings")	
  seq.sendMsg(":joint-angles 0 0 -20 40 -17 0.0 0.0 0.0 -20 40 -17 0.0 0.0 13.0 0.0 0.0 70.0 0.0 50.0 -40.0 20.0 0.0 -10.0  -80.0 20.0 -60.0 -75.0 12.0 -20.0 -10.0 10.0 -10.0 10.0 -10 -10.0 -10.0 10.0 -10.0 10.0 -10.0 5.0")
  seq.sendMsg(":wait-interpolation")


  seq.sendMsg(":joint-angles 0 0 -20 40 -17 0.0 0.0 0.0 -20 40 -17 0.0 0.0 13.0 0.0 30.0 70.0 0.0 50.0 -40.0 0.0 0.0 -10.0  -60.0 20.0 -84.0 -100.0 12.0 -20.0 -10.0 10.0 -10.0 10.0 -10 -10.0 -10.0 10.0 -10.0 10.0 -10.0 5.0")
  seq.sendMsg(":wait-interpolation")

  waitInputConfirm("Push [Ok] to go in the half-sitting position")
  #back halfsitting

  seq.sendMsg(":joint-angles 0 0 -20 40 -20 0 0 0 -20 40 -20 0 0 0 0.0 0.0 15.0 -10.0 0 -30.0 0 0 10.0 15.0 10.0 0 -30.0 0 0 -10.0 10.0 -10.0 10.0 -10.0 10.0 10.0 -10.0 10.0 -10.0 10.0 5.0")
  seq.sendMsg(":wait-interpolation")


 
#*************** Prepare for Going backward. *********************/

if PrepareVSBehavior==1:
  waitInputConfirm("Push [Ok] to prepare for Backward Motion + 90 deg. Rotation on the right")	 
  print("Prepare for Visual Servoing ")	 
  walk.sendMsg(":parsecmd :walkmode 0"); 
  #Going to the right.
  walk.sendMsg(":stepseq 0.0 -0.095 0.0 -0.2 0.19 0.0 -0.2 -0.19 0.0 -0.2 0.19 0.0 -0.2 -0.19 0.0 -0.1 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 0.0");
  # Goind to the left
  #walk.sendMsg(":parsecmd :stepseq 0.0 -0.095 0.0 -0.2 0.19 0.0 -0.2 -0.19 0.0 -0.2 0.19 0.0 -0.2 -0.19 0.0 -0.1 0.19 0.0 0.0 -0.19 10.0 0.0 0.19 10.0 0.0 -0.19 10.0 0.0 0.19 10.0 0.0 -0.19 10.0 0.0 0.19 10.0 0.0 -0.19 10.0 0.0 0.19 10.0 0.0 -0.19 10.0 0.0 0.19 0.0");
 	
  walk.sendMsg(":SendStackToControl")
  walk.sendMsg(":synchronize")
  walk.sendMsg(':profile')

#*************** Prepare for other walk *********************/

if PreparePlanification==1:
   print("Prepare for planification");
   waitInputConfirm("Push [Ok] to turn 120 deg. on the right and 1m and 90.0 deg. on the right. ")	 
   walk.sendMsg(":parsecmd :walkmode 0");
   walk.sendMsg(":stepseq 0.0 -0.095 0.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 0.0");
   walk.sendMsg(":SendStackToControl")
   walk.sendMsg(":synchronize")
   walk.sendMsg(":parsecmd :stepseq 0.0 -0.095 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.15 -0.19 0.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0  0.0 -0.19 0.0");
   walk.sendMsg(":SendStackToControl")
   walk.sendMsg(":synchronize")
   walk.sendMsg(':profile')


if StepOver2Behavior==1:
  #************ Part for Stepping over ***************
  print("Step Over Behavior")
  walk.sendMsg(":parsecmd :walkmode 2")
  walk.sendMsg(":parsecmd :TimeDistributeParameters 2.0 3.7 1.5 3.0")
  walk.sendMsg(":parsecmd :UpperBodyMotionParameters 0.0 -0.0 0.0")
  
  walk.sendMsg(":parsecmd :obstacleparameters 1.0 0.0 0.0 0.0 0.12 1.0 0.05 1")
  waitInputConfirm("WARNING : Push [Ok] to perform stepping over")	 
  walk.sendMsg(":stepseq 0.0 -0.095 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0  0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.0 0.19 0.0")
  walk.sendMsg(":SendStackToControl")
  walk.sendMsg(":synchronize")
  waitInputConfirm("Remove the obstacle and push [Ok]")

#************ Part for Planification ***************
if Planification==1:
   print("Planification");	
   waitInputConfirm("Push [Ok] to move the robot to the initial position.")
   seq.sendMsg(":joint-angles 0 0 -26 50 -24 0 0 0 -26 50 -24 0 0 0 0.0 0.0 12 -10.0 0 -40.0 0 0 8.2 12 10.0 12.78 -40.0 0 0 8.2 -10.0 10.0 -10.0 10.0 -10.0 -10.0 10.0 -10.0 10.0 -10.0 1.0")
   seq.waitInterpolation()
   waitInputConfirm("Put the bar in the right hand, and push [Ok] to close the hand.")
   seq.sendMsg(":joint-angles 0 0 -26 50 -24 0 0 0 -26 50 -24 0 0 0 0.0 0.0 12 -10.0 0 -40.0 0 0 1.0 12 10.0 12.78 -40.0 0 0 8.2 -1.0 1.0 -1.0 1.0 -1.0 -8.2 8.2 -8.2 8.2 -8.2 1.0")
   seq.sendMsg(":wait-interpolation")

   walk.sendMsg(":parsecmd :walkmode 3")
   walk.sendMsg(":parsecmd :readfilefromkw PartialModel.dat KWBarPath.pth")

   walk.sendMsg(":parsecmd :stepseq 0.0 -0.095 0.0 0.0 0.2 0.19 0.0 0.0 0.2 -0.19 0.0 0.0 0.2 0.19 0.0 0.0 0.2 -0.19 0.0 0.0 0.2 0.19 0.0 -0.05 0.2 -0.19 0.0 -0.10 0.2 0.19 0.0 -0.15 0.2 -0.19 0.0 -0.2 0.2 +0.19 0.0 -0.2 0.2 -0.19 0.0 -0.2 0.2 +0.19 0.0 -0.2 0.2 -0.19 0.0 -0.15 0.2 +0.19 0.0 -0.07 0.2 -0.19 0.0 0.0 0.0 0.19 0.0 0.0")

   waitInputConfirm("Push [Ok] to start to walk")	 
   walk.sendMsg(":parsecmd :SendStackToControl")
   walk.sendMsg(":parsecmd :synchronize")

   waitInputConfirm("Push [Ok] to open the hand")	 
   seq.sendMsg(":joint-angle RARM_JOINT6  8.2")
   seq.waitInterpolation()
   waitInputConfirm("Push [Ok] to go half-sitting")	 
   seq.goHalfSitting(2.0)
   seq.waitInterpolation()

#************* Face again the person  ***********************
if PrepareFin==1:
   waitInputConfirm("Wait before facing the person")
   walk.sendMsg(":parsecmd :walkmode 0");
   walk.sendMsg(":parsecmd :stepseq 0.0 -0.095 0.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 -10.0 0.0 -0.19 -10.0 0.0 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.0 -0.19 0.0");
   walk.sendMsg(":parsecmd :SendStackToControl")
   walk.sendMsg(":parsecmd :synchronize")
   	
if FinishBehavior==1:
   waitInputConfirm("Wait before japanese greeting front")

   #japanese bow

   seq.sendMsg(":joint-angles 0 0 -20 40 -15 0 0 0 -20 40 -15 0 0 25 0.0  0.0 -5.0 -10.0 20 -30.0 0 0 -10.0 -5.0 10.0 -20 -30.0 0 0 -10.0 10.0 -10.0 10.0 -10 -10.0 -10.0 10.0 -10.0 10.0 -10.0 5.0")
   seq.sendMsg(":wait-interpolation")

   seq.sendMsg(":joint-angles 0 0 -20 40 -15 0 0 0 -20 40 -15 0 0 25 0.0 30.0 -5.0 -10.0 20 -30.0 0 0 -10.0 -5.0 10.0 -20 -30.0 0 0 -10.0 10.0 -10.0 10.0 -10 -10.0 -10.0 10.0 -10.0 10.0 -10.0 5.0")
   seq.sendMsg(":wait-interpolation")

   waitInputConfirm("Wait before coming back to half-sitting")
   seq.goHalfSitting(5.0)
   seq.waitInterpolation()

   #back halfsitting

#************* Fin de la demo ***********************

waitInputConfirm("Demo finished - Press [Ok] to log")
log.stop()
log.save("TestHWPGv2")
walk.sendMsg(":parsecmd :dumpzmpreel ZMPreeldump.dat")
print("Script finished")


