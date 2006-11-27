
# Specific to HRP-2
import hrp
import hstsetup
from hrp import *

# ---------------------------------------------------------------------------- #
# --- LOADING THE PLUGINS ---------------------------------------------------- #
# ---------------------------------------------------------------------------- #

ms = findPluginManager("motionsys")

# ---
# --- CLASSICAL PLUGIN: filter, ZMP, Seq.
# ---
ms.load("kfplugin")
kf = ms.create("kfplugin","kf","")
ms.load("ZMPsensor")
zmpsensor = ms.create("ZMPsensor","zmpsensor","")
ms.load("seqplay")
seq = seqpluginHelper.narrow(ms.create("seqplay","seq","-ORBconfig ../../../../../Common/orb.conf"))
seq.start()

# ---
# --- PATTERN GENERATOR
# ---
#waitInput("Wait before loading Pattern.");
ms.load("WalkGenJRL")
walk = ms.create("WalkGenJRL","walk","../../../../../PatternGeneratorJRL/src/PreviewControlParameters.ini /local/data/yoshida/src/OpenHRP/etc/HRP2JRL/ HRP2JRLmain.wrl ../../../../../PatternGeneratorJRL/src/HRP2Specificities.xml")
#walk = ms.create("WalkGenJRL","walk","PreviewControlParameters.ini /home/nmansard/src/OpenHRPFromTheRobot/etc/HRP2JRL/ HRP2JRLmain.wrl")

# ---
# --- STABILIZER
# ---
ms.load("hstabilizer")
st = ms.create("hstabilizer","st","")

# ---
# --- DYNAMICS
# ---
ms.load("dynamics")
dyn = ms.create("dynamics","dyn","-ORBconfig ../../../../../Common/orb.conf")
dyn.start()
# ---
# --- LOG
# ---
ms.load("logplugin")
log = LoggerPluginHelper.narrow(ms.create("logplugin","log",""))
log.sendMsg(":max-length 80")


# ---------------------------------------------------------------------------- #
# --- START THE PLUGINS ------------------------------------------------------ #
# ---------------------------------------------------------------------------- #

# --- 
# --- STABILIZER
# --- 
#waitInput("Push [Ok] to Start Stabilizer")
hstsetup.stsetup()
kf.start()
zmpsensor.start()
st.start()

# --- 
# --- PATTERN GENERATOR
# ---
#waitInput("Push [Ok] to Start the JRL Walking Pattern")
walk.start()
# walk mode : 0 for normal walking ; 
#             1 for walking with waistheight variation ; 
#             2 for walking with obstacle stepover
#             4 for no-upper-body-motion
walk.sendMsg(":walkmode 0")

walk.sendMsg(":omega 0.0")
walk.sendMsg(":stepheight 0.07")
walk.sendMsg(":singlesupporttime 0.7")
walk.sendMsg(":doublesupporttime 0.1")
walk.sendMsg(":armparameters 0.5")
walk.sendMsg(":LimitsFeasibility 0.0")
walk.sendMsg(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
walk.sendMsg(":TimeDistributeParameters 2.0 3.7 1.7 3.0")
walk.sendMsg(":UpperBodyMotionParameters -0.1 -1.0  0.0")
walk.sendMsg(":SetAlgoForZmpTrajectory PBW")
walk.sendMsg(":setpbwconstraint XY 0.07 0.05")
walk.sendMsg(":setpbwconstraint T 0.01 ")
walk.sendMsg(":setpbwconstraint N 150")


# -------------------------- -------------------------------------------------- #
# --- RUN -------------------------------------------------------------------- #
# ---------------------------------------------------------------------------- #

waitInputConfirm("Wait before going into half sitting")
seq.goHalfSitting(2.0)
seq.waitInterpolation()


# --- 
# --- START THE Walk
# --- 

waitInputConfirm("Wait before preparing stack step.")
walk.sendMsg(":stepseq 0.0 -0.095 0.0 0.2 0.19 0.0 0.2 -0.19 0.0 0.2  0.19 0.0 0.2 -0.19 0.0 0.2 0.19 0.0 0.0 -0.19 0.0")
# --- START THE WALK
waitInputConfirm("Wait before walking.")
log.start()
walk.sendMsg(":SendStackToControl");
walk.sendMsg(":synchronize")

waitInputConfirm("Wait the end of the walking.")
walk.sendMsg(":profile")
log.stop()
log.save("PBWWalking")
walk.sendMsg(":dumpzmpreel ZMPreeldump.dat")

