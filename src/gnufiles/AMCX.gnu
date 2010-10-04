 set xlabel "Time (s)"
 set ylabel "Position (m)"
 set title "Changing step landing position at t=2.4 s "
 plot "AnalyticalZMPCOGTrajectoryX_000.dat" u 1:3 w l t "ZMP Initial Trajectory", \
      "AnalyticalZMPCOGTrajectoryX_000.dat" u 1:2 w l t "CoM Initial Trajectory", \
      "AnalyticalZMPCOGTrajectoryX_001.dat" u 1:3 w l t "ZMP New Trajectory", \
      "AnalyticalZMPCOGTrajectoryX_001.dat" u 1:2 w l t "CoM New Trajectory"
