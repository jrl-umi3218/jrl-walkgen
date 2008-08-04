 plot "AnalyticalZMPCOGTrajectoryY_000.dat" u 1:3 w l t "ZMP Initial", \
      "AnalyticalZMPCOGTrajectoryY_000.dat" u 1:2 w l t "CoM Initial", \
      "AnalyticalZMPCOGTrajectoryY_001_Bump.dat" u 1:3 w l t "ZMP Time Shift", \
      "AnalyticalZMPCOGTrajectoryY_001_Bump.dat" u 1:2 w l t "CoM Time Shift", \
      "AnalyticalZMPCOGTrajectoryY_001.dat" u 1:3 w l t "ZMP Fluc. red.", \
      "AnalyticalZMPCOGTrajectoryY_001.dat" u 1:2 w l lc rgb 'brown' t "CoM Fluc. red."
 set xlabel "Time (s)"
 set ylabel "Position (m)"
 set title "Fluctuation reduction by preview control"