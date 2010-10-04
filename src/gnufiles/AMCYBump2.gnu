 plot "AnalyticalZMPCOGTrajectoryY_000.dat" u 1:3 w l t "ZMP Initial", \
      "AnalyticalZMPCOGTrajectoryY_000.dat" u 1:2 w l t "CoM Initial", \
      "AnalyticalZMPCOGTrajectoryY_001_Bump.dat" u 1:3 w l t "ZMP First Correction", \
      "AnalyticalZMPCOGTrajectoryY_001_Bump.dat" u 1:2 w l t "CoM First Correction"
