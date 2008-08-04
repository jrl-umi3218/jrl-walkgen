set title "After second loop of control"
set xlabel "Time (iteration numbers x 0.005)"
set ylabel "meter"
plot "DebugDataLong.txt" u 2 w l t "Desired ZMP",  "DebugDataLong.txt" u 4 w l t "Multibody ZMP"
