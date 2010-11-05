plot "TestFGPI.dat" u ($0/200):3 w l t "CoM", "TestFGPI.dat" u ($0/200):6 w l t "ZMP", "TestFGPI.dat" u ($0/200):8 w l t "Left Foot", "TestFGPI.dat" u ($0/200):14 w l t "Right Foot"
set title "ZMP, Com, Feet position along the Y-axis"
set xlabel "Time (s)"
set ylabel "Position (m)"
set xrange [9:15]
set yrange [-0.25:2.0]
replot
