plot "TestFGPI.dat" u ($0/200):6 w l t "dCoM", "TestFGPI.dat" u ($0/200):9 w l t "ZMP", "TestFGPI.dat" u ($0/200):11 w l t "Left Foot", "TestFGPI.dat" u ($0/200):17 w l t "Right Foot"
set title "ZMP, Com, Feet position along the Y-axis"
set xlabel "Time (s)"
set ylabel "Position (m)"
set xrange [9:15]
set yrange [-0.25:0.25]
replot
