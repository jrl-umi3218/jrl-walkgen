plot "TestFGPI.dat" u ($0/200):2 w l t "CoM", "TestFGPI.dat" u ($0/200):8 w l t "ZMP", "TestFGPI.dat" u ($0/200):10 w l t "Left Foot", "TestFGPI.dat" u ($0/200):16 w l t "Right Foot"
set title "ZMP, Com, Feet position along the X-axis"
set xlabel "Time (s)"
set ylabel "Position (m)"
set xrange [9.5:12.5]
set yrange [-0.1:0.5]
replot
