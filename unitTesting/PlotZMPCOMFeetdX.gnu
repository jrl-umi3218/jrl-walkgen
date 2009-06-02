plot "TestFGPI.dat" u ($0/200):5 w l t "dCoM", "TestFGPI.dat" u ($0/200):2 w l t "CoM", 0.3 w l t "0.3 m/s"
set title "ZMP, Com, Feet position along the X-axis"
set xlabel "Time (s)"
set ylabel "Position (m)"
set xrange [9.5:12.5]
set yrange [-0.1:10]
replot
