set title "Position of the feet"
set xlabel "X (m)"
set ylabel "Y (m)"
splot "DebugDataLong.txt" u 15:16:17 w l t "Left Foot",  "DebugDataLong.txt" u 18:19:20 w l t "Right Foot"
