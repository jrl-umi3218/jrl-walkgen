iPu=fscanfMat("iPu.dat");
Pu=fscanfMat("Pu.dat");
b=fscanfMat("lb.dat");
ZMPRef=fscanfMat("CZMPRef.dat");

i2Pu= [ [iPu' zeros(75,75)]; [zeros(75,75) iPu'];];

DPu=fscanfMat("DPu.dat");
D=DPu*i2Pu;

w=D*ZMPRef'+b';
v=min(w)
if v<0
  error("ZMP Ref trajectory not good");
end
Vinit=fscanfMat("InitialSolution.dat");

DPx=fscanfMat("DPx.dat");
w2=DPu*Vinit'+DPx';
v2=min(w2)

Px=fscanfMat("Px.dat");
Px2=[[Px zeros(75,3)];[zeros(75,3) Px];];
XkYk=fscanfMat("XkYk.dat");

Vinitr=-i2Pu*Px2*XkYk'+i2Pu*ZMPRef';
w3=DPu*Vinitr+DPx';
v3=min(w3)

iLQ=fscanfMat("iLQ.dat");
LQ=fscanfMat("LQ.dat");
Pur=pinv(i2Pu)*LQ';

iPur=pinv(Pur);
Vinitr2=-iPur*Px2*XkYk'+iPur*ZMPRef';
w4=D*(Pur*Vinitr2+Px2*XkYk')+b';
v4=min(w4)


Vinitr3=-LQ'*iPur*Px2*XkYk'+LQ'*iPur*ZMPRef';
w5=D*(Pur*iLQ'*Vinitr3+Px2*XkYk')+b';
w6=DPu*Vinitr3+DPx';
w7=D*(Pur*iLQ'*Vinitr+Px2*XkYk')+b';

v5=min(w5)
v6=min(w6)
v7=min(w7)

DPxr=D*Px2*XkYk'+b';

