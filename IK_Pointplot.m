%%%%%%%%%%%%%%%%%%%Souvik Dan%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


IKinput=[76.4583   65  143 0 0] %%input of position X-Y-Z - Orientation
OPik=fullIK(IKinput)
Theta1 = rad2deg(OPik(:,1))
Theta2 = rad2deg(OPik(:,2))
Theta3 = rad2deg(OPik(:,3))
Theta4 = rad2deg(OPik(:,4))


function thetas=fullIK(xyzpTarget)

ll1=0;
ll2=150;
ll3=200;
ll4=100;
ll5=0;

pEX=xyzpTarget(1);
pEY=xyzpTarget(2);
pEZ=xyzpTarget(3);
pEP=xyzpTarget(4);

%theta 1
theta1=atan2(pEY,pEX);

%theta 3
pX4=sqrt(pEX^2+pEY^2)-((ll4+ll5)*cos(pEP));
pZ4=(pEZ-ll1)-((ll4+ll5)*sin(pEP));

costh3=(pX4^2+pZ4^2-ll2^2-ll3^2)/(2*ll2*ll3);
sinth3=sqrt(1-(costh3)^2);
theta3=atan2(sinth3,costh3);

%theta 2
costh2=((pX4*(ll2+(ll3*cos(theta3))))+(pZ4*ll3*sin(theta3)))/(pX4^2+pZ4^2);
sinth2=sqrt(1-costh2^2);
theta2=atan2(sinth2,costh2);

%theta 4
theta4=pEP-theta2-theta3;

thetas=[theta1 theta2 theta3 theta4 0];

end