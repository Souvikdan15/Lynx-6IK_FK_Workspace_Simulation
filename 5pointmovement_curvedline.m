clc;
clear all;
close all;

a=0;
b=0;

steps=100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%INPUT POINTS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
orientation=-pi/2;
point1=[5 0 0];
p1IK=fullIK([point1,orientation]);
point2=[4.5 0 2.5];
p2IK=fullIK([point2 orientation]);
point3=[3.2 3.2 2.5];
p3IK=fullIK([point3,orientation]);
point4=[1 4.7 2.5];
p4IK=fullIK([point4,orientation]);
point5=[0 4.5 2.5];
p5IK=fullIK([point5,orientation]);
point6=[5 0 0];
p6IK=fullIK([point6,orientation]);


%%%%%%%%%%%%%%Joint for point 1 to 2%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
j11angles=linspace(p1IK(1),p2IK(1),steps);
j12angles=linspace(p1IK(2),p2IK(2),steps);
j13angles=linspace(p1IK(3),p2IK(3),steps);
j14angles=linspace(p1IK(4),p2IK(4),steps);
j15angles=linspace(p1IK(5),p2IK(5),steps);

%%%%%%%%%%%%%%Joint for point 2 to 3%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
j21angles=linspace(p2IK(1),p3IK(1),steps);
j22angles=linspace(p2IK(2),p3IK(2),steps);
j23angles=linspace(p2IK(3),p3IK(3),steps);
j24angles=linspace(p2IK(4),p3IK(4),steps);
j25angles=linspace(p2IK(5),p3IK(5),steps);

%%%%%%%%%%%%%%Joint for point 3 to 4%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
j31angles=linspace(p3IK(1),p4IK(1),steps);
j32angles=linspace(p3IK(2),p4IK(2),steps);
j33angles=linspace(p3IK(3),p4IK(3),steps);
j34angles=linspace(p3IK(4),p4IK(4),steps);
j35angles=linspace(p3IK(5),p4IK(5),steps);

%%%%%%%%%%%%%%Joint for point 4 to 5%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
j41angles=linspace(p4IK(1),p5IK(1),steps);
j42angles=linspace(p4IK(2),p5IK(2),steps);
j43angles=linspace(p4IK(3),p5IK(3),steps);
j44angles=linspace(p4IK(4),p5IK(4),steps);
j45angles=linspace(p4IK(5),p5IK(5),steps);

%%%%%%%%%%%%%%Joint for point 5 to 6%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
j51angles=linspace(p5IK(1),p6IK(1),steps);
j52angles=linspace(p5IK(2),p6IK(2),steps);
j53angles=linspace(p5IK(3),p6IK(3),steps);
j54angles=linspace(p5IK(4),p6IK(4),steps);
j55angles=linspace(p5IK(5),p6IK(5),steps);
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Finish %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

j1all=[j11angles',j12angles',j13angles',j14angles',j15angles'];
j2all=[j21angles',j22angles',j23angles',j24angles',j25angles'];
j3all=[j31angles',j32angles',j33angles',j34angles',j35angles'];
j4all=[j41angles',j42angles',j43angles',j44angles',j45angles'];
j5all=[j51angles',j52angles',j53angles',j54angles',j55angles'];

for a=1:steps
    joints1(:,:,a)=fullFK(j1all(a,:));
    endef1(a,:)=joints1(1,:,a);
    joints2(:,:,a)=fullFK(j2all(a,:));
    endef2(a,:)=joints2(1,:,a);
    joints3(:,:,a)=fullFK(j3all(a,:));
    endef3(a,:)=joints3(1,:,a);
    joints4(:,:,a)=fullFK(j4all(a,:));
    endef4(a,:)=joints4(1,:,a);
    joints5(:,:,a)=fullFK(j5all(a,:));
    endef5(a,:)=joints5(1,:,a);
end
%%%%%%%%%%%%%plot1link%%%%%%%%%%%%%%%%

for b=1:steps
scatter3(endef1(:,1),endef1(:,2),endef1(:,3),'.');
hold on
scatter3(endef2(:,1),endef2(:,2),endef2(:,3),'.');
hold on
scatter3(endef3(:,1),endef3(:,2),endef3(:,3),'.');
hold on
scatter3(endef4(:,1),endef4(:,2),endef4(:,3),'.');
hold on
scatter3(endef5(:,1),endef5(:,2),endef5(:,3),'.');
hold on
plot3(joints1(:,1,b),joints1(:,2,b),joints1(:,3,b),'ko-','linewidth',1);
axis([-2 8 -2 8 -2 8]);
grid on;
xlabel('x')
ylabel('y')
zlabel('z')
hold on;
pause(0.03);
hold off

end
%%%%%%%%%%%%%%%%%%%%%%plot2link%%%%%%%%%%%%%%%%%%%%
for c=1:steps
scatter3(endef1(:,1),endef1(:,2),endef1(:,3),'.');
hold on
scatter3(endef2(:,1),endef2(:,2),endef2(:,3),'.');
hold on
scatter3(endef3(:,1),endef3(:,2),endef3(:,3),'.');
hold on
scatter3(endef4(:,1),endef4(:,2),endef4(:,3),'.');
hold on
scatter3(endef5(:,1),endef5(:,2),endef5(:,3),'.');
hold on
plot3(joints2(:,1,c),joints2(:,2,c),joints2(:,3,c),'ko-','linewidth',1);
axis([-2 8 -2 8 -2 8]);
grid on;
xlabel('x')
ylabel('y')
zlabel('z')
hold on;
pause(0.03);
hold off

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%plot3line%%%%%%%%%%%%%%%%%%%%%%%%%%%
for d=1:steps
scatter3(endef1(:,1),endef1(:,2),endef1(:,3),'.');
hold on
scatter3(endef2(:,1),endef2(:,2),endef2(:,3),'.');
hold on
scatter3(endef3(:,1),endef3(:,2),endef3(:,3),'.');
hold on
scatter3(endef4(:,1),endef4(:,2),endef4(:,3),'.');
hold on
scatter3(endef5(:,1),endef5(:,2),endef5(:,3),'.');
hold on
plot3(joints3(:,1,d),joints3(:,2,d),joints3(:,3,d),'ko-','linewidth',1);
axis([-2 8 -2 8 -2 8]);
grid on;
xlabel('x')
ylabel('y')
zlabel('z')
hold on;
pause(0.03);
hold off

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%plot4line%%%%%%%%%%%%%%%%%%%
for e=1:steps
scatter3(endef1(:,1),endef1(:,2),endef1(:,3),'.');
hold on
scatter3(endef2(:,1),endef2(:,2),endef2(:,3),'.');
hold on
scatter3(endef3(:,1),endef3(:,2),endef3(:,3),'.');
hold on
scatter3(endef4(:,1),endef4(:,2),endef4(:,3),'.');
hold on
scatter3(endef5(:,1),endef5(:,2),endef5(:,3),'.');
hold on
plot3(joints4(:,1,e),joints4(:,2,e),joints4(:,3,e),'ko-','linewidth',1);
axis([-2 8 -2 8 -2 8]);
grid on;
xlabel('x')
ylabel('y')
zlabel('z')
hold on;
pause(0.03);
hold off

end
%%%%%%%%%%%%%%%%%%%%%%%%%plot5line%%%%%%%%%%%%%%%%%%%%%
for f=1:steps
scatter3(endef1(:,1),endef1(:,2),endef1(:,3),'.');
hold on
scatter3(endef2(:,1),endef2(:,2),endef2(:,3),'.');
hold on
scatter3(endef3(:,1),endef3(:,2),endef3(:,3),'.');
hold on
scatter3(endef4(:,1),endef4(:,2),endef4(:,3),'.');
hold on
scatter3(endef5(:,1),endef5(:,2),endef5(:,3),'.');
hold on
plot3(joints5(:,1,f),joints5(:,2,f),joints5(:,3,f),'ko-','linewidth',1);
axis([-2 8 -2 8 -2 8]);
grid on;
xlabel('x')
ylabel('y')
zlabel('z')
hold on;
pause(0.03);
hold off

end


function thetas=fullIK(xyzpTarget)

ll1=0;
ll2=3;
ll3=3;
ll4=1;
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
sinth3=-sqrt(1-(costh3)^2);
theta3=atan2(sinth3,costh3);

%theta 2
costh2=((pX4*(ll2+(ll3*cos(theta3))))+(pZ4*ll3*sin(theta3)))/(pX4^2+pZ4^2);
sinth2=sqrt(1-costh2^2);
theta2=atan2(sinth2,costh2);

%theta 4
theta4=pEP-theta2-theta3;

thetas=[theta1 theta2 theta3 theta4 0];

end

function joints=fullFK(angles)

tt1=angles(1);
tt2=angles(2);
tt3=angles(3);
tt4=angles(4);
tt5=angles(5);

ll1=0;
ll2=3;
ll3=3;
ll4=1;

dht1=[deg2rad(90) 0 0 tt1];
dht2=[0 ll2 0 tt2];
dht3=[0 ll3 0 tt3];
dht4=[deg2rad(-90) ll4 0 tt4];
dht5=[0 0 0 tt5];

DHmaster=DHT(dht1)*DHT(dht2)*DHT(dht3)*DHT(dht4)*DHT(dht5);
dhp4=DHT(dht1)*DHT(dht2)*DHT(dht3)*DHT(dht4);
dhp3=DHT(dht1)*DHT(dht2)*DHT(dht3);
dhp2=DHT(dht1)*DHT(dht2);
dhp1=DHT(dht1);

xyz1(1,:)=DHmaster(1:3,4);
xyz2(1,:)=dhp4(1:3,4);
xyz3(1,:)=dhp3(1:3,4);
xyz4(1,:)=dhp2(1:3,4);
xyz5(1,:)=dhp1(1:3,4);

joints=[xyz1(1,:);xyz2(1,:);xyz3(1,:);xyz4(1,:);xyz5(1,:)];

end

function DHTmatrix=DHT(ipDHpara)    %(an_1,alphan_1,dn,thetan)

an_1=ipDHpara(2);
alphan_1=ipDHpara(1);
dn=ipDHpara(3);
thetan=ipDHpara(4);

sa=sin(alphan_1);
st=sin(thetan);
ca=cos(alphan_1);
ct=cos(thetan);

DHTmatrix=[ct -ca*st sa*st an_1*ct;
           st ca*ct -sa*ct an_1*st;
           0 sa ca dn
           0 0 0 1];
end
