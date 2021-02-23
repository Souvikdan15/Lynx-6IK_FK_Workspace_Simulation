%%%%%%%%%%%%%%%%%%%Souvik Dan%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta1=0;
theta2=0;
theta3=0;
theta4=0;
theta5=0;
length1=1;
length2=3;
length3=3;
length4=1;
n=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for theta1= -pi/2:0.2:pi/2     
    for theta2= 0 : 0.2 : 3*pi/4 
        for theta3= -3*pi/4 : 0.2 : 0
            for theta4= -pi/2: 0.2 : pi/2
   
n=n+1;  
%dh =[theta  d an alpha]
dh1= [theta1 length1 0 deg2rad(90)];
dh2= [theta2 0 length2 0];
dh3= [theta3 0 length3 0];
dh4= [theta4 0 length4 deg2rad(-90)];
dh5= [theta5 0 0 0];


dtrans= dht(dh1) *dht(dh2) *dht(dh3) *dht(dh4) *dht(dh5);



xyz_cor(n,:) = [dtrans(1:3,4); theta1; theta2 ;theta3;theta4];
end
end
end
end


figure;
subplot(2,2,1)

scatter3(xyz_cor(:,1),xyz_cor(:,2),xyz_cor(:,3),0.5,xyz_cor(:,3));
xlabel('x')
ylabel('y')
zlabel('z')
title('Workspace in 3D space');
%axis([-10 10 -10 10 -10 10])
grid on

subplot(2,2,2)
scatter3(xyz_cor(:,1),xyz_cor(:,2),xyz_cor(:,3),0.5,xyz_cor(:,3));
view(90,90);
xlabel('x')
ylabel('y')
zlabel('z')
title('Workspace in X-Y plane');
%axis([-10 10 -10 10 -10 10])
grid on

subplot(2,2,3)
scatter3(xyz_cor(:,1),xyz_cor(:,2),xyz_cor(:,3),0.5,xyz_cor(:,3));
view(90,0);
xlabel('x')
ylabel('y')
zlabel('z')
title('Workspace in Y-Z plane')
%axis([-10 10 -10 10 -10 10])
grid on

subplot(2,2,4)
scatter3(xyz_cor(:,1),xyz_cor(:,2),xyz_cor(:,3),0.5,xyz_cor(:,3));
view(0,0);
xlabel('x')
ylabel('y')
zlabel('z')
title('Workspace in X-Z plane')
%axis([-10 10 -10 10 -10 10])
grid on


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%function to extract dh parameter%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
function dhparameter = dht(matrix)

an_1 = matrix(3);
alpha = matrix(4);
d_n = matrix(2);
theta=matrix(1);
sa= sin(alpha);
ca=cos(alpha);
st=sin(theta);
ct=cos(theta);

dhparameter = [ct    -ca*st    sa*st   an_1*ct;
               st     ca*ct    -sa*ct  an_1*st;
               0      sa        ca     d_n;
               0     0      0   1];
end