        clc;
        clear all;
        close all;

        a=0;

        point1=[5 0 0];
        point2=[4.5 0 2.5];
        point3=[3.2 3.2 2.5];
        point4=[1 4.7 2.5];
        point5=[0 4.5 2.5];
        point6=[5 0 0];
        orientation=-pi/2;

        steps=100;

        x12points=linspace(point1(1),point2(1),steps);
        y12points=linspace(point1(2),point2(2),steps);
        z12points=linspace(point1(3),point2(3),steps);

        x23points=linspace(point2(1),point3(1),steps);
        y23points=linspace(point2(2),point3(2),steps);
        z23points=linspace(point2(3),point3(3),steps);

        x34points=linspace(point3(1),point4(1),steps);
        y34points=linspace(point3(2),point4(2),steps);
        z34points=linspace(point3(3),point4(3),steps);

        x45points=linspace(point4(1),point5(1),steps);
        y45points=linspace(point4(2),point5(2),steps);
        z45points=linspace(point4(3),point5(3),steps);

        x56points=linspace(point5(1),point6(1),steps);
        y56points=linspace(point5(2),point6(2),steps);
        z56points=linspace(point5(3),point6(3),steps);

        scatter3(x12points,y12points,z12points,'.');
        scatter3(x23points,y23points,z23points,'.');



        for a=1:steps;

            scatter3(x12points,y12points,z12points,'.');
            hold on
            scatter3(x23points,y23points,z23points,'.');
            hold on
            scatter3(x34points,y34points,z34points,'.');
            hold on
            scatter3(x45points,y45points,z45points,'.');
            hold on
            scatter3(x56points,y56points,z56points,'.');
            hold on

           IKinput12=[x12points(a),y12points(a),z12points(a),orientation];
           IKoutput12=fullIK(IKinput12);
           FKoutput12=fullFK(IKoutput12);

           plot3(FKoutput12(:,1),FKoutput12(:,2),FKoutput12(:,3),'ko-');
           axis([-2 8 -2 8 -2 8]);
            xlabel('x')
            ylabel('y')
            zlabel('z')
           grid on;
           hold on;
           pause(0.001);
           hold off;

        end
        for b=1:steps;

            scatter3(x12points,y12points,z12points,'.');
            hold on
            scatter3(x23points,y23points,z23points,'.');
            hold on
            scatter3(x34points,y34points,z34points,'.');
            hold on
            scatter3(x45points,y45points,z45points,'.');
            hold on
            scatter3(x56points,y56points,z56points,'.');
            hold on

               IKinput23=[x23points(b),y23points(b),z23points(b),orientation];
               IKoutput23=fullIK(IKinput23);
               FKoutput23=fullFK(IKoutput23);

               plot3(FKoutput23(:,1),FKoutput23(:,2),FKoutput23(:,3),'ko-');
               axis([-2 8 -2 8 -2 8]);
                xlabel('x')
                ylabel('y')
                zlabel('z')
               grid on;
              hold on;
               pause(0.001);
              hold off;    
        end
           for c=1:steps;

            scatter3(x12points,y12points,z12points,'.');
            hold on
            scatter3(x23points,y23points,z23points,'.');
            hold on
            scatter3(x34points,y34points,z34points,'.');
            hold on
            scatter3(x45points,y45points,z45points,'.');
            hold on
            scatter3(x56points,y56points,z56points,'.');
            hold on


               IKinput34=[x34points(c),y34points(c),z34points(c),orientation];
               IKoutput34=fullIK(IKinput34);
               FKoutput34=fullFK(IKoutput34);

               plot3(FKoutput34(:,1),FKoutput34(:,2),FKoutput34(:,3),'ko-');
               axis([-2 8 -2 8 -2 8]);
                   xlabel('x')
                   ylabel('y')
                   zlabel('z')
               grid on;
               hold on;
               pause(0.001);
               hold off;
           end
           for d=1:steps;

                  scatter3(x12points,y12points,z12points,'.');
            hold on
            scatter3(x23points,y23points,z23points,'.');
            hold on
            scatter3(x34points,y34points,z34points,'.');
            hold on
            scatter3(x45points,y45points,z45points,'.');
            hold on
            scatter3(x56points,y56points,z56points,'.');
            hold on


               IKinput45=[x45points(d),y45points(d),z45points(d),orientation];
               IKoutput45=fullIK(IKinput45);
               FKoutput45=fullFK(IKoutput45);

               plot3(FKoutput45(:,1),FKoutput45(:,2),FKoutput45(:,3),'ko-');
               axis([-2 8 -2 8 -2 8]);
                   xlabel('x')
                   ylabel('y')
                   zlabel('z')
               grid on;
               hold on;
               pause(0.001);
               hold off;
           end

           for e=1:steps;

                  scatter3(x12points,y12points,z12points,'.');
            hold on
            scatter3(x23points,y23points,z23points,'.');
            hold on
            scatter3(x34points,y34points,z34points,'.');
            hold on
            scatter3(x45points,y45points,z45points,'.');
            hold on
            scatter3(x56points,y56points,z56points,'.');
            hold on


               IKinput56=[x56points(e),y56points(e),z56points(e),orientation];
               IKoutput56=fullIK(IKinput56);
               FKoutput56=fullFK(IKoutput56);

               plot3(FKoutput56(:,1),FKoutput56(:,2),FKoutput56(:,3),'ko-');
               axis([-2 8 -2 8 -2 8]);
                   xlabel('x')
                   ylabel('y')
                   zlabel('z')
               grid on;
               hold on;
               pause(0.001);
               hold off;
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
