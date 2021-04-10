%% IKSovler for 6-DOF arm with configuration of the last three joints intersecting
% Given desired pose, compute all joint solutions.
clc
clear
close all

r2d = 180/pi;
d2r = pi/180;
%% Robot Model:6-DOF,standard DH
d_bs = 0.22;     % Elfin5
d_se = 0.38;
d_ew = 0.42;
d_wt = 0.18;

%              theta        d        a       alpha
links = [
            Link([0        d_bs      0       pi/2])
%             Link([-pi/2     0      -d_se       pi])
%             Link([-pi/2     0        0       pi/2])
            Link([0         0      -d_se       pi])
            Link([0         0        0       pi/2])
            Link([0        d_ew      0      -pi/2])
            Link([0        0         0       pi/2])
            Link([0        d_wt      0          0])
         ];

elfin5=SerialLink(links, 'name', 'elfin5');
% elfin5.plot([0 -90 -90 0 0 0]*d2r);
% elfin5.teach('callback',@(r,q) r.vellipse(q));

qz0 = [0 -pi/2 -pi/2 0 0 0];
elfin5.fkine(qz0);
%% IKSolver
% All the angles are transformed between [-PI, PI]
qz = [10 30 30 20 30 10]*d2r;
Xd60 = elfin5.fkine(qz+qz0).t
Rd60 = elfin5.fkine(qz+qz0).R

Lbs0 = [0 0 d_bs]';
Lse2 = [-d_se 0 0]';
Lew3 = [0 0 d_ew]';
Lwt6 = [0 0 d_wt]';

q_serial = zeros(6,8);
m = 0;
%===========theta3===========%
Xsw0 = Xd60 - Lbs0 - Rd60*Lwt6;
if Xsw0(1)==0 && Xsw0(2)==0
    error('shoulder sigularity!')
end
num = Xsw0'*Xsw0 - d_se^2 - d_ew^2;
den = 2*d_se*d_ew;
C3 = num/den;
if abs(C3)>1.0
    C3 = sign(num)*sign(den);
end
theta3(1) = acos(C3);   %[0,pi)
theta3(2) = -acos(C3);  %(-pi,0]
for i = 1:2
    a = d_ew*sin(theta3(i));
    b = d_se + d_ew*cos(theta3(i));
    rou = sqrt(a^2 + b^2);
    %===========theta2===========%
    div = Xsw0(3) / rou;
    if abs(div)>(1+1e-12)
        continue;
    end
    if abs(rou-abs(Xsw0(3)))<1e-12 && abs(div)>1.0
        div = sign(Xsw0(3));
    end
    theta2(i,1) = atan2(a, b) + acos(div);
    theta2(i,2) = atan2(a, b) - acos(div);
    
    index_j = 2;
    if theta2(i,1)==theta2(i,2)
        index_j = 1;
    end
    for j = 1:index_j
        %===========theta1===========%
        temp = a*cos(theta2(i,j)) - b*sin(theta2(i,j));
        if abs(Xsw0(2))<1e-12 && abs(Xsw0(1))<1e-12
            continue;
        end
        theta1(i,j) = atan2(Xsw0(2)*sign(temp), Xsw0(1)*sign(temp));
        %======================WRIST======================%
        R03 = rotz(theta1(i,j)*r2d) * rotx(90)...
              * rotz(theta2(i,j)*r2d-90) * rotx(180)...
              * rotz(theta3(i)*r2d-90) * rotx(90);
        R36 = R03' * Rd60;
        
        cos5 = R36(3,3);
        if abs(cos5) > (1+1e-12)
            continue;
        end
        if abs(abs(cos5)-1) < 1e-12
            cos5 = sign(cos5);
%             error('wrist sigularity!')
%             continue;
        end
        %===========theta5===========%
        theta5(i,j,1) = acos(cos5);
        theta5(i,j,2) = -theta5(i,j,1);
        index_k = 2;
        if theta5(i,j,1) == theta5(i,j,2)
            index_k = 1;
        end
        for k = 1:index_k
            if sin(theta5(i,j,k))==0
                fprintf('sin(theta5(%d,%d,%d)==0\n',i,j,k);
                continue;
            end
            %===========theta4===========%
            theta4(i,j,k) = atan2(sign(sin(theta5(i,j,k)))*R36(2,3),...
                                  sign(sin(theta5(i,j,k)))*R36(1,3));
            %===========theta6===========%
            theta6(i,j,k) = atan2(sign(sin(theta5(i,j,k)))*R36(3,2),...
                                  sign(-sin(theta5(i,j,k)))*R36(3,1));
        
            % save the results
            m = m+1;
            q_serial(:,m) = r2d * [theta1(i,j),theta2(i,j),theta3(i),...
                theta4(i,j,k),theta5(i,j,k),theta6(i,j,k)]';
        end
    end
end
%% Verification test
n = 0;
error_t = 1e-8;
error_R = 1e-7;
for i=1:2
    for j=1:index_j
        for k=1:index_k
            qzn = [theta1(i,j),theta2(i,j),theta3(i),...
                theta4(i,j,k),theta5(i,j,k),theta6(i,j,k)];
            Tc = elfin5.fkine(qzn+qz0);
            if norm(Tc.t - Xd60) > error_t
                n = n+1;
                fprintf('<-------------->\ni=%d,j=%d,k=%d\n',i,j,k)
                disp('Tc.t=')
                Tc.t
                Xd60
                fprintf('error = %d\n',norm(Tc.t-Xd60))
%                     error('target pos does not coincide!')
                continue;
            end
            if norm(Tc.R-Rd60)>error_R
                n = n+1;
                fprintf('<-------------->\ni=%d,j=%d,k=%d\n',i,j,k)
                disp('Tc.R=')
                Tc.R
                Rd60
                fprintf('error = %d\n',norm(Tc.R-Rd60))
%                     error('target pose does not coincide!')
                continue;
            end
    end
    end
end
fprintf('Äæ½âÊ§°Ü´ÎÊýn = %d\n',n)

