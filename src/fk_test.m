clear all;
close all;

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
elfin5.plot([0 -90 -90 0 0 0]*d2r);         
% elfin5.teach('callback',@(r,q) r.vellipse(q));

% qz = [0 -pi/2 -pi/2 0 0 0];
% qz = [0 0];
% elfin5.plot(qz)
% elfin5.fkine(qz)ex