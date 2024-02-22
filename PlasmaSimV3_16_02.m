clear all; close all; clc;

q=1.6e-19;
m=9.1e-31;
B=[0,0,1];
E=[0,10,0];

dy=@(t,y) [y(4); y(5); y(6);
(q/m)*( E(1) + ( y(5)*B(3) - y(6)*B(2) ));
(q/m)*( E(2) + ( y(6)*B(1) - y(4)*B(3) ));
(q/m)*( E(3) + ( y(4)*B(2) - y(5)*B(1) ))];


delta_t=[0 1e-8];
a_i=[0 0 0, 0 1e5 1e5];

options=odeset('RelTol',1e-2); %,'AbsTol',1e-8);
[t, x] = ode45(dy, delta_t, a_i,options);

% [azimuth,elevation,r] = cart2sph(x(:,1),x(:,2),x(:,3))

%r = sqrt(x(:,1).^2 + x(:,2).^2 + x(:,3).^2);
%v1 = atan(sqrt(x(:,1) + x(:,2))/x(:,3));
%v2 = atan(x(:,2)/x(:,1));



figure(1)
plot3(x(:,1),x(:,2),x(:,3))
hold on 
title('Harmonic oscillator')
xlabel('X')
ylabel('Y')
zlabel('Z')
hold off



V = [x(:,4),x(:,5),x(:,6)]
Blen = repmat(B,length(V),1)



sintheta = cross(V,Blen)./(abs(V).*abs(Blen))
 
r = (m*V.*sintheta)./(abs(q).*Blen)

% figure(2)
% plot(r,t)
% hold on
% title('radius v tid') 
% ylabel('radius[m]')
% xlabel('tid [s]')
% hold off
