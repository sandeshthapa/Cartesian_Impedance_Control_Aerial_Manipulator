
clear all; close all; clc; 

xinit = [0;0;2;0;0;0;0;pi/2;
         1;1;1;0;1;1;1;1;
         0;1;1;0;1;1;1;1;
         0;0;1;0;0;0;0;0];
% tspan = linspace(t1,t2,n);
tspan = 0:.1:100;
% options = odeset('RelTol',1e-5,'Stats','on','OutputFcn',@odeplot);
[t,xt] = ode45('DynAdpC',tspan,xinit);

% x_des = [0 0 2 0 0 0 0 0]';

% x_telta1 = abs(xt(:,1));
% x_telta2 = abs(xt(:,2));
% [a,b] = size(xt);
% x_telta3 = abs(4*ones(1,b)-xt(:,3));
% % plot(t,x_telta1,t,x_telta2,t,x_telta3)
% 
% 
% subplot(2,2,1)
% plot(t,abs(xt(:,1)),'b');
% xlabel('t');
% ylabel('m');
% title('x position')
% 
% 
% subplot(2,2,2)
% plot(t,abs(xt(:,2)),'b');
% xlabel('t');
% ylabel('m');
% title('y-position')
% 
% 
% subplot(2,2,3)
% p = plot(t,x_telta3,'b');
% p(1).LineWidth = 1;
% xlabel('t');
% ylabel('m');
% title('z-position');
% 
% subplot(2,2,4)
% plot(t,abs(xt(:,4)),'b');
% xlabel('t');
% ylabel('rad');
% title('yaw');
% 
% 
