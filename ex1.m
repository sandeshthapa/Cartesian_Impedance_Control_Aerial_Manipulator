close all
[t,y] = ode45(@dyn,[0 5],[1;.5;.3;.4]);
plot(t,y(:,1));
ylabel('error')
xlabel('time (s)')
title('x');
figure
plot(t,y(:,2),t,y(:,3),t,y(:,4))
ylabel('error')
xlabel('time (s)')
title('thetaHat');