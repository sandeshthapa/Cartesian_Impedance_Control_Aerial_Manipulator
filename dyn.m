function [zDot] = dyn(t,z)
x = z(1);
thetaHat1 = z(2);
thetaHat2 = z(3); 
thetaHat3 = z(4);
thetaHat = [thetaHat1 thetaHat2 thetaHat3]';
a = 1;
k = 20;
b = 1; 
d = 1*tanh(t);
c = 1*sin(t);
c_d = (c)/(d+x^2);
Y = [b a  c_d]';
u = -Y'*thetaHat - k*x;
xDot = a*x^2 + b*x*tanh(x) + c*x^3/(d+x^2)+  u;
thetaHatDot1 = x*tanh(x) * Y(1);
thetaHatDot2 = x*x*Y(2);
thetaHatDot3 = x^3*Y(3);

thetaHatDot = [thetaHatDot1; thetaHatDot2;thetaHatDot3];

zDot = [xDot;thetaHatDot];