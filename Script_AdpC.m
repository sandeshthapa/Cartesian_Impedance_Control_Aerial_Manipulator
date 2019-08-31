close all; clear all; clc; 

sim('Adaptive_C');

figure
plot(time,torque(:,1));
legend('Torque 1');
hold on 

figure
plot(time,torque(:,2));
legend('Torque 2')
hold on

figure
plot(time,torque(:,3));
legend('Torque 3')
hold on

figure
plot(time,torque(:,4));
legend('Torque 4')
hold on

figure
plot(time,error(:,5));
legend('Torque 5')
hold on

figure
plot(time,torque(:,6));
legend('Torque 6')
hold on

figure
plot(time,error(:,7));
legend('Torque 7')
hold on

figure
plot(time,torque(:,8));
legend('Torque 8')
hold on


figure
plot(time,error(:,1));
legend('error 1')
hold on

figure
plot(time,error(:,2));
legend('error 2')
hold on

figure
plot(time,error(:,3));
legend('error 3')
hold on

figure
plot(time,error(:,4));
legend('error 4')
hold on

figure
plot(time,error(:,5));
legend('error 5')
hold on

figure
plot(time,error(:,6));
legend('error 6')
hold on

figure
plot(time,error(:,7));
legend('error 7')
hold on

figure
plot(time,error(:,8));
legend('error 1')
hold on
