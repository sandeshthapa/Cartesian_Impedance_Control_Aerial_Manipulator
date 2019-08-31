
function [xtDot] = DynAdpC(~,xt,t,~,~)

x = xt(1); 
y = xt(2);
z = xt(3);
phi = xt(4);
theta = xt(5);
psi = xt(6); 
n1 = xt(7);
n2 = xt(8);

x_dot = xt(9);
y_dot = xt(10);
z_dot = xt(11);
phi_dot = xt(12);
theta_dot = xt(13); 
psi_dot = xt(14);
n1_dot = xt(15);
n2_dot = xt(16);

m_b = 2; 
Ixx = 1.24; 
Iyy = 1.24;
Izz = 2.48; 

m_1 = 0.049 ; 
Ix1 = 0.0011; 
Iy1 = Ix1; 
Iz1 = 0;
l1 = 0.015; 
l2 = 0.005;

Ix2 = 1.25/10000; 
Iy2 = Ix2; 
Iz2 = 0; 

m_2 = 0.05 ; 

ksi = [x y z phi theta psi n1 n2]';
ksi_dot = [x_dot y_dot z_dot phi_dot theta_dot psi_dot n1_dot n2_dot]';
p_b = [x y z]'; %postion of UAV with respect to inertial frame
p_b_dot = [x_dot y_dot z_dot]';
An_b = [psi theta phi]'; %Euler angles used: yaw pitch roll 
An_dot = [psi_dot theta_dot phi_dot]';
n_b = [n1 n2]';  % joint angles 
n_dot = [n1_dot n2_dot]'; % joint angles rate 

T_b = [ 0, -sin(psi), cos(psi)*cos(theta); 
        0,  cos(psi), cos(theta)*sin(psi); 
        1,         0,         -sin(theta)];
    
R_b = [ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta); 
      cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta),   cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);
      -sin(theta),         cos(theta)*sin(phi),                              cos(phi)*cos(theta)];
 
 
hat_w_b = [ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
           cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);
                   -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta)];
               
w_b = [phi_dot*cos(psi)*cos(theta) - theta_dot*sin(psi); 
      theta_dot*cos(psi) + phi_dot*cos(theta)*sin(psi);
                     psi_dot - phi_dot*sin(theta)];
Q = R_b'*T_b;

% Manipulator Forward Kinematics 
% PNT is the final position of end effector 
% position of center of link 1 

h1 = [1 0 0]'; 
P10 = [0 0 0.1]';
p1c = [0 0 l1/2]';

R01 = expm((hat(h1))*n1);

p1c_b = P10 + R01*p1c;    % in base frame 
p1c_i = p_b + R_b*p1c_b;  % in inertial frame

% position of center of link 2 
h2 = [0 1 0]';
p12 = [0 0 l1]';
p2T = [0 l2 0]';
p2c = [0 l2/2 0]';

R12 = expm((hat(h2))*n2);
R02 = R01*R12;

p2c_b = P10 + R01*p1c + R02*p2c;   % in base frame
p2c_i = p_b + R_b*p2c_b;  % in inertial frame

% position of end effector 

p2_b = P10 + R01*p12 + R02*p2T;   % in base frame
p2_i = p_b + R_b*p2_b;  % in inertial frame

% xt(17) = p2_b(1); 
% xt(18) = p2_b(2);

% Jacobians 
% Partial Jacobian for link 1 center 
% All joints are assumed to be prismatic
Jw1c = [h1 zeros(3,1)];
Jv1c = [hat(h1)*p1c_b zeros(3,1)];
J1c = [Jw1c; Jv1c];
% Jacobian for link 2 center 
Jw2c = [h1 h2];
Jv2c = [hat(h1)*(R01*p12) hat(h2)*p2c_b];
J2c = [Jw2c; Jv2c];

% J2c = [h1 h2; hat(h1)*(R01*p12) hat(h2)*p2c_b];
% Jacobian for end effector 
JT = [h1 h2; hat(h1)*R01*p12 hat(h2)*p2_b];
JwT = JT(1:3,1:2);
JvT = JT(4:6,1:2);

% Linear velocity of link COM 1-2 and end effector 
p1c_i_dot = p_b_dot + hat(w_b)*R_b*p1c_b + R_b*Jv1c*n_dot; % link 1 COM
p2c_i_dot = p_b_dot + hat(w_b)*R_b*p2c_b + R_b*Jv2c*n_dot; % link 2 COM
p2_i_dot = p_b_dot + hat(w_b)*R_b*p2_b + R_b*JvT*n_dot;    % end effector

% angular velocity 
w1c_i = w_b + R_b*Jw1c*n_dot;
w2c_i = w_b + R_b*Jw2c*n_dot;
w2_i = w_b + R_b*JwT*n_dot;

% Dynamic Model of Aerial Manipulator 
% Kinetic Energy of UAV 
I_b = [Ixx 0 0;0 Iyy 0;0 0 Izz]';
I_1 = [Ix1 0 0;0 Iy1 0;0 0 Iz1]';
I_2 = [Ix2 0 0;0 Iy2 0;0 0 Iz2]';

K_b = 1/2*m_b*(p_b_dot)'*p_b_dot   +   1/2*An_dot'*T_b'*R_b*I_b*R_b'*T_b*An_dot;

% KE of link
% I_1 = [Ix1 0 0;0 Iy1 0;0 0 Iz1]'; %link 1 moment of inertia 

K_1 = 1/2*m_1*(p1c_i_dot)'*p1c_i_dot  +   1/2*w1c_i'*R_b*R01*I_1*R01'*R_b'*w1c_i;
K_2 = 1/2*m_2*(p2c_i_dot)'*p2c_i_dot  +   1/2*w2c_i'*R_b*R02*I_2*R02'*R_b'*w2c_i;

% Inertia matrix B % Need to calculate the matrix 

M11 = (m_b + m_1 + m_2)*eye(3);

M22 = Q'*I_b*Q  +  m_1*T_b'*hat((R_b*p1c_b)')*hat(R_b*p1c_b)*T_b + Q'*R01*I_1*R01'*Q ...
        + m_2*T_b'*hat((R_b*p2c_b)')*hat(R_b*p2c_b)*T_b + Q'*R02*I_2*R02'*Q ;
    
M33 = m_1*(Jv1c)'*Jv1c + Jw1c'*R01*I_1*R01'*Jw1c ...
      + m_2*(Jv2c)'*Jv2c + Jw2c'*R02*I_2*R02'*Jw2c;
M12 = -(m_1*hat((R_b*p1c_b)')) -(m_2*hat((R_b*p2c_b)'));
M21 = M12';
M13 = m_1*R_b*Jv1c + m_2*R_b*Jv2c;
M31 = M13';
M23 = Q'*R01*I_1*R01'*Jw1c -m_1*T_b'*hat((R_b*p1c_b)')*R_b*Jv1c ...
      +Q'*R02*I_2*R02'*Jw2c -m_2*T_b'*hat((R_b*p2c_b)')*R_b*Jv2c ;
M32 = M23';

M1 = [M11,M12,M13];
M2 = [M21,M22,M23];
M3 = [M31,M32,M33];

M = [M1;M2;M3];
%  
C1 = C_Matrix_vars(2,0.049,0.05,1.24,1.24,2.48,0.0011,0.0011,0,0.000125,0.000125,0,0.015,0.005,xt(1),xt(2),xt(3),xt(4),xt(5),xt(6),xt(7),xt(8),xt(9),xt(10),xt(11),xt(12),xt(13),xt(14),xt(15),xt(16));
G1 = G_Matrix(2,0.049,0.05,1.24,1.24,2.48,0.0011,0.0011,0,0.000125,0.000125,0,0.015,0.005,xt(1),xt(2),xt(3),xt(4),xt(5),xt(6),xt(7),xt(8),xt(9),xt(10),xt(11),xt(12),xt(13),xt(14),xt(15),xt(16));
J1_dot = Jdot_Matrix(2,0.049,0.05,1.24,1.24,2.48,0.0011,0.0011,0,0.000125,0.000125,0,0.015,0.005,xt(1),xt(2),xt(3),xt(4),xt(5),xt(6),xt(7),xt(8),xt(9),xt(10),xt(11),xt(12),xt(13),xt(14),xt(15),xt(16));

% Control 

temp1 = -hat(R_b*p2_b)*T_b;
temp2 = R_b*JvT;

J_a = [eye(3,3)  , zeros(3,3),    zeros(3,2); 
       zeros(3,3), T_b       ,    zeros(3,2);
       eye(2,3)  , temp1(1:2,1:3),temp2(1:2,1:2)]; 
   

% disp('position of end effector');
% disp(vpa(p2_i)); 

% invJ_a = inv(J_aa);
B_x = inv(J_a')*M*inv(J_a);
% disp('B_x');
% disp(vpa(B_x));

C_x = inv(J_a')*(C1-M*inv(J_a')*J1_dot)*inv(J_a);
% disp('C_x');
% disp(vpa(C_x));

qd = [0 0 4 0 0 0 .5 .5]';
qdDot = [0 0 0 0 0 0 0 0]';
qdDotDot = [0 0 0 0 0 0 0 0]';

% x_a = [xt(1) xt(2) xt(3) xt(4) xt(5) xt(6) xt(7) xt(8)]'; 
% x_a_dot = [xt(9) xt(10) xt(11) xt(12) xt(13) xt(14) xt(15) xt(16)]';
% x_a_ddt = [0 0 0 0 0 0 0 0]';

q = [xt(1) xt(2) xt(3) xt(4) xt(5) xt(6) xt(7) xt(8)]'; 
qDot = [xt(9) xt(10) xt(11) xt(12) xt(13) xt(14) xt(15) xt(16)]';
qDotDot = xt(17:24);

if t <10
%     if sin(1/2*pi*pi*t) < 0
%         sin(1/2*pi*pi*t) =0;
%     else 
%         sin(1/2*pi*pi*t) = sin(1/2*pi*pi*t);
%     end 

    xe = (sin(pi/4*t));
    if xe < 0
        xe = 0;
    end
    
%     %Find Negative values in xetr
%     negid = xetr<0;
%     xetr(negid) = 0;
    
    fext = [xe 0 0 0 0 0 0 0]';  
else 
    fext = [0 0 0 0 0 0 0 0]';
end 

% fext = [0 0 0 0 0 0 0 0]';
lamda = eye(8,8);
e = q - qd; 
eDot = qDot - qdDot;
s = eDot + lamda*e;
qrDot = qdDot-lamda*e;
qrDotDot = qdDotDot - lamda*eDot;

% x_telta = qd - q;
% x_telta_dot = qdDot - qDot;
% disp('x_telta');
% disp(x_telta);

dxdt(1:8,1) = xt(9:16);

Kv = 0.5*(eye(8,8)); 
Kp = 5*eye(8,8);

% K_D = [0.8*eye(6,6),zeros(6,2);
%        zeros(2,6) .85*eye(2,2)];
% K_P = [80*eye(6,6),zeros(6,2);
%        zeros(2,6) 100*eye(2,2)];
tau_ext = [0 0 0 0 0 0 0 0];
MHat = eye(8,8);
CHat = zeros(8,8);
GHat = [0 0 10 0 0 0 0 0]';
MTelta = MHat - M; 
CTelta = CHat - C1; 
GTelta = GHat - G1;
Delta = -MTelta*qDotDot - CTelta*qDot - GTelta - tau_ext;
DeltaHat = xt(25:32);
DeltaTelta = DeltaHat - Delta; 
tau = MHat*qrDotDot + CHat*qrDot + GHat - Kv*eDot + Kp*e+ DeltaHat;  
dxdt(9:16,1) = inv(M)*(tau + tau_ext + G1);

sDot =  inv(M)*(DeltaTelta - CHat*s -Kv*eDot - Kp*e);
% xt(17:25,1) = qDotDot;

% -((B_x)\(fext - ((C_x + K_D)*x_telta_dot + K_P*x_telta)));

% disp('xt');
% disp(xt);
% 
% disp('dxdt');
% disp(dxdt);

Kgm = eye(8,8);
DeltaHatDot = -Kgm*s;

xtDot = [dxdt(9:16,1);sDot;DeltaHatDot];

% plot(t,x_telta(1),t,x_telta(2),t,x_telta(3));
% hold on;


% for ii = 1:21
% %     n = 16;
%     plot3(xt(ii,1),(xt(ii,2)),(xt(ii,3)),5); 
% hold on 
% end 
% hold off
% n = 16;
% plot3(x,y,z,'r');
% hold on; 

% plot3([xt()],[xt(2)],[xt(3)])
% hold off

% dxdt

















