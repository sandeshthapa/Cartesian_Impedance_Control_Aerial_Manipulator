function J_a_dot = Jdot_Matrix(m_b,m_1,m_2,Ixx,Iyy,Izz,Ix1,Iy1,Iz1,Ix2,Iy2,Iz2,l1,l2,x,y,z,phi,theta,psi,n1,n2,x_dot,y_dot,z_dot,phi_dot,theta_dot,psi_dot,n1_dot,n2_dot)
%JDOT_MATRIX
%    J_A_DOT = JDOT_MATRIX(M_B,M_1,M_2,IXX,IYY,IZZ,IX1,IY1,IZ1,IX2,IY2,IZ2,L1,L2,X,Y,Z,PHI,THETA,PSI,N1,N2,X_DOT,Y_DOT,Z_DOT,PHI_DOT,THETA_DOT,PSI_DOT,N1_DOT,N2_DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    21-Mar-2017 11:50:11

J_a_dot = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[8,8]);