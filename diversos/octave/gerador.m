%% Euler angles and transformation matrices
clear all
clc;
pkg load symbolic;
 
% gyro data expressed in sensor xyz frame
syms Gyro Px Qy Rz;
Gyro = [Px, Qy, Rz];

% accel data expressed in sensor xyz frame
syms Accel fx fy fz;
Accel = [fx; fy; fz];

% Attitude in Euler angles
syms PHI psi theta phi;
PHI = [phi ; theta; psi ];

% from/to sensor xyz to body frd
syms Cxyzfrd Cfrdxyz;
Cxyzfrd = [[  1   0   0  ];
          [   0   -1  0  ];
          [   0   0   -1 ]];

Cfrdxyz = [[  1   0   0  ];
          [   0   -1  0  ];
          [   0   0   -1  ]];

% from/to tp xyh to ned
syms Ctpned Cnedtp;
Cnedtp = [[  1   0   0   ];
          [  0   1   0   ];
          [  0   0   -1  ]];

Ctpned = [[  1   0   0   ];
          [  0   1   0   ];
          [  0   0   -1  ]];

          
% Rotations matrix to frd <- from ned C(frd/ned)
syms Ryaw Rpitch Rrow phi theta psi
Rroll   = [[  1             0             0             ];
          [   0             cos(phi)      sin(phi)      ];
          [   0             -sin(phi)     cos(phi)      ]];

Rpitch  = [[ cos(theta)     0             -sin(theta)   ];
          [  0              1             0             ];
          [  sin(theta)     0             cos(theta)    ]];

Ryaw   = [[ cos(psi)       sin(psi)       0             ];
          [ -sin(psi)       cos(psi)      0             ];
          [ 0               0             1             ]];

syms  Cfrdned Cnedfrd;
Cfrdned = Rroll * Rpitch * Ryaw;
Cnedfrd = transpose(Ryaw) * transpose(Rpitch) * transpose(Rroll);

%% Angular rates
% angular velocity in body frame
syms omega P Q R;
P =  Px;
Q = -Qy;
R = -Rz;
omega = [P; Q; R];

syms H;
H = [[ 1         sin(phi)*tan(theta)     cos(phi)*tan(theta)   ];
     [ 0         cos(phi)                -sin(phi)             ];
     [ 0         sin(phi)                cos(phi)/cos(theta)   ]];

% Attitude change frd/ned in Euler angle rates
syms PHI_dot phi_dot theta_dot psi_dot;
PHI_dot = H * omega;
phi_dot   = PHI_dot(1);
theta_dot = PHI_dot(2);
psi_dot   = PHI_dot(3);

% Express accel reading in different frames
syms ffrd ff fr fd ;% accel data in body frd system
ffrd = Cfrdxyz * Accel;
ff = ffrd(1);
fr = ffrd(2);
fd = ffrd(3);

syms f Fn Fe Fd; % accel data in earth ned system
f = Cnedfrd * Cfrdxyz * Accel;
fn = f(1);
fe = f(2);
fd = f(3);

% gravity vector data
G = 9.8065;
syms g gn ge gd gfrd;
g    = [ gn ; ge; gd ];
gfrd = Cfrdned * g;

syms a afrd;
a = f - g;
afrd = ffrd - gfrd;

disp(a);

