function DX = ODIN_dynamics(X, u)
%% Model parameters
rho = 1025;
g = 9.81;
R = 0.2;
V = 4/3*pi*R^3;
B = rho*g*V;
W = B;

m = W/g;

Ib = diag([5, 5, 5]);
rgb = [0; 0; 0];

eta = X(1:6);
nu_r = X(7:12);
%% Model dynamics
I3 = eye(3);
nu2 = X(10:12);
skewMatrix = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

%% Described in center of gravity (COM)
MRB_G = [m*I3 -m*skewMatrix(rgb); m*skewMatrix(rgb) Ib];
CRB_G = [m*skewMatrix(nu2), -m*skewMatrix(nu2)*skewMatrix(rgb);
    skewMatrix(nu2)*skewMatrix(rgb), -skewMatrix(Ib*nu2)];

S = skewMatrix(rgb);
H = [eye(3)     S'
     zeros(3,3) eye(3) ];

%% Described in origin
MRB = H'*MRB_G*H;
CRB = H'*CRB_G*H;

O3 = zeros(3);
phi = eta(4); theta = eta(5); psi = eta(6);

% Rotation matrix about x-axis (Roll)
R_x = [1, 0, 0;
    0, cos(phi), -sin(phi);
    0, sin(phi), cos(phi)];
% Rotation matrix about y-axis (Pitch)
R_y = [cos(theta), 0, sin(theta);
    0, 1, 0;
    -sin(theta), 0, cos(theta)];
% Rotation matrix about z-axis (Yaw)
R_z = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi), 0;
    0, 0, 1];
% Combined rotation matrix (Z-Y-X convention)
R = R_z * R_y * R_x;

T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi), -sin(phi);
    0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
J = [R O3; 
    O3 T];
Vcb = R^(-1)*[-0.1; 0; 0];
nu = nu_r + [Vcb;0;0;0];
Deta = J*nu;

%% M*dnu + C*nu + G = u
Dnu_r = MRB^-1*(u - CRB*nu_r);

DX = [Deta;Dnu_r];
end