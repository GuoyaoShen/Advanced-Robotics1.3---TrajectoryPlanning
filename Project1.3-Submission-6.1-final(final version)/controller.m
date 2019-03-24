function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd{qn}: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd{qn}.pos, qd{qn}.vel   position and velocity
%  qd{qn}.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des  desired position, velocity, accel
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%    
% qn: quadrotor number, should always be 1
%    
% params: various parameters
%  params.I     moment of inertia
%  params.grav  gravitational constant g (9.8...m/s^2)
%  params.mass  mass of robot
%
%% Outputs:
%
% F: total thrust commanded (sum of forces from all rotors)
% M: total torque commanded
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

% =================== Your code goes here ===================
% ======calculate u1======
%define Kp and Kd
% K_p = [30.5,0,0;
%        0,35.5,0;
%        0,0,57.5];
% K_d = [7.5,0,0;
%        0,9.5,0;
%        0,0,10.5];
   
K_p = [8,0,0;
       0,8,0;
       0,0,8];
K_d = [8,0,0;
       0,8,0;
       0,0,8];
%calculate desired acceleration - Eq(26), acc_des: 3x1 matrix
acc_des = qd{qn}.acc_des - K_d * (qd{qn}.vel - qd{qn}.vel_des) - K_p * (qd{qn}.pos - qd{qn}.pos_des);
%calculate total command force - Eq(28)
F_des = params.mass * acc_des + [0;0;params.mass*params.grav];
%  qd{qn}.euler = [roll;pitch;yaw]
% function m = eulzxy2rotmat(ang)  %calculate rotation matrix (a)R(B) - Eq(4)
%     phi   = ang(1);
%     theta = ang(2);
%     psi   = ang(3);
%calculate rotation matrix using euler angles - Eq(4)
R_ab = eulzxy2rotmat(qd{qn}.euler);
%calculate b3
b3=R_ab(:,3);

%======calculate u2======
%calculate b3_des - Eq(30)
b3_des = F_des / normest(F_des);
%calculate a_psi - Eq(31)
a_psi = [cos(qd{qn}.yaw_des);sin(qd{qn}.yaw_des);0];
%calculate b2_des - Eq(32)
b2_des = cross(b3_des,a_psi) / (normest(cross(b3_des,a_psi)));
%calculate b1_des
b1_des = cross(b2_des,b3_des);
%generate R_des
R_des = [b1_des,b2_des,b3_des];
%calculate e_R - Eq(34)
e_R_temp = R_des'* R_ab - R_ab' * R_des;
e_R = 0.5 * (veemap(e_R_temp))';
%define K_R and K_omega
% K_R = [380,0,0;
%        0,390,0;
%        0,0,300];
% K_omega = [20,0,0;
%            0,22,0;
%            0,0,27];
       
K_R = [380,0,0;
       0,390,0;
       0,0,390];
K_omega = [20,0,0;
           0,22,0;
           0,0,25];
%calculate u2 - Eq(35)
u2 = params.I * (- K_R * e_R - K_omega * qd{qn}.omega);

% ==============================

% Desired roll, pitch and yaw (in rad). In the simulator, those will be *ignored*.
% When you are flying in the lab, they *will* be used (because the platform
% has a built-in attitude controller). Best to fill them in already
% during simulation.

% phi_des   = 0;
% theta_des = 0;
% psi_des   = 0;

angle_des = rotmat2eulzxy(R_des);
phi_des   = angle_des(1);
theta_des = angle_des(2);
psi_des   = angle_des(3);

%calculate phi_des, theta_des, psi_des - Eq(22)(a),(b)
% theta_des = (acc_des(1) * cos(qd{qn}.yaw_des) + acc_des(2) * sin(qd{qn}.yaw_des)) / params.grav;
% phi_des = (acc_des(1) * sin(qd{qn}.yaw_des) - acc_des(2) * cos(qd{qn}.yaw_des)) / params.grav;
% psi_des = qd{qn}.yaw_des;

%
%
%
u    = zeros(4,1); % control input u, you should fill this in
%calculate u1 - Eq(29)
u(1) = b3' * F_des;
%calculate u2
u(2:4) = u2;
% Thrust
F    = u(1);       % This should be F = u(1) from the project handout

% Moment
M    = u(2:4);     % note: params.I has the moment of inertia

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end

%
% ------------------------------------------------------------
%    should you decide to write a geometric controller,
%    the following functions should come in handy
%

function m = eulzxy2rotmat(ang)  %calculate rotation matrix (a)R(B) - Eq(4)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)  %rotation matrix to Euler ZXY
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end