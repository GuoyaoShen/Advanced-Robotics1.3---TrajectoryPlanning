function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. At first, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% parameters:
%   map: The map structure returned by the load_map function
%   path: This is the path returned by your planner (dijkstra function)
%   desired_state: Contains all the information that is passed to the
%                  controller, as in Phase 1
%
% NOTE: It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

%==========================================================================

if nargin > 2
   first_call = true; 
else
   first_call = false;
end

persistent t_total;
persistent t_delta_new;
persistent t_fregment;
persistent points_total;
persistent path_persistent;
persistent yaw_persistent;
persistent resxy_persistent;
persistent resz_persistent;
persistent snap_coeffs

if first_call == true
    %% Called Once First
    %======Define points must pass======
    resxy_persistent = map.res_xyz(1);
    resz_persistent = map.res_xyz(3);
    %======Here to refine the path points====== 
%     points_path = path_refine_samedirection_new(path, 0.1);
%     points_path = path_refine_closepts(points_path, 0, 3*1.42*resxy_persistent);
%     points_path = path_refine_samedirection_new(points_path, 0.2);  %only about 10 points
    points_path = path;
    
%     points_path = pathSmooth(points_path, map);
    
%     density = 5;
%     points_path = addWaypoints(density, points_path);
%     points_path = pathSmooth(points_path, map);
%     density = 10;
%     points_path = addWaypoints(density, points_path);
    
%     points_path = path;
    
    %reset ending point
    if any(points_path(end, :) ~= path(end, :))
        points_path = [points_path; path(end, :)];
    end
    
    points_path_num = size(points_path, 1);
    points_total = points_path_num;  %save total points number
    path_persistent = points_path;  %save path  
    
    %======calculate distance between points======
    points_dis = sqrt(sum(diff(points_path).^2, 2));  %(n-1)x1 matrix
    sum_dis = sum(points_dis);  %sum distance

    %======Initialize para======
    t_total = 16.0;  %total time to finish the path
    t_delta = t_total * points_dis / sum_dis;
    
    %======Here to filter time delta======
%     t_delta_new = time_filter(t_delta,0.2);
    t_delta_new = t_delta;
    t_fregment = [0; cumsum(t_delta_new)];

    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    yaw_persistent = 0;
    
    snap_coeffs = Min_SnapSpline(t_fregment, points_path, zeros(4,3));

else
    %% Called for Other Times
%     [n_floor,~] = find(t_fregment <= t);
%     n_floor_num = n_floor(end);  
%     
%     n_ceil_num = n_floor_num + 1;
% 
%     if n_ceil_num <= points_total
%         [pos, vel, acc] = traject_interpolation(t_fregment(n_floor_num), t_fregment(n_ceil_num), t, path_persistent(n_floor_num,:), path_persistent(n_ceil_num,:));
%         desired_state.pos = pos(:);  %left: 3x1, right: pos 3x1
%         desired_state.vel = vel(:);
%         desired_state.acc = acc(:);
%         
%         vel_xy = desired_state.vel(1:2);
%         x_xy = [1; 0];
%         if all(vel_xy == 0) 
%             yaw = yaw_persistent;
%         else
%             yaw = atan2(vel_xy(2), vel_xy(1)) ;
%         end
%         
%         desired_state.yaw = yaw;
%         desired_state.yawdot = 0;
%         yaw_persistent = yaw;
%     else
%         desired_state.pos = path_persistent(points_total, :)';
%         desired_state.vel = [0; 0; 0];
%         desired_state.acc = [0; 0; 0];
%         desired_state.yaw = yaw_persistent;
%         desired_state.yawdot = 0;
%     end
    
    %=============================================================
    if (t > t_fregment(end))
        desired_state.pos = path_persistent(end, :)';
        desired_state.vel = [0,0,0]';
        desired_state.acc = [0,0,0]';
        desired_state.yaw = 0;
        desired_state.yawdot = 0; 
    else
        [pos, vel, acc] = computeTrajectory(t_fregment, t, snap_coeffs);

        desired_state.pos = pos';
        desired_state.vel = vel';
        desired_state.acc = acc';
        desired_state.yaw = 0;
        desired_state.yawdot = 0;    
    end

end

%% ======NEW-Function to refine the path points with same direction======
% This function is used to refine and delete points with same direction
% within error
function path_new = path_refine_samedirection_new(path, error)  %path nx3 matrix; path_new mx3 matrix
    orie_error = error;
    path_num = size(path, 1);
    path_diff = diff(path);  %calculate the difference between 2 points, (n-1)x3 matrix
    %calculate distance between 2 points
    path_dis = sqrt(path_diff(:, 1).^2 + path_diff(:, 2).^2 + path_diff(:, 3).^2)';
    path_dis = path_dis';  %(n-1)x3 matrix
    path_orie = path_diff ./ path_dis;
    
    path_new = path;
    delete = [];
    for i = 1: size(path_orie)-1
        if path_orie(i,:) > path_orie(i+1,:) - orie_error & path_orie(i,:) < path_orie(i+1,:) + orie_error
            delete = [delete; i+1];
        end
    end
    path_new(delete,:) = [];
end

%% ======Function to refine the path points which are too close======
% This function is used to delete those points which are too close so that
% to refine path
function path_new = path_refine_closepts(path, dis_min, dis_max)  %path nx3 matrix, dis_min to indicate min distance to filter
    path_new = path;
    path_diff = diff(path); %path_diff (n-1)x3 matrix
    %calculate distance between 2 points
    path_dis = sqrt(path_diff(:, 1).^2 + path_diff(:, 2).^2 + path_diff(:, 3).^2)';
    path_dis = path_dis';  %(n-1)x3 matrix
    path_delete_row = find(path_dis < dis_max & path_dis > dis_min);
    points_delete_row = path_delete_row + 1;
    path_new(points_delete_row, :) = [];
end

%% ======Function to filter delta time======
function t_delta_new = time_filter(t_delta, mintime)  %mintime to give min time value between 2 points
    t_delta_new = t_delta;
%     t_delta_sort = sort(t_delta);
%     t_min = t_delta_sort(2);  %find min t in all t_delta
%     if t_min <mintime
%         scalar = mintime / t_min;
%         t_delta_new = scalar * t_delta;
%     end
    for i = 1: size(t_delta_new)
        if t_delta_new(i) < mintime && t_delta_new(i) > 0
            t_delta_new(i) = mintime;
        end
    end
end

%======Interpolation function======
function [pos, vel, acc] = traject_interpolation(t_start, t_finish, t, point_start, point_final)
    pos = zeros(1,3); vel = zeros(1,3); acc = zeros(1,3);
    matrix_Quintic = [1, t_start, t_start^2, t_start^3, t_start^4, t_start^5;
                      0, 1, 2*t_start, 3*t_start^2, 4*t_start^3, 5*t_start^4;
                      0, 0, 2, 6*t_start, 12*t_start^2, 20*t_start^3;
                      1, t_finish, t_finish^2, t_finish^3, t_finish^4, t_finish^5;
                      0, 1, 2*t_finish, 3*t_finish^2, 4*t_finish^3, 5*t_finish^4;
                      0, 0, 2, 6*t_finish, 12*t_finish^2, 20*t_finish^3];
    matrix_Time = [1, t, t^2, t^3, t^4, t^5;
                   0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4;
                   0, 0, 2, 6*t, 12*t^2, 20*t^3;];
    matrix_State = [point_start; zeros(2,3); point_final; zeros(2,3)];  %6x3 matrix
    matrix_Coefficient = matrix_Quintic \ matrix_State;
    matrix_Result = matrix_Time * matrix_Coefficient;
    pos = matrix_Result(1,:)'; vel = matrix_Result(2,:)'; acc = matrix_Result(3,:)';  %pos, vel, acc, 3x1
end

%% ======Function to generate min acc Cubic Matrix (n = 2)======
function cubic_matrix = cubic_interpolation(path, t_fregment, state_init)
    path_size = size(path, 1);
        
end

%% ****************** Function to generate Spline matrix ******************
function snap_coeffs = Min_SnapSpline(time_matrix, path, state_SF)
% MINIMUM SNAP TRAJECTORY
% DO: Generate a Spline Matrix that can be used to minimize the snap of
%     drone. The trajectory takes the whole path as input and add
%     constrains on intermediate points. As a result, the drone can move
%     without stop and go.
%     Input:
%       t_matrix: A [n, 3] matrix restores the time of each waypoint
%       path: [n, 3] matrix restores the position of each waypoint
%       state_SF: a [6, 3] matrix restores the start and final states. The
%                 first three rows are the vel, acc, snap, for start.
%       time: the time now, used to compute the trajectory in realtime
%     Output:
%       pos: the position at each time [1, 3]
%       vel: the velocity at each time [1, 3]
%       acc: the acceleration at each time [1, 3]
%
% ******************************* Initial *********************************
size_pt = size(path, 1);
snap_matrix = zeros(10 * (size_pt - 1),10 * (size_pt - 1)); 
% for j = 1:3
    snap_matrix(:,:) = eye(10 * (size_pt - 1))*20*eps;
% end

state_matrix = zeros(10 * (size_pt - 1), 3);

% ****************** Write sanp_matrix and state matrix *******************
for i = 1:size_pt
    t = time_matrix(i);
        xt = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1];
        xd = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0];
        xdd = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2, 0, 0];
        xddd = [210*t^4, 120*t^3, 60*t^2, 24*t, 6, 0, 0, 0];
        xd4 = [840*t^3, 360*t^2, 120*t, 24, 0, 0, 0, 0];
        xd5 = [2520*t^2, 720*t, 120, 0, 0, 0, 0, 0];
        xd6 = [5040*t, 720, 0, 0, 0, 0, 0, 0];
%     
%     xt = [1, t, t^2, t^3, t^4, t^5];
%     xd = [0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4];
%     xdd = [0, 0, 2, 6*t, 12*t^2, 20*t^3];
%     xddd = [0, 0, 0, 6, 24*t, 40*t^2];
%     xd4 = [0, 0, 0, 0, 24, 80*t];

%             T =   [    t^3, t^2, t, 1]; % position time
%             dT =  [3 * t^2, 2*t,  1, 0]; % velocity time
%             ddT = [6 * t  , 2   ,  0, 0]; % acceleration time
    
    % ************* Firstly write initial and final condition *************
    if i == 1
        snap_matrix(1:4, 1:8) = [xt; xd; xdd; xddd];
        state_matrix(1:4,:) = [path(i,:); zeros(3)];
    elseif i == size_pt
        snap_matrix(end-3:end, end-7:end) = [xt; xd; xdd; xddd];
        state_matrix(end-3:end, :) = [path(i,:); zeros(3)];

%     if i == 1
%         snap_matrix(1:3, 1:6) = [xt; xd; xdd];
%         state_matrix(1:3,:) = [path(i,:); zeros(2,3)];
%     elseif i == size_pt
%         snap_matrix(end-2:end, end-5:end) = [xt; xd; xddd];
%         state_matrix(end-2:end, :) = [path(i,:);  zeros(2,3)];

%     if (i == 1)
%                 A = [T;...
%                     dT];
%                 snap_matrix(1:2,1:4) = A;
%                 state_matrix(1:2, :) = [path(i,:);zeros(1,3)];
% 
%             % Build in xf and vf constraints    
%     elseif (i == size_pt)
%                 A = [T;...
%                     dT];
%                 snap_matrix(end-1:end,end-3:end) = A;
%                 state_matrix(end-1:end, :) = [path(i,:);zeros(1,3)];
        
    % ******************* Then the intermediate constrains ****************
    else
        snap_matrix(8*i-11:8*i-4, 8*i-15:8*i) = [xt, zeros(1,8);...
                                                 zeros(1,8), xt;...
                                                 xd, -xd;...
                                                 xdd, -xdd;...
                                                 xddd, -xddd;...
                                                 xd4, -xd4;...
                                                 xd5, -xd5;...
                                                 xd6, -xd6];
        state_matrix(8*(i-1)-3:8*(i-1)+4, :) = [path(i, :); path(i,:); zeros(6, 3)];

%         snap_matrix(6*i-8:6*i-3, 6*i-11:6*i) = [xt, zeros(1,6);...
%                                              zeros(1,6), xt;...
%                                              xd, -xd;...
%                                              xdd, -xdd;
%                                              xddd, -xddd;
%                                              xd4, -xd4];
%         state_matrix(6*i-8:6*i-3, :) = [path(i, :); path(i,:); zeros(4, 3)];

%                A = [T,  zeros(1,4);...
%                     zeros(1,4), T;...
%                     dT,      -dT;...
%                     ddT,     -ddT];
%                 snap_matrix((4*i-5):(4*i-2),(4*i-7):(4*i))= A;
%                 state_matrix((4*i-5):(4*i-2),:) = [path(i,:);path(i,:);zeros(2,3)];

    end
end
% ********************* Compute coefficient matrix ************************
snap_coeffs = snap_matrix \ state_matrix;
end

%% ********************* Generate pos, vel and acc ************************ 
function [pos, vel, acc] = computeTrajectory(time_matrix, t, coeff_matrix)
% COMPUTE TRAJECTORY
% DO: Compute the desire states of drone according to the time now, and the
%     coefficient matrix above
%     Input:
%       time_matrix: [n, 3] matrix restores the time of each waypoint
%       t: the time now
%       coeff_matrix: [8*(n-1), 3] matrix, the coefficient matrix computed 
%                      above
%
% ********************* determine the segment *****************************
    [row_need, ~] = find(time_matrix > t);
    coeff = coeff_matrix(8*(row_need(1) - 1)-7:8*(row_need(1) - 1), :);
    
% ************************ Compute real state *****************************
T_matrix = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1;
            7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0;
            42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2, 0, 0];
    
%     T_matrix= [1, t, t^2, t^3, t^4, t^5;
%                0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4;
%                0, 0, 2, 6*t, 12*t^2, 20*t^3];

% T_matrix=[   t^3, t^2,  t,  1 ;
%        3*t^2, 2*t,  1,  0 ;
%           6*t,   2,  0,  0 ];

result_matrix = T_matrix * coeff;
pos = result_matrix(1,:); vel = result_matrix(2,:); acc = result_matrix(3,:);
end

%% *********************** Function to add waypoints **********************
function path_new = addWaypoints(density, path)
% ADD WAYPOINTS
% DO: In order to ensure the performance of minimum snap spline, More 
%     waypoints should be added into the path. The points is added according
%     to the waypoint density. 
%     Input:
%       density: the density of waypoints per meter
%       path: [n, 3] matrix, a path after smooth
%     Output:
%       path_new: [m, 3]matrix, the path after adding waypoints. m>n
%
path_new = path;
dist_diff = sqrt(sum(diff(path).^2, 2));
total_dist = sum(dist_diff);
addpt_num = floor(dist_diff * density);
addpt_sum = sum(addpt_num);
addpt_dist = dist_diff ./ addpt_num;
path_tempt = [];
for u = 1 : size(dist_diff)
   direct = (path(u+1, :) - path(u, :))/norm(path(u+1, :) - path(u, :));
   newpt = path(u, :);
   for r = 1 : addpt_num(u)
       newpt = newpt + addpt_dist(u, :) * direct;
       path_tempt = [path_tempt; newpt];
   end
   [row_bool, ~] = ismember(path_new, path(u, :), 'rows');
   row_num = find(row_bool == 1);
   path_new = [path_new(1:row_num, :);path_tempt;path_new(row_num+1:end, :)];
   path_tempt = [];
end
path_new(end,:) = [];
end

%% ************** Function to check line block collision***************
function flag = lineBlock_intersect(start, goal, map)
% LINEBLOCK INTERSECT
% Do: Check whether the line intersects with blocks. If check the function
%     will return a flag.
%     Input:
%       map: a structure with the position and size of blocks
%       start: the start point of the line segment
%       end: the end point of the line segment
%     Output:
%       flag: return 1 if check occurs, else return 0
%
collision = [];
margin = 0.2;
newblocks = zeros(size(map.blocks, 1), 6);
obstacle = map.blocks(:, 1:6);

% ************ To make sure free of collision, add margin to blocks********
newblocks(:, 1:3) = obstacle(:, 1:3) - margin;
newblocks(:, 4:6) = obstacle(:, 4:6) + margin;

% ************************* check every blocks*****************************
for i=1:size(map.blocks)
    c_flag = detectcollision(start, goal, newblocks(i,:));
    collision = [collision,c_flag];
end

% ************************* Return value **********************************
if any(collision == 1)
    flag = 1;
else
    flag = 0;
end
end

%% ****************** Function to smooth path *************************
function path_smooth = pathSmooth(path, map)
% PATH SMOOTH
% Do: Try to line up all the waypoints from start to end. If the two way
%     points can be lined without collision with blocks, write down those
%     waypoints. In this way, the size of path will decrease greatly.
%     Input:
%       map: a structure with the position and size of blocks
%       path: a [n, 3] matrix, in which all points are restored
%     Output:
%       path_smooth: a new path with much less waypoints. [m, 3] matrix,
%       where m < n
%
path_smooth = path(1, :);
size_path = size(path, 1);
i = 1;

while ( ~isequal(path_smooth(i, :), path(end, :)))      
% *********************** Line and check **********************************
    for j = 1:size_path
        if lineBlock_intersect(path_smooth(i,:), path(size_path-j+1, :),...
            map) == 0 && sqrt(sum((path_smooth(i,:)- path(size_path-j+1, :)).^2,2))<5        % line the two points if no collision
            path_smooth = [path_smooth; path(size_path-j+1, :)];
            break;
        end
    end
    
% ************** Check until the new waypoint is goal *********************
    if path(size_path-j+1, :) == path(end, :)
        break;
    end
    i=i+1;
end
end

end

