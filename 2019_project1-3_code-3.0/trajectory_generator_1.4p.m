function [desired_state] = trajectory_generator(t, qn, varargin)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% t: A scalar, specifying inquiry time
%
% varargin: variable number of input arguments. In the framework,
% this function will first (and only once!) be called like this:
%
% trajectory_generator([],[], 0, path)
%
% i.e. map = varargin{1} and path = varargin{2}.
%
% path: A N x 3 matrix where each row is (x, y, z) coordinate of a
% point in the path. N is the total number of points in the path
%
% This is when you compute and store the trajectory.
%
% Later it will be called with only t and qn as an argument, at
% which point you generate the desired state for point t.
%

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

if first_call == true
    %% Called Once First
    %======Define points must pass======
    %======Here to refine the path points====== 
    points_path = path_refine(varargin{2});
%     points_path = varargin{2};
    points_path_num = size(points_path, 1);
    points_total = points_path_num;  %save total points number
    path_persistent = points_path;  %save path  
    
    points_diff = diff(points_path);  %calculate the difference between 2 points, (n-1)x3 matrix
    %calculate distance between 2 points
    points_dis = sqrt(points_diff(:, 1).^2 + points_diff(:, 2).^2 + points_diff(:, 3).^2)';
    points_dis = points_dis';  %(n-1)x1 matrix
    sum_dis = sum(points_dis);  %sum distance
    time_ratio = points_dis / sum_dis;  %(n-1)x1 matrix

    %======Initialize para======
    t_total = 16.0;  %total time to finish the path
    t_delta = time_ratio * t_total;  %calculate time between 2 points in path
    t_delta = [0; t_delta];  %nx1 matrix, THIS IS CORRECT
    
    %======Here to filter time delta======
%     t_delta_new = t_delta;
    t_delta_new = time_filter(t_delta,3);
    
    t_fregment = [];  %should be nx1 matrix finally
    t_flag = 0;
    for i = 1: size(t_delta_new)
        t_flag = t_flag + t_delta_new(i);
        t_fregment = [t_fregment;t_flag];
    end

    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    yaw_persistent = 0;

else
    %% Called for Other Times
    [n_floor,~] = find(t_fregment <= t);
    n_floor_num = n_floor(end);  
    
    n_ceil_num = n_floor_num + 1;

    if n_ceil_num <= points_total
        [pos, vel, acc] = traject_interpolation(t_fregment(n_floor_num), t_fregment(n_ceil_num), t, path_persistent(n_floor_num,:), path_persistent(n_ceil_num,:));
        desired_state.pos = pos(:);  %left: 3x1, right: pos 3x1
        desired_state.vel = vel(:);
        desired_state.acc = acc(:);
        
        vel_xy = desired_state.vel(1:2);
        x_xy = [1; 0];
        if all(vel_xy == 0) 
            yaw = yaw_persistent;
        else
            yaw = atan2(vel_xy(2), vel_xy(1)) ;
        end
        
        desired_state.yaw = yaw;
        desired_state.yawdot = 0;
        yaw_persistent = yaw;
    else
        desired_state.pos = path_persistent(points_total, :)';
        desired_state.vel = [0; 0; 0];
        desired_state.acc = [0; 0; 0];
        desired_state.yaw = yaw_persistent;
        desired_state.yawdot = 0;
    end

end
%======Function to refine the path points======
function path_new = path_refine(path)  %path nx3 matrix; path_new mx3 matrix
    path_num = size(path, 1);
    path_diff = diff(path);  %calculate the difference between 2 points, (n-1)x3 matrix
    %calculate distance between 2 points
    path_dis = sqrt(path_diff(:, 1).^2 + path_diff(:, 2).^2 + path_diff(:, 3).^2)';
    path_dis = path_dis';  %(n-1)x3 matrix
    path_orie = path_diff ./ path_dis;
    
    path_new = path;
    delete = [];
    for i = 1: size(path_orie)-1
        if path_orie(i,:) == path_orie(i+1,:)
            delete = [delete; i+1];
        end
    end
    path_new(delete,:) = [];
end

%======Function to filter delta time======
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
    matrix_State = [point_start; zeros(2,3); point_final; zeros(2,3)];
    matrix_Coefficient = matrix_Quintic \ matrix_State;
    matrix_Result = matrix_Time * matrix_Coefficient;
    pos = matrix_Result(1,:)'; vel = matrix_Result(2,:)'; acc = matrix_Result(3,:)';  %pos, vel, acc, 3x1
end

% use the "persistent" keyword to keep your trajectory around
% inbetween function calls



%
% When called without varargin (isempty(varargin) == true), compute
% and return the desired state here.
%

% desired_state.pos = pos(:);
% desired_state.vel = vel(:);
% desired_state.acc = acc(:);
% desired_state.yaw = yaw;
% desired_state.yawdot = yawdot;

end
