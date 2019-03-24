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
persistent resxy_persistent;

if first_call == true
    %% Called Once First
    %======Define points must pass======
    resxy_persistent = map.res_xyz(1);
    %======Here to refine the path points====== 
    points_path = path;
%     points_path = path_refine_samedirection_new(path, 0.2);
%     points_path = path_refine_samedirection_new(points_path, 0.4);
%     points_path = path_refine_closepts(points_path, 0, 2*1.42*resxy_persistent);
%     points_path = path_refine_samedirection_new(points_path, 0.4);
%     points_path = path_refine_samedirection_new(points_path, 0.4);
    
    points_path = path_refine_deletepts(points_path, map);
    points_path = path_refine_samedirection_new(points_path, 0.2);
    
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
    vel_average = 3;  %here to set average vel
    t_total = sum_dis / vel_average;  %total time to finish the path
    t_delta = t_total * points_dis / sum_dis;
    
    %======Here to filter time delta======
    t_delta_new = time_filter(t_delta,1.3); %1.2
    t_fregment = [0; cumsum(t_delta_new)];

    desired_state.yaw = 0;
    desired_state.yawdot = 0;

else
    %% Called for Other Times
    [n_floor,~] = find(t_fregment <= t);
    n_floor_num = n_floor(end);  
    n_ceil_num = n_floor_num + 1;

    if n_ceil_num <= points_total
        [pos, vel, acc] = traject_interpolation(t_fregment(n_floor_num), t_fregment(n_ceil_num), t,...
            path_persistent(n_floor_num,:), path_persistent(n_ceil_num,:));
        desired_state.pos = pos(:);  %left: 3x1, right: pos 3x1
        desired_state.vel = vel(:);
        desired_state.acc = acc(:);
        
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    else
        desired_state.pos = path_persistent(points_total, :)';
        desired_state.vel = [0; 0; 0];
        desired_state.acc = [0; 0; 0];
        
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    end

end

%======NEW-Function to refine the path points with same direction======
% This function is used to refine and delete points with same direction
% within error
function path_new = path_refine_samedirection_new(path, error)  %path nx3 matrix; path_new mx3 matrix
    orie_error = error;
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

%======Function to refine the path points which are too close======
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

%======Function to filter delta time======
% This function is used to guarantee enough time between 2 points, avoid
% the peak in controller
function t_delta_new = time_filter(t_delta, mintime)  %mintime to give min time value between 2 points
    t_delta_new = t_delta;
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

% ====== Function to check line block collision======
function collision = check_collision(start, goal, map)
    %Initialize
    collision = [];
    margin_save = 0.2;
    obstacle = map.blocks(:, 1:6);
    newblocks = zeros(size(map.blocks, 1), 6);

    %Add more margin to obstacles to ensure no collision
    newblocks(:, 1:3) = obstacle(:, 1:3) - margin_save;
    newblocks(:, 4:6) = obstacle(:, 4:6) + margin_save;

    %check each obstacle
    for i=1:size(obstacle)
        collision_flag = detectcollision(start, goal, newblocks(i,:));
        collision = [collision,collision_flag];
    end

    if any(collision == 1)  %collision
        collision = 1;
    else  %no sollision
        collision = 0;
    end
end

%====== Function to delete points in path ======
% This function is used to delete points in path without collision. New
% path shape would be different from path planner result
function path_new = path_refine_deletepts(path, map)
    path_new = path(1, :);
    size_path = size(path, 1);
    
    while ( ~isequal(path_new(end, :), path(end, :)))      
        %Line and check
        for j = 1:size_path
            % line the two points if no collision and in certain distance
            if check_collision(path_new(end,:), path(size_path-j+1, :),...
                map) == 0 && sqrt(sum((path_new(end,:)- path(size_path-j+1, :)).^2,2))<5
                path_new = [path_new; path(size_path-j+1, :)];
                break;
            end
        end
        %Check until the new waypoint is goal
        if path(size_path-j+1, :) == path(end, :)
            break;
        end
    end
end

end

