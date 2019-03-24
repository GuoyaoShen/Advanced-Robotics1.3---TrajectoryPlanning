function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an mx3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path. The first
%   row is start and the last row is goal. If no path is found, PATH is a
%   0x3 matrix. Consecutive points in PATH should not be farther apart than
%   neighboring voxels in the map (e.g. if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of nodes that were expanded while performing the search.
%   
% paramaters:
%   map     - the map object to plan in
%   start   - 1x3 vector of the starting coordinates [x,y,z]
%   goal:   - 1x3 vector of the goal coordinates [x,y,z]
%   astar   - boolean use astar or dijkstra

tic;
%% Check input
if nargin < 4
    astar = false;
end

%% Initial output
path = [];
num_expanded = 0;

%% Check if start and goal points are in boundary
if boundcheck([start; goal], map)
    warning('START OR GOAL POINTS OUT OF BOUNDARY !');
    return
end

%% Initialize
% Initial start and goal point data
start_ind = pos2ind(map, start);
goal_ind = pos2ind(map, goal);
goal_sub = pos2sub(map, goal);
start_sub = pos2sub(map, start);

% check whether start and goal points are valid
input_points_ind = [start_ind; goal_ind];
if any(map.occgrid(input_points_ind) == 1)
   warning  ('INPUT OR OUTOUT POINTS IN OBSTACLE !');
   return
end

% Get the information of cells in the map grid.
cell_total = numel(map.occgrid);
[cell_row, cell_col, cell_hei] = size(map.occgrid);

% Set all the cost of cell infinity
nodecost = inf(cell_row, cell_col, cell_hei);
nodecost_Astar = inf(cell_row, cell_col, cell_hei);

% Initial open list and close list
list_open = ones(cell_total,1);  %open list
list_close = zeros(cell_total, 1);  %close list

% Initial parents matrix and current node
parents = zeros(cell_total, 1);
current_ind = start_ind;

nodecost(current_ind) = 0;
nodecost_Astar(current_ind) = Hueristic(start_sub, goal_sub, map);

%% Main loop for path finding
%======Dijkstra======
if astar == false
    while true
        % Get current point with smallest cost
        [cost_min, current_ind] = min(nodecost(:));
        %Check if a new point found
        if isinf(cost_min)
            warning('NO PATH FOUND !');
            return
        end
        % Check if it is the goal
        if current_ind == goal_ind
            break
        end
        % Open and close list operation for new current point
        list_open(current_ind) = 0;  % Remove from open list
        list_close(current_ind) = 1;  % Add to close list
        nodecost(current_ind) = inf;
        
        % Get sub from ind of the current point
        [i, j, k] = ind2sub(size(nodecost), current_ind);
        current_sub = [i, j, k];
        
        % Find all neighbor points of the current point
        neighbors_sub = find_neighbors(i, j, k);
        % Check if neighbors in mape bound
        isbelow = neighbors_sub < 1;
        isover = neighbors_sub > [cell_row, cell_col, cell_hei];
        outrange = isbelow + isover;
        % Delete all neighbors outrange
        [delete_row, ~] = find(outrange > 0);
        neighbors_sub(delete_row,:) = [];  % Delete outrange neighbors
        % Get ind of the neighbors
        neighbors_ind = sub2ind(size(map.occgrid),neighbors_sub(:,1),...
            neighbors_sub(:,2), neighbors_sub(:,3));
        
       %% Check and update all neighbor costs
        % Check whether neighbirs are obstackes
        neighbors_occ = map.occgrid(neighbors_ind);
        [neighbors_occ_row,~] = find(neighbors_occ == 1);
        % Check points that in close list
        neighbors_close = list_close(neighbors_ind);
        [neighbors_close_row,~] = find(neighbors_close == 1);
        % Delete occ and close neighbors
        neighbors_delete_row = [neighbors_occ_row; neighbors_close_row];
        neighbors_sub(neighbors_delete_row,:) = [];
        % Re-Get ind of the neighbors
        neighbors_ind = sub2ind(size(map.occgrid),neighbors_sub(:,1),...
            neighbors_sub(:,2), neighbors_sub(:,3));
        
        cost_new =  cost_min + distance_points(neighbors_sub, current_sub, map);
        [update, ~] = find(cost_new < nodecost(neighbors_ind));
        nodecost(neighbors_ind(update)) = cost_new(update);
        parents(neighbors_ind(update)) = current_ind;
        list_close(neighbors_ind(update)) = 1;
        
    end
%======Astar======
else
    while true
        % Get current point with smallest cost
        [cost_min_Astar, current_ind] = min(nodecost_Astar(:));
        %Check if a new point found
        if isinf(cost_min_Astar)
            warning('NO PATH FOUND !');
            return
        end
        % Check if it is the goal
        if current_ind == goal_ind
            break
        end
        % Open and close list operation for new current point
        list_open(current_ind) = 0;  % Remove from open list
        list_close(current_ind) = 1;  % Add to close list
        nodecost_Astar(current_ind) = inf;
        
        % Get sub from ind of the current point
        [i, j, k] = ind2sub(size(nodecost_Astar), current_ind);
        current_sub = [i, j, k];
        
        % Find all neighbor points of the current point
        neighbors_sub = find_neighbors(i, j, k);
        % Check if neighbors in mape bound
        isbelow = neighbors_sub < 1;
        isover = neighbors_sub > [cell_row, cell_col, cell_hei];
        outrange = isbelow + isover;
        % Delete all neighbors outrange
        [delete_row, ~] = find(outrange > 0);
        neighbors_sub(delete_row,:) = [];  % Delete outrange neighbors
        % Get ind of the neighbors
        neighbors_ind = sub2ind(size(map.occgrid),neighbors_sub(:,1),...
            neighbors_sub(:,2), neighbors_sub(:,3));
        
       %% Check and update all neighbor costs
        % Check whether neighbirs are obstackes
        neighbors_occ = map.occgrid(neighbors_ind);
        [neighbors_occ_row,~] = find(neighbors_occ == 1);
        % Check points that in close list
        neighbors_close = list_close(neighbors_ind);
        [neighbors_close_row,~] = find(neighbors_close == 1);
        % Delete occ and close neighbors
        neighbors_delete_row = [neighbors_occ_row; neighbors_close_row];
        neighbors_sub(neighbors_delete_row,:) = [];
        % Re-Get ind of the neighbors
        neighbors_ind = sub2ind(size(map.occgrid),neighbors_sub(:,1),...
            neighbors_sub(:,2), neighbors_sub(:,3));
        
        cost_min = nodecost(current_ind);
        cost_new =  cost_min + distance_points(neighbors_sub, current_sub, map);
        cost_new_Astar =  cost_new + Hueristic(neighbors_sub, goal_sub, map);
        [update, ~] = find(cost_new < nodecost(neighbors_ind));
        nodecost(neighbors_ind(update)) = cost_new(update);
        nodecost_Astar(neighbors_ind(update)) = cost_new_Astar(update);
        parents(neighbors_ind(update)) = current_ind;
        list_close(neighbors_ind(update)) = 1;
    end
end
    
%% Generate path according to parents
path_ind = [];
    path_ind = [path_ind; goal_ind];
    while (parents(path_ind(1)) ~= 0)
        path_ind = [parents(path_ind(1)); path_ind];
    end
    [path_sub_x, path_sub_y, path_sub_z] = ind2sub(size(map.occgrid), path_ind);
    path_sub = [path_sub_x, path_sub_y, path_sub_z];
    path = sub2pos(map, path_sub);
    path(1,:) = start;
    path(end, :) = goal;

%% Compute total expanded nodes
num_expanded = sum(list_close);
toc;

end

%% Function to find neighbor points of a point
function neighbor = find_neighbors(i, j, k)
    neighbor   = [i - 1, j - 1, k - 1;
        i - 1, j - 1,     k;
        i - 1, j - 1, k + 1;
        i - 1,     j, k - 1;
        i - 1,     j,     k;
        i - 1,     j, k + 1;
        i - 1, j + 1, k - 1;
        i - 1, j + 1,     k;
        i - 1, j + 1, k + 1;
        i    , j - 1, k - 1;
        i    , j - 1,     k;
        i    , j - 1, k + 1;
        i    ,     j, k - 1;
        i    ,     j, k + 1;
        i    , j + 1, k - 1;
        i    , j + 1,     k;
        i    , j + 1, k + 1;
        i + 1, j - 1, k - 1;
        i + 1, j - 1,     k;
        i + 1, j - 1, k + 1;
        i + 1,     j, k - 1;
        i + 1,     j,     k;
        i + 1,     j, k + 1;
        i + 1, j + 1, k - 1;
        i + 1, j + 1,     k;
        i + 1, j + 1, k + 1];
end

%% Function of bound check
 function result = boundcheck(input_points, map)
    if all(all(input_points >= map.bound_xyz(1:3) == 1)) && all(all(input_points <= map. bound_xyz(4:6) == 1))
       result = 0 ;
    else
       result = 1;
    end
end

%% Function to calculate distance between two points
function dist = distance_points(children, parents, map)
   diff_sub = abs(children - parents);
   dist = sqrt((map.res_xyz(1) * diff_sub(:, 1)).^2 + ...
            (map.res_xyz(2) * diff_sub(:, 2)).^2 + (map.res_xyz(3) * diff_sub(:, 3)).^2);
end

%% Function to calculate distance between two points
% function dist = distance_points(children, parents, map)
%    diff_sub = abs(children - parents);
%    diff_sub_sort = sort(diff_sub, 2);
%    diff_min1 = diff_sub_sort(:, 1);
%    diff_min2 = diff_sub_sort(:, 2);
%    diff_min3 = diff_sub_sort(:, 3);
%    dist = sqrt(map.res_xyz(1).^2 + map.res_xyz(2).^2 + map.res_xyz(3).^2) * diff_min1 +...
%        sqrt(map.res_xyz(1).^2 + map.res_xyz(2).^2) * (diff_min2 - diff_min1) + map.res_xyz(1) * (diff_min3 - diff_min2);
% %    dist = sqrt((map.res_xyz(1) * diff_sub(:, 1)).^2 + ...
% %             (map.res_xyz(2) * diff_sub(:, 2)).^2 + (map.res_xyz(3) * diff_sub(:, 3)).^2);
% end

%% Function to compute Huristics distance
function dist = Hueristic(children, goal, map)
    diff_sub = abs(goal - children);
    diff_pos = [map.res_xyz(1) * diff_sub(:,1), map.res_xyz(2)...
                    * diff_sub(:,2), map.res_xyz(3) * diff_sub(:,3)];
    dist = sqrt(diff_pos(:,1).^2+ diff_pos(:,2).^2 + diff_pos(:,3).^2);
end

%% Function to compute Huristics distance
% function dist = Hueristic(children, goal, map)
%     diff_sub = abs(goal - children);
%     diff_sub_sort = sort(diff_sub, 2);
%     diff_min1 = diff_sub_sort(:, 1);
%     diff_min2 = diff_sub_sort(:, 2);
%     diff_min3 = diff_sub_sort(:, 3);
% %     diff_pos = [map.res_xyz(1) * diff_sub(:,1), map.res_xyz(2)...
% %                     * diff_sub(:,2), map.res_xyz(3) * diff_sub(:,3)];
% %     dist = sqrt(diff_pos(:,1).^2+ diff_pos(:,2).^2 + diff_pos(:,3).^2);
%     dist = sqrt(map.res_xyz(1).^2 + map.res_xyz(2).^2 + map.res_xyz(3).^2) * diff_min1 +...
%        sqrt(map.res_xyz(1).^2 + map.res_xyz(2).^2) * (diff_min2 - diff_min1) + map.res_xyz(1) * (diff_min3 - diff_min2);
% end
