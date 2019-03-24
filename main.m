%% This script is used as main script to start simulation

% map = load_map('maps/map1.txt', 0.1, 1.0, 0.25);
% start = [6.0, -4.9, 3.2];
% stop = [8.0, 18.0, 3.0];

map = load_map('maps/map3.txt', 0.1, 1.0, 0.25);
start = [2.0, 0.9, 1.2];
stop = [20.0, 5.0, 6.0];

% map = load_map('maps/mymap.txt', 1.0, 1.0, 0.25);
% start = [10.0, 2.0, 10.0];
% stop = [10.0, 90.0, 40.0];

path = dijkstra(map, start, stop, true);
trajectory_generator([], [], map, path);
% trajectory_generator2([], [], map, path);
trajectory = test_trajectory(start, stop, map, path, true);