function [field, color_map] = constructMap(row,column,obstacle_rate)
%传入需要构建地图的row和column和obstacle_rate，返回一个随机障碍物的地图
%   [field, color_map] = constructMap(row,column,obstacle_rate)
global color_map
color_map = [1 1 1;...  % 1-白色-空地
    0 0 0;...  % 2-黑色-障碍物
    0 1 1;...  % 3-青蓝色-start_pos
    1 0 0;...  % 4-红色-goal_pos
    0 1 0;...  % 5-绿色
    0 0 1;...  % 6-蓝色
    1 1 0;...  % 7-黄色
    1 0 1];...  % 8-品红色
field = ones(row, column);
numb_obs = floor(row*column*obstacle_rate);
index_obs = randperm(row*column, numb_obs);
field(index_obs) = 2;  % 对应的color_map是black
end

