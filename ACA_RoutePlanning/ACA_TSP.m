function [dist_best, path_best, per_iter_best,per_iter_avg] = ACA_TSP( city_pos)
%Input:起始点的索引序号和城市坐标（每一行表示一个城市，每一列表示x或者y）
%Output:[dist_best, path_best, per_iter_best,per_iter_avg]
%       最短路径长度，最短路径，每次的最好路径数组，每次平均路径数组
%Watch : 蚂蚁的数量<=城市的数量具体看第48行，可以进行优化

[city_numb, ~] = size(city_pos);
%% cal dist_matrix
coord_x = city_pos(:,1);
coord_y = city_pos(:,2);
temp_x_1 = coord_x*ones(1,city_numb);
temp_x_2 = temp_x_1';
temp_y_1 = coord_y*ones(1,city_numb);
temp_y_2 = temp_y_1';
dist_matrix = sqrt((temp_x_2 - temp_x_1).^2 + ...
    (temp_y_2 - temp_y_1).^2);
clear coord_x coord_y temp_x_1 temp_x_2 temp_y_1 temp_y_2

%% init param
ant_numb = 20;  % 蚂蚁数量
alpha = 5;  % 信息素的重要程度因子
beta = 1;  % 启发式函数的重要程度因子
rho = 0.1;  % 信息素挥发因子
Q = 10;
Iteration = 400;  % 外循环迭代次数
Iteration_cur = 1;  % 当前迭代次数
path_best = inf(1, city_numb);
dist_best = inf;
path_cur = inf(ant_numb, city_numb);
dist_cur = zeros(ant_numb, 1);
tau = ones(city_numb);  % 信息素浓度，从城市i到j，这是个对称矩阵
eta = 1./dist_matrix;  %启发式信息
per_iter_best = zeros(Iteration, 1);
per_iter_avg = zeros(Iteration, 1);
% tabu = zeros(ant_numb, city_numb);  % 第k只蚂蚁已经走过的路
% allow = zeros(ant_numb, city_numb);  % 第k只蚂蚁还可以走的路
%上面两个参数也可以每只蚂蚁处理一次，因为他们都是在变化的
%% init solution space
% for i = 1:ant_numb
%     allow(i,:) = city_index_array(~ismember(city_index_array, tabu));
% end
%这里这样弄就会使得allow数组过于巨大，并且不容易赋值（左侧是1000，右侧维度不是1000）
%注意区分数组的赋值和数组的重新替换，如果拿数组的一部分出来，将另外
%一个数组赋值给他，那么要求维度相同，如果拿全部数组则不需要
%a = [1,2;3,4];a(2,1) = [2,3] is wrong;a = [2,3] is right
%% ACA major part
while Iteration_cur <= Iteration
    path_cur(:,1) = randperm(city_numb, ant_numb);
    city_index_array = 1:city_numb;
    for i = 1:ant_numb
        %第几只蚂蚁
        for j = 2:city_numb
            %需要前往的城市个数
            tabu = path_cur(i,1:(j-1));
            allow = city_index_array(~ismember(city_index_array, tabu));
            P_cur2allow = allow;  % 使P和allow有相同大小
            %这里如果是Route Planning问题那么就要遍历邻节点
            for k = 1:length(allow)
                P_cur2allow(k) = tau(tabu(end),allow(k)).^alpha.*eta(tabu(end),allow(k)).^beta;
            end
            P = P_cur2allow./sum(P_cur2allow);
            %通过轮盘赌的方式选择前往那个城市（同样也可以换）
            Pc = cumsum(P);
            target_index = find(Pc >= rand);
            target = allow(target_index(1));
            path_cur(i, j) = target;
        end
    end
    %cal dist_cur
    dist_cur = zeros(ant_numb, 1);
    for q = 1:ant_numb
        for p = 1:city_numb-1
            dist_cur(q) = dist_cur(q) + dist_matrix(path_cur(q,p), path_cur(q,p+1));
        end
        dist_cur(q) = dist_cur(q) + dist_matrix(path_cur(q,end),path_cur(q,1));
    end
    per_iter_avg(Iteration_cur) = mean(dist_cur);
    %cal dist_best
    [min_dist, min_index] = min(dist_cur);
    per_iter_best(Iteration_cur) = min_dist;
    if min_dist < dist_best
        dist_best = min_dist;
        path_best = path_cur(min_index, :);
    end
    %update tao
    delta_tau_k = Q./dist_cur;
    delta_tau = zeros(city_numb);
    for i = 1:ant_numb
        for j = 1:city_numb-1
            delta_tau(path_cur(i,j),path_cur(i,j+1)) = ...
                delta_tau(path_cur(i,j),path_cur(i,j+1)) ...
                + delta_tau_k(i);
        end
        delta_tau(path_cur(i,end),path_cur(i,1)) = ...
            delta_tau(path_cur(i,end),path_cur(i,1)) ...
            + delta_tau_k(i);
    end
    tau = (1 - rho).*tau + delta_tau;
    Iteration_cur = Iteration_cur + 1;
end

