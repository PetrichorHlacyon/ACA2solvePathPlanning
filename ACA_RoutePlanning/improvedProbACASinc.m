function [dist_best, path_best, per_iter_best,per_iter_avg] = improvedProbACA(map, start_pos_ind, goal_pos_ind)
%[dist_best, path_best, per_iter_best,per_iter_avg] = improvedProbACA(map, start_pos_ind, goal_pos_ind)
%APACO,改进的状态转移概率
%无自适应效果

%% dist_matrix定义为所有点到目标点终点的距离
[row_map, col_map] = size(map);
gap_numb = row_map*col_map;
gap_size = 1;  % 栅格大小默认1
[goal_pos_sub(1), goal_pos_sub(2)] = ind2sub([row_map, col_map],goal_pos_ind);
[start_pos_sub(1), start_pos_sub(2)] = ind2sub([row_map, col_map],start_pos_ind);
for i = 1:row_map
    for j = 1:col_map
        dist_matrix(i,j) = (1/2.*gap_size.*norm(goal_pos_sub - [i,j]))...
            ./ (1/2.*gap_size.*norm(start_pos_sub - [i,j]));
        %         dist_matrix(i,j) = gap_size.*norm(goal_pos_sub - [i,j]);
        if dist_matrix(i,j) == 0
            dist_matrix(i,j) = 0.01;
        elseif dist_matrix(i,j) == inf
            dist_matrix(i,j) = 60;
        end
    end
end
dist_matrix = reshape(dist_matrix,gap_numb,1);
dist_start2goal = gap_size*norm(goal_pos_sub-start_pos_sub);
%% init param
ant_numb = 50;  % 蚂蚁数量
alpha = 1.1;  % 信息素的重要程度因子
beta = 7;  % 启发式函数的重要程度因子
gamma = 3;  % 方向引导因子重要程度因子
rho = 0.2;  % 信息素挥发因子
fai0 = acos((goal_pos_sub(2)-start_pos_sub(2)) ...
    /dist_start2goal);  % 起点和终点的角度
%acos返回弧度值
fai1 = 2*pi/180;
% fai = 1/(cos);
delta0 = 0.3;
nu = 14;
mu = 1.6;
Q = 1;
Iteration = 300;  % 外循环迭代次数
Iteration_cur = 1;  % 当前迭代次数
path_best = [];  % 每次迭代的最佳路径
dist_best = inf;
%每次就把dist_cur和dist_best(Iteration)比较
path_cur = inf(ant_numb, gap_numb);
dist_cur = zeros(ant_numb, 1);
tau = ones(gap_numb);  % 信息素浓度，从城市i到j，这是个对称矩阵
eta = 1./dist_matrix;  %单个蚂蚁邻域的启发式信息
per_iter_best = zeros(Iteration, 1);
per_iter_avg = zeros(Iteration, 1);
%% ACA major part
while Iteration_cur <= Iteration
    r0 = delta0*abs(sinc(nu*Iteration_cur/Iteration).^mu);
    path_cur = inf(ant_numb, gap_numb);
    dist_cur = zeros(ant_numb, 1);
    %定义每个蚂蚁的起点
    %起点的设置要注意，是出发点为起点
    path_cur(:,1) = start_pos_ind;
    city_index_array = 1:gap_numb;
    for i = 1:ant_numb
        %第几只蚂蚁
        %         for j = 1:length(dist_matrix)%一个蚂蚁只能在他的邻域内移动
        %             %需要前往的城市个数
        fisrt_inf_ind = find(isinf(path_cur(i,:)),1);
        tabu = path_cur(i,1:fisrt_inf_ind-1);  % 禁忌表
        allow_total = city_index_array(~ismember(city_index_array, tabu));
        pos_cur_ind = tabu(end); %当前的索引
        allow_local = getNeighAvailablePoint(map,tabu,pos_cur_ind,gap_size);
        j = 2;
        %         while ~isempty(allow_total) || isempty(allow_local)
        while tabu(end) ~= goal_pos_ind && ...
                ~isempty(allow_local)
            %不需要allow_total全部清空，只需要到达终点就可以了
            %轮盘赌添加路径
            fai = allow_local;  % 同样是定义大小
            P_cur2allow = allow_local;  % 使P和allow有相同大小
            %这里如果是Route Planning问题那么就要遍历邻节点
            for k = 1:length(allow_local)
                allow_ind = allow_local(k);
                [allow_sub(1),allow_sub(2)] = ind2sub([row_map,col_map],allow_ind);
                m = abs(allow_sub(2)-goal_pos_sub(2))/(gap_size*norm(allow_sub-goal_pos_sub)+eps);
                n = abs(allow_sub(1)-goal_pos_sub(1))/(gap_size*norm(allow_sub-goal_pos_sub)+eps);
                fai = 1./(m*cos(fai0)+n*sin(fai0)+eps);
                if fai < cos(fai1)
                    fai = 0.1;
                end
                P_cur2allow(k) = (tau(tabu(end),allow_local(k)).^alpha).*(eta(tabu(end)).^beta).*(fai.^gamma);
            end
            if sum(P_cur2allow) == 0 || sum(P_cur2allow == inf) ~=0 %每个都等概率
                break
            end
            P = allow_local;
            for dd = 1:length(allow_local)
                if rand < r0
                    P(dd) = P_cur2allow(k);
                else
                    P(dd) = P_cur2allow(dd)./sum(P_cur2allow);
                end
            end
            P = P_cur2allow./sum(P_cur2allow);
            Pc = cumsum(P);
            target_index = find(Pc >= rand);
            target = allow_local(target_index(1));
            path_cur(i, j) = target;
            j = j + 1;
            %计算path_cur里面的最后一个非inf元素的索引
            fisrt_inf_ind = find(isinf(path_cur(i,:)),1);
            %更新tabu和allow_local allow_total
            tabu(end+1) = target;
            allow_total(allow_total == target) = [];
            allow_local = getNeighAvailablePoint(map,tabu,target,gap_size);
        end
    end
    %     end
    %cal dist_cur
    dist_cur = zeros(ant_numb, 1);
    for q = 1:ant_numb
        fisrt_inf_ind = find(isinf(path_cur(q,:)),1);
        if path_cur(q,fisrt_inf_ind-1) == goal_pos_ind
            for p = 1:fisrt_inf_ind-1-1
                %一个-1是到最后一个不是inf的数，还有一个
                %是为了数组越界
                x1_ind = path_cur(q,p);
                x2_ind = path_cur(q,p+1);
                [x1_sub(1),x1_sub(2)] = ind2sub([row_map, col_map],x1_ind);
                [x2_sub(1),x2_sub(2)] = ind2sub([row_map, col_map],x2_ind);
                dist_cur(q) = dist_cur(q) + gap_size.*norm(x1_sub-x2_sub);
            end
        else
            dist_cur(q) = inf;
        end
    end
    per_iter_avg(Iteration_cur) = mean(dist_cur);
    %cal dist_best
    [min_dist, min_index] = min(dist_cur);
    if min_dist < dist_best
        dist_best = min_dist;
        path_best = path_cur(min_index, :);
    end
    %update tao
    delta_tau_k = Q./dist_cur;
    delta_tau = zeros(gap_numb);
    for i = 1:ant_numb
        fisrt_inf_ind = find(isinf(path_cur(i,:)),1);
        for j = 1:fisrt_inf_ind-2
            delta_tau(path_cur(i,j),path_cur(i,j+1)) = ...
                delta_tau(path_cur(i,j),path_cur(i,j+1)) ...
                + delta_tau_k(i);
        end
        delta_tau(path_cur(i,fisrt_inf_ind-1),path_cur(i,1)) = ...
            delta_tau(path_cur(i,fisrt_inf_ind-1),path_cur(i,1)) ...
            + delta_tau_k(i);
    end
    per_iter_best(Iteration_cur) = dist_best;
    
    tau = (1 - rho).*tau + delta_tau;
    Iteration_cur = Iteration_cur + 1;
    
end
end
