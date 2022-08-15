function allow_local = getNeighAvailablePoint(field,tabu,pos_cur_ind,gap_size);
%allow_local = getNeighAvailablePoint(field,tabu,pos_cur_ind);
%获取邻近的没有走过的点的线性索引
%tabu是禁忌表

allow_local = [];
[r,c] = size(field);
% [pos_cur_sub(1),pos_cur_sub(2)] = ind2sub([r,c],pos_cur_ind);
%障碍物的颜色值设为2
neigh_nodes = getNeighNodes(pos_cur_ind,field,2,gap_size);
for i = 1:length(neigh_nodes) 
    if ~isinf(neigh_nodes(i,2)) && ~isinf(neigh_nodes(i,1))
        if ~ismember(neigh_nodes(i,1),tabu)
            %邻节点不是障碍物，也在可行域内，那么就允许走
            allow_local = [allow_local;neigh_nodes(i,1)];
        end
    end
end

end

