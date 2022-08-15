function flag = isCrossObstacle(field, parent_node, child_node)
%flag = isCrossObstacle(field, parent_node, child_node)
%parent_node,child_node都是线性索引


[row_map, col_map] = size(field);
flag = 0;
if field(child_node) == 2
    flag = 1;
    return
end
[parent_node(1), parent_node(2)] = ind2sub([row_map, col_map], parent_node);
[child_node(1), child_node(2)] = ind2sub([row_map, col_map], child_node);

P2 = parent_node ;
P1 = child_node ;

row_min = min([P1(1), P2(1)]);
row_max = max([P1(1), P2(1)]);
col_min = min([P1(2), P2(2)]);
col_max = max([P1(2), P2(2)]);

for i = row_min:row_max
    for j = col_min:col_max
        if field(i,j) == 2
            P = [i,j];
            d = abs(det([P2-P1;P-P1])/norm(P2-P1));
            %----------这里也可以优化------------------
            if d < sqrt(2)  %尽量取大一点，不能取2咯
                flag = 1;
                return
            end
        end
    end
end
end

