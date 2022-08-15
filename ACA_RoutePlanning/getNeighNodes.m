function NeighNodes = getNeighNodes(LineIndex,Map,ObstacleColorValue,GridSize)
%传入需要在Map中查找邻节点的线性索引，返回他的邻近节点（以线性索引的形式）
%   NeighNodes = getNeighNodes(LineIndex, Map, ObstacleColorValue,GridSize)
%PosIndex 就是线性索引的行列表达形式


[row_map, column_map] = size(Map);
sz = [row_map, column_map];
[row_PosIndex, column_PosIndex] = ind2sub([row_map, column_map], LineIndex);
NeighNodes = inf(8, 2);  % 第二列放邻近节点到中心节点的距离

%check left,up
if row_PosIndex-1 >= 1 && column_PosIndex-1 >= 1 && ...
        row_PosIndex-1 <= row_map && column_PosIndex-1 <= column_map
    %左上角没有出界
    ChildNodeSub = [row_PosIndex-1, column_PosIndex-1];
    ChildNodeInd = sub2ind(sz, ChildNodeSub(1), ChildNodeSub(2));
    NeighNodes(1,1) = ChildNodeInd;
    if Map(ChildNodeSub(1), ChildNodeSub(2)) ~= ObstacleColorValue
        cost = GridSize*norm(ChildNodeSub-[row_PosIndex, column_PosIndex]);
        NeighNodes(1,2) = cost;
    end
end

%check left
if row_PosIndex >= 1 && column_PosIndex-1 >= 1 && ...
        row_PosIndex <= row_map && column_PosIndex-1 <= column_map
    ChildNodeSub = [row_PosIndex, column_PosIndex-1];
    ChildNodeInd = sub2ind(sz, ChildNodeSub(1), ChildNodeSub(2));
    NeighNodes(2,1) = ChildNodeInd;
    if Map(ChildNodeSub(1), ChildNodeSub(2)) ~= ObstacleColorValue
        cost = GridSize*norm(ChildNodeSub-[row_PosIndex, column_PosIndex]);
        NeighNodes(2,2) = cost;
    end
end

%check left,down
if row_PosIndex+1 >= 1 && column_PosIndex-1 >= 1 && ...
        row_PosIndex+1 <= row_map && column_PosIndex-1 <= column_map
    ChildNodeSub = [row_PosIndex+1, column_PosIndex-1];
    ChildNodeInd = sub2ind(sz, ChildNodeSub(1), ChildNodeSub(2));
    NeighNodes(3,1) = ChildNodeInd;
    if Map(ChildNodeSub(1), ChildNodeSub(2)) ~= ObstacleColorValue
        cost = GridSize*norm(ChildNodeSub-[row_PosIndex, column_PosIndex]);
        NeighNodes(3,2) = cost;
    end
end

%check down
if row_PosIndex+1 >= 1 && column_PosIndex >= 1 && ...
        row_PosIndex+1 <= row_map && column_PosIndex <= column_map
    ChildNodeSub = [row_PosIndex+1, column_PosIndex];
    ChildNodeInd = sub2ind(sz, ChildNodeSub(1), ChildNodeSub(2));
    NeighNodes(4,1) = ChildNodeInd;
    if Map(ChildNodeSub(1), ChildNodeSub(2)) ~= ObstacleColorValue
        cost = GridSize*norm(ChildNodeSub-[row_PosIndex, column_PosIndex]);
        NeighNodes(4,2) = cost;
    end
end

%check right,down
if row_PosIndex+1 >= 1 && column_PosIndex+1 >= 1 && ...
        row_PosIndex+1 <= row_map && column_PosIndex+1 <= column_map
    ChildNodeSub = [row_PosIndex+1, column_PosIndex+1];
    ChildNodeInd = sub2ind(sz, ChildNodeSub(1), ChildNodeSub(2));
    NeighNodes(5,1) = ChildNodeInd;
    if Map(ChildNodeSub(1), ChildNodeSub(2)) ~= ObstacleColorValue
        cost = GridSize*norm(ChildNodeSub-[row_PosIndex, column_PosIndex]);
        NeighNodes(5,2) = cost;
    end
end

%check right
if row_PosIndex >= 1 && column_PosIndex+1 >= 1 && ...
        row_PosIndex <= row_map && column_PosIndex+1 <= column_map
    ChildNodeSub = [row_PosIndex, column_PosIndex+1];
    ChildNodeInd = sub2ind(sz, ChildNodeSub(1), ChildNodeSub(2));
    NeighNodes(6,1) = ChildNodeInd;
    if Map(ChildNodeSub(1), ChildNodeSub(2)) ~= ObstacleColorValue
        cost = GridSize*norm(ChildNodeSub-[row_PosIndex, column_PosIndex]);
        NeighNodes(6,2) = cost;
    end
end

%check right,up
if row_PosIndex-1 >= 1 && column_PosIndex+1 >= 1 && ...
        row_PosIndex-1 <= row_map && column_PosIndex+1 <= column_map
    ChildNodeSub = [row_PosIndex-1, column_PosIndex+1];
    ChildNodeInd = sub2ind(sz, ChildNodeSub(1), ChildNodeSub(2));
    NeighNodes(7,1) = ChildNodeInd;
    if Map(ChildNodeSub(1), ChildNodeSub(2)) ~= ObstacleColorValue
        cost = GridSize*norm(ChildNodeSub-[row_PosIndex, column_PosIndex]);
        NeighNodes(7,2) = cost;
    end
end

%check up
if row_PosIndex-1 >= 1 && column_PosIndex >= 1 && ...
        row_PosIndex-1 <= row_map && column_PosIndex <= column_map
    ChildNodeSub = [row_PosIndex-1, column_PosIndex];
    ChildNodeInd = sub2ind(sz, ChildNodeSub(1), ChildNodeSub(2));
    NeighNodes(8,1) = ChildNodeInd;
    if Map(ChildNodeSub(1), ChildNodeSub(2)) ~= ObstacleColorValue
        cost = GridSize*norm(ChildNodeSub-[row_PosIndex, column_PosIndex]);
        NeighNodes(8,2) = cost;
    end
end
end

