function cost = costfunc(current_neato_pos, goal)
    dist = goal-current_neato_pos;
    cost = (dist)^2; %Squared distance
end