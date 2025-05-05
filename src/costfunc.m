function cost = costfunc(current_neato_pos, goal)
    dist = sqrt((goal(2)-current_neato_pos(2))^2 + (goal(1)-current_neato_pos(1))^2);
    cost = dist; %Squared distance
end