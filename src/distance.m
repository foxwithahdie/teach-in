function D = distance(current_neato_pos, goal)
    dist = sqrt(norm(goal - current_neato_pos));
    D = dist/0.1;
end