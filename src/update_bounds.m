function update_bounds(bounds_struct, new_x, new_y)
    arguments
        bounds_struct struct
        new_x (1, 1) double
        new_y (1, 1) double
    end

    bounds_struct.left = new_x - bounds_struct.width / 2;
    bounds_struct.right = new_x + bounds_struct.width / 2;
    bounds_struct.top = new_y + bounds_struct.height / 2;
    bounds_struct.bottom = new_y - bounds_struct.height / 2;
    bounds_struct.x = new_x;
    bounds_struct.y = new_y;

end