ip = "192.168.16.84";
neatov3.connect(ip);
sensors = 0;
neato_rect = bounds(0, 0, 0.381, 0.381);
encoder_left_list = [0];
encoder_right_list = [0];
time = [0];
total_time = 20;
tic;

threshold = 0.1;

v0 = 0.1;
neato_curr_vel_left = v0;
neato_curr_vel_right = v0;

while toc <= total_time
    neatov3.setVelocities(neato_curr_vel_left, neato_curr_vel_right);
    sensors = neatov3.receive();
    encoder_left_list(end+1) = sensors.encoders(1);
    encoder_right_list(end+1) = sensors.encoders(2);
    time(end+1) = toc;
    v_left = differential(encoder_left_list, time);
    v_right = differential(encoder_right_list, time);
    speed = abs((v_left + v_right) / 2);
    rotation_rate = (v_right - v_left) / 0.24;
    rotation_vector = integration(rotation_rate, time, pi / 2);
    rotation_vector_x = cos(rotation_vector);
    rotation_vector_y = sin(rotation_vector);
    neato_velocity_x = speed .* rotation_vector_x(1:end-1);
    neato_velocity_y = speed .* rotation_vector_y(1:end-1);
    neato_pos_x = integration(neato_velocity_x, time, 0);
    neato_pos_y = integration(neato_velocity_y, time, 0);

    %plot_lidar_data = [-lidar_y,lidar_x]
    
    [lidar_x, lidar_y] = polar_to_cartesian(sensors.ranges, sensors.thetasInRadians);
    plot(-lidar_y, lidar_x, "."); hold on
        plot(final_position - [neato_pos_x(end), neato_pos_y(end)], ".", Color="red", MarkerSize=20)
        plot(0, 0, ".", MarkerSize=20, Color=[0.3, 0.3, 1])
    hold off
    % change tactics if neato is within a distance to a dot (obstacle).
    pause(1);

    left_most_threshold = -0.4;
    right_most_threshold = 0.4;
    
    upper_limit_threshold = 0.7;
    lower_limit_threshold = 0.2;
    

    if (any((left_most_threshold<lidar_y & lidar_y < right_most_threshold)) & any((lower_limit_threshold<lidar_x & lidar_x<upper_limit_threshold)))
            
        upper_crossing_points = find((lower_limit_threshold<lidar_x & lidar_x<upper_limit_threshold))
        
        focus_point_y = lidar_x(min(upper_crossing_points))
        focus_point_x = lidar_y(min(upper_crossing_points))

        focus_point = [focus_point_x, focus_point_y]

        %points of a single obsticle are within 0.02 away from each other on the x axis
        %within 0.015 on the y axis.

        %find leftmost point of obstacle.   
       
           closest_right_element = max(find( (focus_point_x<lidar_y & lidar_y < focus_point_x+0.02)  & (focus_point_y<lidar_x & lidar_x < focus_point_y+0.015 )))
           %new_focus_point = [lidar_y(closest_element), lidar_x(closest_element)]
        break;
    end
end

sensors = neatov3.receive();
disp("disconnected");
neatov3.disconnect();