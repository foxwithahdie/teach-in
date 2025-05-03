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

final_position = [0 10]

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
    
    coordinate_x = -1*lidar_y;
    coordinate_y = lidar_x;
    
    left_most_threshold = -0.4;
    right_most_threshold = 0.4;
    
    upper_limit_threshold = 0.7;
    lower_limit_threshold = 0.2;
    
    for i = 1:360
        (left_most_threshold<coordinate_x(i) & coordinate_x(i) < right_most_threshold) & (lower_limit_threshold<coordinate_y(i) & coordinate_y(i)<upper_limit_threshold)
        
        if ( (left_most_threshold<coordinate_x(i) & coordinate_x(i) < right_most_threshold) & (lower_limit_threshold<coordinate_y(i) & coordinate_y(i)<upper_limit_threshold) )
            
        focus_point = [coordinate_x(i) coordinate_y(i)]  
        break;
    
        % 
        % %points of a single obsticle are within 0.02 away from each other on the x axis
        % %within 0.015 on the y axis.
        % 
        % %find leftmost point of obstacle.   
        % 
        %closest_right_index = find( (focus_point(1)<coordinate_x & coordinate_x < focus_point(1)+0.02)  & (focus_point(2)-0.015<coordinate_y & coordinate_y < focus_point(2)+0.015 ))
        
       % closest_right_point = [coordinate_x(closest_right_index),coordinate_y(closest_right_index)]

           %new_focus_point = [lidar_y(closest_element), lidar_x(closest_element)]
        
        end
    break;
    end
    % break;
end

sensors = neatov3.receive();
disp("disconnected");
neatov3.disconnect();