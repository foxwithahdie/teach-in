ip = "192.168.16.84";
neatov3.connect(ip);
sensors = 0;
neato_rect = bounds(0, 0, 0.381, 0.381);
encoder_left_list = [0];
encoder_right_list = [0];
time = [0];
lidar_data = {};
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
    
    
    [lidar_x, lidar_y] = polar_to_cartesian(sensors.ranges, sensors.thetasInRadians);
    lidar_data{end+1} = [lidar_x + neato_pos_x(end), lidar_y + neato_pos_y(end)];
    plot(-lidar_y, lidar_x, "."); hold on
        plot(final_position - [neato_pos_x(end), neato_pos_y(end)], ".", Color="red", MarkerSize=20)
        plot(0, 0, ".", MarkerSize=20, Color=[0.3, 0.3, 1])
    hold off
    
    % change tactics if neato is within a distance to a dot (obstacle).
    if (any((0.1<lidar_x & lidar_x < 0.5)) && any((0.2<lidar_y & lidar_y<0.7)))
        break;
    end
    pause(1);
end

sensors = neatov3.receive();
disp("disconnected");
neatov3.disconnect();