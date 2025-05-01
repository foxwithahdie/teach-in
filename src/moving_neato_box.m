ip = "192.168.16.84";
neatov3.connect(ip);
sensors = 0;
neato_rect = bounds(0, 0, 0.381, 0.381);
encoder_left_list = [0];
encoder_right_list = [0];
time = [0];
total_time = 10;
tic;

v0 = 0.1;
neato_curr_vel_left = v0;
neato_curr_vel_right = v0;

final_position = [0, 5];

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
    plot(-lidar_y, lidar_x, "."); hold on
        plot(final_position - [neato_pos_x(end), neato_pos_y(end)], ".", Color="red", MarkerSize=20)
    pause(.1);
    update_bounds(neato_rect, neato_pos_x(end), neato_pos_y(end));
    hold off

end

sensors = neatov3.receive();
disp("disconnected");
neatov3.disconnect();