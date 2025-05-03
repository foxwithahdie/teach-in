ip = "192.168.16.84";
neatov3.connect(ip);
sensors = 0;
neato_rect = bounds(0, 0, 0.381, 0.381);
encoder_left_list = [0];
encoder_right_list = [0];
%setup for data collection in 6.4
encoderdata=zeros(200,3);
time = [0];
total_time = 20;
tic;

threshold = 0.1;

d = 0.24; %distance between wheels on the Neato

v0 = 0.1;
neato_curr_vel_left = v0;
neato_curr_vel_right = v0;

goal = [0, 5];

while toc <= total_time
    neatov3.setVelocities(neato_curr_vel_left, neato_curr_vel_right);
    sensors = neatov3.receive();
    encoder_left_list(end+1) = sensors.encoders(1);
    encoder_right_list(end+1) = sensors.encoders(2);
    time(end+1) = toc;
    v_left = differential(encoder_left_list, time);
    v_right = differential(encoder_right_list, time);
    speed = abs((v_left + v_right) / 2);
    rotation_rate = (v_right - v_left) / d;
    rotation_vector = integration(rotation_rate, time, pi / 2);
    rotation_vector_x = cos(rotation_vector);
    rotation_vector_y = sin(rotation_vector);
    neato_velocity_x = speed .* rotation_vector_x(1:end-1);
    neato_velocity_y = speed .* rotation_vector_y(1:end-1);
    neato_pos_x = integration(neato_velocity_x, time, 0);
    neato_pos_y = integration(neato_velocity_y, time, 0);
    
    optimizing_params = params(0.5, 0.9, 500, 1e-4);
    guess_pos = run_gradient_descent(@(curr_neato_pos, end_goal) costfunc(curr_neato_pos, end_goal), [neato_pos_x(end), neato_pos_y(end)], goal, optimizing_params);

    new_pos_x = [neato_pos_x(2:end); guess_pos(1)];
    new_pos_y = [neato_pos_y(2:end); guess_pos(2)];

    velocity_x = differential(new_pos_x, time);
    velocity_y = differential(new_pos_y, time);

    velocity = (velocity_x.^2 + velocity_y.^2).^0.5;
    speed = norm(velocity);

    tangent_vec_x = velocity_x ./ norm(velocity_x);
    tangent_vec_y = velocity_y ./ norm(velocity_y);

    q_vec_x = diff(tangent_vec_x);
    q_vec_y = diff(tangent_vec_y);

    ang_vel = (tangent_vec_x .* q_vec_y) - (tangent_vec_y .* q_vec_x);
    
    vl = speed - ((d/2)*ang_vel) 
    vr = speed + ((d/2)*ang_vel)
    
    [lidar_x, lidar_y] = polar_to_cartesian(sensors.ranges, sensors.thetasInRadians);
    
    % change tactics if neato is within a distance to a dot (obstacle).
    pause(1);
end

sensors = neatov3.receive();
disp("disconnected");
neatov3.disconnect();




