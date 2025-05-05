ip = "192.168.16.84";

neatov3.connect(ip);

sensors = 0;
encoder_left_list = [0];
encoder_right_list = [0];
neato_pos_x = [0];
neato_pos_y = [0];
time = [0];

total_time = 20;
tic;

threshold = 0.1;

v0 = 0.1;
neato_curr_vel_left = v0;
neato_curr_vel_right = v0;

goal = [0, 2.5];

determine_point_runs = 0;
gradient_descent_mode = 1;
calculated_gradient_descent = 0;
focus_point = [];
vl = [];
vr = [];
d = 0.24;

v_index = 1;

start = 0;

disp("start")

EPSILON = 0.01;

within_threshold = @(reference,lowest,biggest) ( ...
        (lowest< reference & reference < biggest) ...
);

counter = 0;

while ((abs(neato_pos_y(end) - goal(2)) > EPSILON))
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
    
    [lidar_x, lidar_y] = polar_to_cartesian(sensors.ranges, sensors.thetasInRadians);
    plot(-lidar_y, lidar_x, ".", DisplayName="Lidar Points"); hold on
        plot(goal - [neato_pos_x(end), neato_pos_y(end)], ".", Color="blue", MarkerSize=20, DisplayName="Goal")
        plot(0, 0, ".", MarkerSize=20, Color="green", DisplayName="Neato")
        legend()
        title("Bird's Eye View of Neato")
        xlabel("X Position (m)"); ylabel("Y Position (m)")
    hold off
    % change tactics if neato is within a distance to a dot (obstacle).
    %pause(.2);
    
    coordinate_x = -1 * lidar_y;
    coordinate_y = lidar_x;
    
    left_most_threshold = -0.4;
    right_most_threshold = 0.4;
    
    upper_limit_threshold = 0.7;
    lower_limit_threshold = 0.2;
    
    if (determine_point_runs == 0 && calculated_gradient_descent == 0 && gradient_descent_mode == 1)
        optimizing_params = params(0.5, 0.9, 500, 1e-4);
        guess_pos = run_gradient_descent(@(curr_neato_pos, end_goal) costfunc(curr_neato_pos, end_goal), [neato_pos_x(end), neato_pos_y(end)], goal, optimizing_params);
        
        new_pos_x = [neato_pos_x(2:end); guess_pos(1)];
        new_pos_y = [neato_pos_y(2:end); guess_pos(2)];

        plot(-lidar_y, lidar_x, ".", DisplayName="Lidar Points"); hold on
            plot(goal - [neato_pos_x(end), neato_pos_y(end)], ".", Color="blue", MarkerSize=20, DisplayName="Goal")
            plot(0, 0, ".", MarkerSize=20, Color="green", DisplayName="Neato")
            plot(-new_pos_y, new_pos_x, Color="k", DisplayName="Neato Path");
            legend()
            title("Bird's Eye View of Neato")
            xlabel("X Position (m)"); ylabel("Y Position (m)")
        hold off

        velocity_x = differential(new_pos_x, time);
        velocity_y = differential(new_pos_y, time);
    
        velocity = (velocity_x.^2 + velocity_y.^2).^0.5;
        speed = norm(velocity);

        tangent_vec = velocity ./ speed;
        tangent_vec = tangent_vec - (pi / 2);

        tangent_vec_x = cos(tangent_vec);
        tangent_vec_y = sin(tangent_vec);
    
        q_vec_x = diff(tangent_vec_x);
        q_vec_y = diff(tangent_vec_y);

        ang_vel = (tangent_vec_x(1:end-1) .* q_vec_y) - (tangent_vec_y(1:end-1) .* q_vec_x);
        
        vl = (speed - ((d/2)*ang_vel)) / 30;
        vr = (speed + ((d/2)*ang_vel)) / 30;

        calculated_gradient_descent = 1;

        disp("calculated gradient descent")

    elseif (determine_point_runs == 0 && gradient_descent_mode == 1 && counter == 0)
        if ((v_index <= length(vl) || v_index <= length(vr)) && abs(rotation_vector(end) - rotation_vector(1)) >= 0.5 && start ~= 0)
            neato_curr_vel_left = -0.1;
            neato_curr_vel_right = 0.1;
        elseif (v_index <= length(vl) || v_index <= length(vr))
            neato_curr_vel_left = vl(v_index);
            neato_curr_vel_right = vr(v_index);
            v_index = v_index + 1;
        end
        disp("updated velocity")
        counter = 10;

    elseif (determine_point_runs == 0 && gradient_descent_mode == 1)
        counter = counter - 1;
        disp("counting down")
    end
    
    if (determine_point_runs == 0)
        disp("checking for object")
        %create separate plot that identifies left and right obstacle coordinates
        [lidar_x, lidar_y] = polar_to_cartesian(sensors.ranges, sensors.thetasInRadians);

        coordinate_x = -1*lidar_y;
        coordinate_y = lidar_x;
       
        %Bounding box around neato (range that triggers obstacles)
        left_most_threshold = -0.4;
        right_most_threshold = 0.4;
        
        upper_limit_threshold = 0.7;
        lower_limit_threshold = 0.2;
  
        for i = 1:360
               % change tactics if neato is within a distance to a dot (obstacle).

            
            if ( within_threshold(coordinate_x(i),left_most_threshold,right_most_threshold ) & ...
                 within_threshold(coordinate_y(i), lower_limit_threshold,upper_limit_threshold))
                                    
                focus_point = [coordinate_x(i) coordinate_y(i)] ; 
                neato_curr_vel_left = 0;
                neato_curr_vel_right = 0;
                break
    
            end

        end
 
    end
   
    next_point_x_threshold = 0.02;
    next_point_y_threshold = 0.03;

    if(~isempty(focus_point))
        disp("found a point")
        start = 1;
        calculated_gradient_descent = 0;
        gradient_descent_mode = 0;
        if(determine_point_runs == 0)

            furthest_right_point = focus_point
            furthest_left_point = focus_point

            disp("Beginning points")
            
            
            while ~isempty( ...
                    find( ...
                        within_threshold(coordinate_x, furthest_right_point(1), furthest_right_point(1) + next_point_x_threshold) & ...
                        within_threshold(coordinate_y, furthest_right_point(2) - next_point_y_threshold, furthest_right_point(2) + next_point_y_threshold) ...
                    ) ...
                )
                
                neato_curr_vel_left = 0;
                neato_curr_vel_right = 0;
                
                closest_right_index = find( ...
                    within_threshold(coordinate_x, furthest_right_point(1), furthest_right_point(1) + next_point_x_threshold) & ...
                    within_threshold(coordinate_y, furthest_right_point(2) - next_point_y_threshold, furthest_right_point(2) + next_point_y_threshold) ...
                )
            
                furthest_right_point = [coordinate_x(closest_right_index),coordinate_y(closest_right_index)];
                
                disp("finish furthest right")
            
            
            end
                
            while ~ isempty(find( ...
                    within_threshold(coordinate_x, furthest_left_point(1) - next_point_x_threshold, furthest_left_point(1)) & ...
                    within_threshold(coordinate_y, furthest_left_point(2) - next_point_y_threshold, furthest_left_point(2) + next_point_y_threshold) ...
                ))

                neato_curr_vel_left = 0;
                neato_curr_vel_right = 0;

                furthest_left_index = find( ...
                    within_threshold(coordinate_x, furthest_left_point(1) - next_point_x_threshold, furthest_left_point(1)) & ...
                    within_threshold(coordinate_y, furthest_left_point(2) - next_point_y_threshold, furthest_left_point(2) + next_point_y_threshold) ...
                )
    
                furthest_left_point = [coordinate_x(furthest_left_index),coordinate_y(furthest_left_index)];
            
            end
            
            %determine which point is closest to (0,0)
    
                distance_to_furthest_left = sqrt( (furthest_left_point(1))^2 + (furthest_left_point(2))^2 );
                distance_to_furthest_right = sqrt( (furthest_right_point(1))^2 + (furthest_right_point(2))^2 );
    
                determine_point_runs = 1;
        end

        %Neato Rotation vector starts at around 1.57 radians ~90 degrees
       

        buffer_distance = 0.15;
        %go slightly further than obstacle detection before stopping
        if(distance_to_furthest_right<=distance_to_furthest_left)
            %turn right

            if (rotation_vector(end) > 0.785) %pi/4
                 disp("turn right");
                 neato_curr_vel_right = -0.1;
                 neato_curr_vel_left = 0.1;
                 
                disp("turn right")
                
            else
                %when it stops turning, go forward.
                    neato_curr_vel_right = 0.1;
                    neato_curr_vel_left = 0.1;
                if neato_pos_x(end) > furthest_right_point(1)+buffer_distance
                    
                    determine_point_runs = 0;
                    gradient_descent_mode = 1;
                    
                    disp("now in gradient descent mode")
                end
                 
            end

            
           %right w neg, left wheel pos
           %rotation vector is 90 degrees
        else
            %turn left
            if (rotation_vector(end) < 2.356) % 3*pi/4
                 disp("turn left");
                 
                 neato_curr_vel_right = 0.1;
                 neato_curr_vel_left = -0.1;
                 
                
            else
                %when it stops turning, go forward.
                    neato_curr_vel_right = 0.1;
                    neato_curr_vel_left = 0.1;
                if neato_pos_x(end) < furthest_left_point(1)-buffer_distance
                    
                    determine_point_runs = 0;
                    gradient_descent_mode = 1;

                    disp("now in gradient descent mode")
                end
                 
            end
            
        end

    end


    
end

sensors = neatov3.receive();
disp("disconnected");
neatov3.disconnect();