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
determine_point_runs = 0
focus_point = [];

disp("start")

within_threshold = @(reference,lowest,biggest) ( ...
        (lowest< reference & reference < biggest) ...
        )

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
        plot(0, 0, ".", MarkerSize=20, Color=[0.3, 0.3, 1])
    hold off
    % change tactics if neato is within a distance to a dot (obstacle).
    %pause(.2);
    
    coordinate_x = -1*lidar_y;
    coordinate_y = lidar_x;
    
    left_most_threshold = -0.4;
    right_most_threshold = 0.4;
    
    upper_limit_threshold = 0.7;
    lower_limit_threshold = 0.2;

    
    if (determine_point_runs ==0)
        %create separate plot that identifies left and right obstacle coordinates
        [lidar_x, lidar_y] = polar_to_cartesian(sensors.ranges, sensors.thetasInRadians);
        plot(-lidar_y, lidar_x, "."); hold on
            plot(final_position - [neato_pos_x(end), neato_pos_y(end)], ".", Color="blue", MarkerSize=20)
            plot(0, 0, ".", MarkerSize=20, Color=[0, 0, 0])
        hold off
    coordinate_x = -1*lidar_y;
    coordinate_y = lidar_x;
    
    % differential = @(vector1, vector2) diff(vector1) ./ diff(vector2);
    % 
    % differential(vector1, vector2)


   
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
    
                determine_point_runs = 1
        end

        %Neato Rotation vector starts at around 1.57 radians ~90 degrees
       

        buffer_distance = 0.3
        %go slightly further than obstacle detection before stopping
        if(distance_to_furthest_right<=distance_to_furthest_left)
            %turn right

            if (rotation_vector(end) > 0.785) %pi/4
                 disp("turn right");
                 neato_curr_vel_right = -0.1;
                 neato_curr_vel_left = 0.1;
                 
                disp("turn right")
                
            else
                %when it stops turning, just stop.
                disp("I didn't turn right")
                    neato_curr_vel_right = 0.1;
                    neato_curr_vel_left = 0.1;
                if neato_pos_x(end) > furthest_right_point(1)+buffer_distance
                    
                    break
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
                %when it stops turning, just stop.
                    neato_curr_vel_right = 0.1;
                    neato_curr_vel_left = 0.1;
                if neato_pos_x(end) < furthest_left_point(1)-buffer_distance
                    
                    break;
                end
                 
            end
            
        end


        %break
    end
    
end

sensors = neatov3.receive();
disp("disconnected");
neatov3.disconnect();