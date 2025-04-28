% Spins the Neato to spin up its LiDAR, then run its LiDAR.

ip = "192.168.16.84";
neatov3.connect(ip);
left_wheel = [];
right_wheel = [];
sensors = 0;
for i = 1:25
    neatov3.setVelocities(0, 0);
    sensors = neatov3.receive();
    left_wheel(end+1) = sensors.encoders(1);
    right_wheel(end+1) = sensors.encoders(2);
end
sensors = neatov3.receive();
% sensors.ranges
%polarplot(sensors.thetasInRadians, sensors.ranges, "o", Color=[1, 0, 0]);
[x, y] = polar_to_cartesian(sensors.ranges, sensors.thetasInRadians);
plot(x, y, "."); hold on; plot(0, 0, ".", MarkerSize=5)
title("Neato LiDAR 360 degree Plot")
sensors = neatov3.receive();
disp("disconnected");
neatov3.disconnect();