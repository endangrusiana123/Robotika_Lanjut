close all
% Load and process map
im_map = imread('binaryMap_230x230-2.jpg');
im_bin = im2bw(im_map);
myMaplogical = not(logical(im_bin));
mapRealSize = 23; %dihitung berdasarkan vision sensor
mapSize = 230; %dihitung berdasarkna vision sensor
mapFactor = mapSize/mapRealSize; 
disp('map factor:');
disp(mapFactor);
map = binaryOccupancyMap(myMaplogical, mapFactor);
topLeftMapX = -7.825; %dihitung dari cuboid topleft
topLeftMapY =  3.700; %dihitung dari cuboid topleft
%map.XWorldLimits
%map.YWorldLimits
map.GridOriginInLocal = [diff(map.XWorldLimits)-mapRealSize -diff(map.YWorldLimits)];
map.LocalOriginInWorld = [topLeftMapX, topLeftMapY];

screenSize = get(0, 'ScreenSize'); % Mendapatkan ukuran layar
width = 400; % Lebar figure
height = 320; % Tinggi figure
x = (screenSize(3) - width)*4 / 4; % Posisi horizontal
y = ((screenSize(4) - height)*0 / 2)+28; % Posisi vertikal

figure('Position', [x y width height]);
show(map);

% Inflate map to add clearance (in meters)
clearance = 0.5; % Desired clearance in meters
inflate(map, clearance);

% Display the inflated map
figure('Position', [x y width height]);
show(map);
title('Inflated Map with Clearance');

% Create PRM
possibilityPoint = 400;
prmSimple = mobileRobotPRM(map, possibilityPoint);
figure('Position', [x y width height]);
show(prmSimple);

% % Define map and path parameters
initAngle = 180; %dihitung dari posisi robot
calcXStart = 12.855; %dihitung dari posisi robot
calcYStart = -0.200; %dihitung dari posisi robot
calcXGoal = -1.650; %dihitung dari posisi cuboid titik akhir
calcYGoal = -7.950; %dihitung dari posisi cuboid titik akhir

startPosition = [calcXStart calcYStart];
goalPosition = [calcXGoal calcYGoal];

disp('Start Position:');
disp(startPosition);

disp('Goal Position:');
disp(goalPosition);

path = findpath(prmSimple, startPosition, goalPosition);
show(prmSimple);
pause(5); % wait

% Set Pure Pursuit controller
pp = controllerPurePursuit('DesiredLinearVelocity', 4, 'MaxAngularVelocity', 5);
pp.Waypoints = path;
% Menampilkan semua waypoint yang ada
disp('Waypoints:');
disp(pp.Waypoints);

% Initial position and robot parameters
currentPosition = path(1, :); % start position
currentOrientation = deg2rad(initAngle); % 90 degrees
% Display current position and orientation
disp('Current Position:');
disp(['X: ', num2str(currentPosition(1)), ', Y: ', num2str(currentPosition(2))]);

disp('Current Orientation:');
disp(['Orientation (in radians): ', num2str(currentOrientation)]);
disp(['Orientation (in degrees): ', num2str(rad2deg(currentOrientation))]);
currentPose = [currentPosition currentOrientation];
[v,w,npos] = pp(currentPose);
theta = atan2((npos(2)-currentPose(2)),(npos(1)-currentPose(1)));
dtheta=theta-currentPose(3);
pos = [0 0 0]; %x,y,A
R = 0.195/2;
L = 0.415/2;
a = [ v*cos(dtheta); -w];
b = [  R/2    R/2;
     R/2*L -R/2*L];
b_inv = inv(b);
vout = b_inv*a*R;

% Braitenberg parameters
point = [-1 -1 -1 -1 -1 -1 -1 -1];
purePersuitThreshold = 0.3;
noDetectionDist = 0.30;
maxDetectionDist = 0.10;
detect = [0 0 0 0 0 0 0 0];
braitenbergL = [-0.1 -0.4 -0.6 -0.8 -2.0 -2.2 -2.4 -2.6];
braitenbergR = [-2.6 -2.4 -2.2 -2.0 -0.8 -0.6 -0.4 -0.1];
v0 = 4;

% Start remote API
sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if (clientID > -1)
    disp('connected');
    
    % Get all object handles
    [ret, left_motor] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);
    [ret, right_motor] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking);
    for i = 1:1:8
        [ret, sensor(i)] = sim.simxGetObjectHandle(clientID, sprintf('Pioneer_p3dx_ultrasonicSensor%d', i), sim.simx_opmode_blocking);
    end
    [ret, p3dx_position] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking);
    [ret, p3dx_orientation] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking);

    % Setup streaming for sensors and position
    for i = 1:1:8
        [ret, detect(i), distance(i, :), ~, ~] = sim.simxReadProximitySensor(clientID, sensor(i), sim.simx_opmode_streaming);
    end
    [ret, position] = sim.simxGetObjectPosition(clientID, p3dx_position, -1, sim.simx_opmode_streaming);
    [ret, orientation] = sim.simxGetObjectOrientation(clientID, p3dx_orientation, -1, sim.simx_opmode_streaming);

    % Main loop
    while 1
        %% Read sensor 1-8
        for i = 1:1:8
            [ret, detect(i), distance(i, :), ~, ~] = sim.simxReadProximitySensor(clientID, sensor(i), sim.simx_opmode_buffer);
            if (detect(i) == 0)
                point(i) = 1;
            else
                point(i) = norm(distance(i, :));
            end
        end

        % Display sensor readings
        disp('Ultrasonic Sensor Readings:');
        disp(point);

        %% Pure Pursuit Control
        if (all(point > purePersuitThreshold))
            [v, w, npos] = pp(currentPose);
            theta = atan2((npos(2) - currentPose(2)), (npos(1) - currentPose(1)));
            dtheta = theta - currentPose(3);
            a = [v * cos(dtheta); -w];
            vout = b_inv * a * R;
            [ret] = sim.simxSetJointTargetVelocity(clientID, left_motor, vout(1, :), sim.simx_opmode_blocking);
            [ret] = sim.simxSetJointTargetVelocity(clientID, right_motor, vout(2, :), sim.simx_opmode_blocking);

            % Display motor velocities for Pure Pursuit
            disp('Pure Pursuit Motor Velocities:');
            disp(['Left Motor: ', num2str(vout(1, :)), ', Right Motor: ', num2str(vout(2, :))]);

            % Update robot position and orientation
            [ret, position] = sim.simxGetObjectPosition(clientID, p3dx_position, -1, sim.simx_opmode_buffer);
            [ret, orientation] = sim.simxGetObjectOrientation(clientID, p3dx_orientation, -1, sim.simx_opmode_buffer);
            pos = [position(1) position(2) orientation(3)];
            fprintf('X: %.2f, Y: %.2f, Yaw: %.2f (radians)\n', position(1), position(2), orientation(3));
            currentPose = double(pos);

            % Calculate current waypoint index
            distances = vecnorm((pp.Waypoints - currentPose(1:2))');
            [~, waypointIndex] = min(distances);
            waypointCoords = pp.Waypoints(waypointIndex, :);

            % Display information about the current waypoint
            disp(['Current Waypoint: ', num2str(waypointIndex)]);
            disp(['Waypoint Coordinates: [', num2str(waypointCoords(1)), ', ', num2str(waypointCoords(2)), ']']);
        
        %% Braitenberg (Obstacle Avoidance)
        else
            for i = 1:8
                dist = point(i);
                if (dist < noDetectionDist)
                    if (dist < maxDetectionDist)
                        dist = maxDetectionDist;
                    end
                    detect(i) = 1 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist));
                else
                    detect(i) = 0;
                end
            end
            vLeft = v0;
            vRight = v0;
            for i = 1:8
                vLeft = vLeft + braitenbergL(i) * detect(i);
                vRight = vRight + braitenbergR(i) * detect(i);
            end

            % Send to motor
            [ret] = sim.simxSetJointTargetVelocity(clientID, left_motor, vLeft, sim.simx_opmode_blocking);
            [ret] = sim.simxSetJointTargetVelocity(clientID, right_motor, vRight, sim.simx_opmode_blocking);

            % Display motor velocities for Braitenberg
            disp('Braitenberg Motor Velocities:');
            disp(['Left Motor: ', num2str(vLeft), ', Right Motor: ', num2str(vRight)]);
        end

        %% Calculate distance to goal
        distToGoal = sqrt(sum((currentPose(1:2) - goalPosition) .^ 2));
        disp(['Distance to Goal: ', num2str(distToGoal)]);

        % Stop if goal is reached
        if (distToGoal < 0.3)
            [ret] = sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_blocking);
            [ret] = sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_blocking);
            [ret] = sim.simxAddStatusbarMessage(clientID, '=====FINISH=====', sim.simx_opmode_blocking);
            break;
        end
        %break; %?????????????????????????????????????????
    end
    
    % Close the connection to CoppeliaSim
    sim.simxFinish(clientID);
end
sim.delete();
