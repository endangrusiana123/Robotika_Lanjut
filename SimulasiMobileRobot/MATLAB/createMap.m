close all
% Close any existing connections
sim = remApi('remoteApi');
sim.simxFinish(-1);

% Connect to CoppeliaSim
clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if clientID > -1
    disp('Connected to CoppeliaSim');    
else
    disp('Failed to connect to CoppeliaSim');
    sim.delete();
    return;
end

% Get vision sensor handle
[returnCode, visionSensorHandle] = sim.simxGetObjectHandle(clientID, 'visionSensor', sim.simx_opmode_oneshot_wait);

% Retrieve vision sensor image
[returnCode, resolution, rawImage] = sim.simxGetVisionSensorImage2(clientID, visionSensorHandle, 0, sim.simx_opmode_oneshot_wait);

if returnCode == sim.simx_return_ok
    % Convert raw image to MATLAB format
    imageMatrix = permute(rawImage, [2, 1, 3]); % Adjust dimension order
    imageMatrix = flip(imageMatrix, 2); % Flip horizontally
    
    % Display the original image
    figure;
    imshow(imageMatrix);
    title('Original Vision Sensor Image');
    
    % Separate green and orange areas
    % Define thresholds for green and orange in the RGB color space
    greenMask = imageMatrix(:, :, 1) < 100 & imageMatrix(:, :, 2) > 150 & imageMatrix(:, :, 3) < 100; % Adjust RGB ranges for green
    orangeMask = imageMatrix(:, :, 1) > 200 & imageMatrix(:, :, 2) > 100 & imageMatrix(:, :, 2) < 200 & imageMatrix(:, :, 3) < 100; % Adjust RGB ranges for orange
    
    % Combine masks
    binaryMap = greenMask | orangeMask;
    
    % Display the binary map (Green & Orange: White, Others: Black)
    figure;
    imshow(binaryMap);
    title('Processed Binary Map (Green & Orange Areas: White, Others: Black)');
    
    % Save the binary map if needed
    imwrite(binaryMap, 'binaryMap.png');
    disp('Binary map saved as binaryMap.png');
else
    disp('Failed to retrieve vision sensor image');
end

% Close the connection 
sim.simxFinish(clientID);
sim.delete();
