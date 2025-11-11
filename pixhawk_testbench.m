% Capture Pixhawk Telemetry (Mavlink) data in Matlab
% Copyright (c) 2025, Muhammet Emin Yanik

comportSnum = 9;

% Open the serial port
comportnum_str = ['COM' num2str(comportSnum)];
baudrate = 921600;
timeout = 0.5; % Timeout for serial read (in seconds)
sphandle = serialport(comportnum_str,baudrate,"Timeout",timeout);
configureTerminator(sphandle,0);

% Create the mavlink related parameters
ATTITUDE_QUATERNION = 31;
dialect = mavlinkdialect("common.xml");

% Open plotter
fig = figure('Position',[50 50 900 900]);
posePlotter = poseplot(gca,'ENU');
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)')
pause(1)

% If you want to initialize the yaw angle to zero
yawCompensationFlag = true;
if yawCompensationFlag == false
    yawCompensationQuat = [1 0 0 0]; % Identity quaternion initially
end

% Start the loop
flush(sphandle);
while ishandle(fig)
    % Capture the data from the serial port
    if (sphandle.NumBytesAvailable>0)
        rxData = uint8(read(sphandle,sphandle.NumBytesAvailable,"uint8"));
    else
        continue;
    end

    % Parse the mavlink buffer
    [msg,~] = deserializemsg(dialect, rxData);

    % Parse the attitude
    quatMsgs = find([msg.MsgID]==ATTITUDE_QUATERNION);
    if ~isempty(quatMsgs)
        lastQuatMsgPayload = msg(quatMsgs(end)).Payload;

        % Convert the quaternion in NED to ENU - [qw, qx, qy, qz]
        quat = [lastQuatMsgPayload.q1, lastQuatMsgPayload.q3, lastQuatMsgPayload.q2, -lastQuatMsgPayload.q4];

        % Create inverse yaw quaternion (negative angle)
        if yawCompensationFlag == true
            % First measurement - extract the yaw to compensate
            yaw = extractYawFromQuaternion(quat);
            yawCompensationQuat = yawToQuaternion(-yaw);
            yawCompensationFlag = false;
        end

        % Compensate yaw
        rotators = quaternion(quaternionMultiply(yawCompensationQuat, quat));
        
        % Update the plotter
        set(posePlotter,Orientation=rotators,Position=[0 0 0]);
    end

    % Wait until the next frame (100Hz update rate, for the worst case scenario)
    pause(0.01);
end

% Delete the port
delete(sphandle);


% Utility functions for yaw compensation
% Function to extract yaw from quaternion in ENU coordinates
function yaw = extractYawFromQuaternion(q)
    % Convert quaternion to rotation matrix
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    
    % Calculate yaw (rotation around z-axis)
    yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
end

% Function to create quaternion for pure yaw rotation
function q = yawToQuaternion(yaw)
    q = [cos(yaw/2), 0, 0, sin(yaw/2)];
end

% Quaternion multiplication function
function result = quaternionMultiply(q1, q2)
    % q1, q2 are quaternions in format [qw, qx, qy, qz]
    w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
    w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);
    
    result = [
        w1*w2 - x1*x2 - y1*y2 - z1*z2;
        w1*x2 + x1*w2 + y1*z2 - z1*y2;
        w1*y2 - x1*z2 + y1*w2 + z1*x2;
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ].';
end