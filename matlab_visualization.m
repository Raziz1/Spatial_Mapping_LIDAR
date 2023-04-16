%{  
* File: matlab.m
* Author: Rahim Aziz
* Student Number: 400383784
* Date: 2023 - 04 - 08
* Course:  COMP ENG 2DX3 Final Deliverable
* 
* Summary of the file: Communicates with the MSP-EXP432E401Y
* microcontroller via UART. When the appropriate data is read and parsed it
* will be converted from polar to cartesian coordiantes. Once they are
* converted MATLAB will plot them on a 3D scatter plot. Then MATLAB will
* proceed to attach the adjacent points in the same depth level via a line.
* Lastly, MATLAB will attached the corresponding points in each depth level
* to the adjacent depth to create a 3D spacial reconstruction.
%}

% ---------------------------------------- Main Function ----------------------------------------%
clear;
ports = serialportlist("available"); % List serial ports available
serial_port = serialport(ports(2), 115200, 'Timeout', 120); % Connect to the second serial port which is COM5 in my case
flush(serial_port);

% Define parameters
depth_total = 8; % x-axis
num_measurements = 128; % This means a measurment is taken every 5.625 deg
measurements = zeros(0,3);

current_depth = 0; % Variable used to keep track of the current depth ( 0 to 10 * 128)

while (current_depth <= depth_total * num_measurements-1) % Read until all data recieved
    % Read data from serial port
    serial_data = readline(serial_port);
    
    % If the incoming serial string is empty break the loop
    if (isempty(serial_data))
        break;
    end

    % Prase the incoming data and store it in a vector matrix
    temporary_data = parse(serial_data);

    % Return the columns of the data containing relevant data
    temporary_matrix = temporary_data([2 3 4]);
    
    disp(temporary_matrix);

    if (temporary_matrix(1) ~= 0)
        % TOF received incorrect measurement
    else
        % Push the incoming measurment into a final matrix to store all the
        % measurement data
        measurements = vertcat(measurements, temporary_matrix); %#ok<AGROW> 
    end

    current_depth=current_depth+1;
end

% Close the serial port object
%fclose(s);
%delete(s);

measurement_data = measurements.'; % Transpose the data into a new N * M matrix

% Convert polar to Cartesian
[x, y, z] = pol2cart(measurement_data(2,:).*(pi/180), measurement_data(1,:), measurement_data(3,:));
cartesian_data = [z; y; x]; % Flipped X and Z

% Create Plotsk
figure;
% 3D scatter plot
scatter3(cartesian_data(1,:), cartesian_data(2,:), cartesian_data(3,:), 'filled');
hold on;

% Connect the points with the same angle and depth difference of 1
for d = 1:depth_total
    
    % Connect adjacent points in each ring
    offset = (d-1)*num_measurements;
    for current_depth = 1:num_measurements-1
        plot3(cartesian_data(1,offset+current_depth:offset+current_depth+1), cartesian_data(2,offset+current_depth:offset+current_depth+1), cartesian_data(3,offset+current_depth:offset+current_depth+1), 'k-');
    end
    % Last point
    plot3([cartesian_data(1,offset+1), cartesian_data(1,offset+num_measurements)], [cartesian_data(2,offset+1), cartesian_data(2,offset+num_measurements)], [cartesian_data(3,offset+1), cartesian_data(3,offset+num_measurements)], 'k-');
end

for d = 1:depth_total-1
    for current_depth = 1:num_measurements
        % Connect points in adjacent rings
        p1 = (d-1)*num_measurements + current_depth;
        p2 = d*num_measurements - current_depth + num_measurements+1;
        plot3([cartesian_data(1,p1), cartesian_data(1,p2)], [cartesian_data(2,p1), cartesian_data(2,p2)], [cartesian_data(3,p1), cartesian_data(3,p2)], 'k-');
    end
end

% Output 
hold off;
title('2DX3 Project Visualization');
xlabel('X Depth');
ylabel('Y Width');
zlabel('Z Height');
grid on;

% ---------------------------------------- Parsing Data Function ----------------------------------------%
%{
* function parsed_data = parse(n)
*
* Parameters: string n
*
* Return Value: Returns a 1x3 matrix containing the check bit, distance,
* angle, depth, and SPAD number
*
* Description: Parses the incoming serial string by splitting at the comma
* delimiters
* 
%}
function parsed_data = parse(n)
    
    % Read incoming UART string
    incoming_string = n;
    
    % Print out the incoming string
    disp("INCOMING STRING: "+ incoming_string);

    % Parse variables separated by commas
    parsed_variables = sscanf(incoming_string, '%f,%f,%f,%f,%f');
    
    % Store parsed variables in a variable
    check_bit = parsed_variables(1);
    distance = parsed_variables(2);
    angle = parsed_variables(3)*5.625/8; % Steps to deg
    depth = parsed_variables(4);
    spad_num = parsed_variables(5);
    
    % Return the parsed data as a matrix
    parsed_data = [check_bit, distance, angle, depth, spad_num];
end