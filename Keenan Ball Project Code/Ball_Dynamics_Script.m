%% Test of chord length
R = 0.11; %radius of ball
r = (1/2) * R; %radius of controller range of motion
theta_s = 0; %starting position of the ball
 

theta_b = [0:.1:4*pi];  %one revolution of circle (ball)

l_a = sqrt(((R*cos(theta_b + theta_s) - r).^2) + (R*sin(theta_b + theta_s)).^2); %caluclate length of chord connecting anchor and controller

theta_b_deg = theta_b * (360 / (2*pi)); %convert to degrees 
plot(theta_b_deg, l_a, 'LineWidth', 1.5);
title('Length of Chord Over 2 Ball Revolutions')
xlabel('Ball Position (angle from 0)')
ylabel('Chord Length [m]')

%% Finding required speed of motors
omega = 4; %angular velocity of ball
t = linspace(0, ((2*pi)/omega), 200); %calculates time for 1 rotation and creates array 
l_dot = (1/2) .* (R^2 + r^2 - 2*R*r.*cos(omega.*t)).^(-1/2) .* (2*R*r*omega .* sin(omega.*t)); %analytical expression for dl/dt
max_l_speed = max(abs(l_dot)); %find maximum speed from l_dot

figure
plot(t, l_dot, 'LineWidth', 1.5)
grid on 
title(sprintf('dl/dt for \\omega = %d. Max = %.4g m/s', omega, max_l_speed))
xlabel('Time [s]')
ylabel('dl/dt')

% Plot max rpm of motor vs omega
    omega = [0:.1:10];
    max_l_speed_arr = [];
    for i = 1:length(omega)
        t = linspace(0, ((2*pi)/omega(i)), 200);
        l_dot = (1/2) .* (R^2 + r^2 - 2*R*r.*cos(omega(i).*t)).^(-1/2) .* (2*R*r*omega(i) .* sin(omega(i).*t));
        max_l_speed_arr = [max_l_speed_arr max(abs(l_dot))]; %append new max speed to max speed array 
    end
    
    %Convert linear speed array to rpm (assume that radius of spool is 1.5cm diameter)
    max_rpm = max_l_speed_arr * 60 ./ (pi * 0.015);
    
    figure()
    plot(max_rpm, omega, 'LineWidth', 1.5)
    grid on 
    title('\omega_{ball} vs Max RPM of Motor')
    ylabel('\omega_{ball} [rad/sec]')
    xlabel('Max Required RPM')

%     % Confirm analytical expression with numerical differentiation 
%     l_a_c = sqrt(((R*cos(omega*t) - r).^2) + (R*sin(omega*t)).^2);
%     l_dot_num = diff(l_a_c / (t(2) - t(1)));
%     plot(t(1:length(t)-1), l_dot_num)
%     figure()
%     plot(t, l_dot)
%     hold on
%     plot(t(1:length(t)-1), l_dot_num)

%% Find maximum acceleration -> Max torque required

omega = 4; %angular velocity of ball
t = linspace(0, ((2*pi)/omega), 200); %calculates time for 1 rotation and creates array 
l_dot = (1/2) .* (R^2 + r^2 - 2*R*r.*cos(omega.*t)).^(-1/2) .* (2*R*r*omega .* sin(omega.*t)); %analytical expression for dl/dt

% numerically differentiate 
l_acc = diff(l_dot) / (t(2) - t(1));
[max_acc, i_max_acc] = max(-l_acc);
max_acc_theta = t(i_max_acc) * omega * 360 / (2*pi) %finds position where max acceleration occurs in degrees

%Plot acceleration vs time
plot(t(1:length(t)-1), l_acc, 'LineWidth', 1.5)
title(sprintf('d^2l/dt^2 vs. Time. Max Negative Acceleration = %.3g. Position at max = %.3g degrees', max_acc, max_acc_theta))
xlabel('Time')
ylabel('Acceleration')

%Plot acceleration vs theta
plot(t(1:length(t)-1) * omega * 360 / (2*pi), l_acc)
title('Linear Acceleration vs Theta')


%% Create lookup table for length functions 
theta = linspace(0, 2*pi, 50);
    L = sqrt(((R*cos(theta) - r).^2) + (R*sin(theta)).^2)
    dL_dt = (1/2) .* (R^2 + r^2 - 2*R*r.*cos(theta)).^(-1/2) .* 2*R*r.*(sin(theta)) %this value must be multiplied by *omega
    acc = l_acc([1:4:200])

csvwrite('L_vals', L);
csvwrite('dL_dt', dL_dt);
csvwrite('acc', acc)