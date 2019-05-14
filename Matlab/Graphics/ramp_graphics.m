% Script to generate a graphic displaying a video of the drone going over
% the ramp compared 
% synced with the data collected while hovering

clear
% Get the data collected
drone_ramp_graphics_data;

% generate the model data

model_1_path_y = NaN(1, 34);
model_2_path_y = NaN(1, 34);

% model 1
mass_1 = 1; % 1kg
model_1_path_y(1) = set_y(1);
v_y_1 = 0;
a_y_1 = 0;
I_1 = 0;
error_1 = 0;
last_error_1 = 0;
kp_1 = 5;
ki_1 = 5;
kd_1 = 5;

% model 2
mass_2 = 1; % 1kg
model_2_path_y(1) = set_y(1);
v_y_2 = 0;
a_y_2 = 0;
I_2 = 0;
error_2 = 0;
last_error_2 = 0;
kp_2 = kp_1;
ki_2 = ki_1;
kd_2 = kd_1;
max_a_y_2 = 4 * 9.81 * 100; %g
min_a_y_2 = -0.4 * 9.81 * 100; %g

% simulate PID models

for i = 2:length(path_t)
    x = path_x(i);
    % update dt
    dt = path_t(i) - path_t(i-1);
    % update y
    model_1_path_y(i) = model_1_path_y(i - 1) + dt * v_y_1 + 0.5 * dt^2 * a_y_1;
    model_2_path_y(i) = model_2_path_y(i - 1) + dt * v_y_2 + 0.5 * dt^2 * a_y_2;
    % update v 
    v_y_1 = v_y_1 + a_y_1 * dt;
    v_y_2 = v_y_2 + a_y_2 * dt;
    % update last_error
    last_error_1 = error_1;
    last_error_2 = error_2;
    % update error
    error_1 = setpoint - model_sensor(x, model_1_path_y(i), ramp_x, ramp_y);
    error_2 = setpoint - model_sensor(x, model_2_path_y(i), ramp_x, ramp_y);
    % update P
    P_1 = error_1 * kp_1;
    P_2 = error_2 * kp_2;
    % update I
    I_1 = I_1 + error_1 * dt * ki_1;
    I_2 = I_2 + error_2 * dt * ki_2;
    % update D
    D_1 = ((error_1 - last_error_1) / dt) * kd_1;
    D_2 = ((error_2 - last_error_2) / dt) * kd_2;
    % update a
    a_y_1 = (P_1 + I_1 + D_1) / mass_1;
    % for model 2, bound a to its min and max
    a_y_2 = max(min_a_y_2, min(((P_2 + I_2 + D_2) / mass_2), max_a_y_2));
end

figure('Name', 'Ramp, prototype and model');
set(gcf, 'Position',  [25, 25, 700, 600])

% Start playing the video and animating the graphs

subplot(2,1,2);
title('Prototype and Model Data')
xlabel('x (cm)');
ylabel('y (cm)');
set(gca, 'XLim', [0 420], 'YLim', [0 150]);
grid on;
hold on;
%plot(ramp_x, set_y, 'Color', 'g')
plot(ramp_x, ramp_y, 'Color', 'r', 'LineWidth', 1)
plot(path_x, path_y, 'Color', 'b', 'LineWidth', 1)
plot(path_x, model_2_path_y, 'Color', 'g', 'LineWidth', 1)
plot(path_x, model_1_path_y, 'Color', 'm', 'LineWidth', 1)

%legend({'Ramp','Ideal Path', 'Prototype Path', 'Linear Model Path', 'Non-linear Model Path'},'Location','southwest')

vid_plot = subplot(2,1,1);
% top1 = plot(video_ramp_x, video_set_y, 'Color', 'g');
% top2 = plot(video_ramp_x, video_ramp_y, 'Color', 'r');
% top3 = plot(video_path_x, video_path_y, 'Color', 'b');

v = VideoReader('goodfootage_OK_ramp_pass_2.mp4');
index = 1;

while (hasFrame(v) & ceil(index/5) <= 34)
    
    subplot(2,1,2);
    prototype_head = scatter(path_x(ceil(index/5)), path_y(ceil(index/5)), 'filled','MarkerFaceColor', 'b');
    model_2_head = scatter(path_x(ceil(index/5)) , model_2_path_y(ceil(index/5)),'filled' , 'MarkerFaceColor', 'g');
    model_1_head = scatter(path_x(ceil(index/5)) , model_1_path_y(ceil(index/5)), 'filled','MarkerFaceColor', 'm');
    
    drawnow();
    
    %display the next frame
    hold off
    vidFrame = readFrame(v);
%     video_seconds = v.CurrentTime;
    image(vidFrame(200:890, 1:1920, 1:3), 'Parent', vid_plot); %crop
    vid_plot.Visible = 'off';
    hold on
    
    
%     subplot(2,1,2);
%     prototype_head = scatter(path_x(ceil(index/5)), path_y(ceil(index/5)), 'filled', 'MarkerFaceColor', 'b');
%     model_1_head = scatter(path_x(ceil(index/5)) , model_1_path_y(ceil(index/5)), 'filled', 'MarkerFaceColor', 'y');
%     model_2_head = scatter(path_x(ceil(index/5)) , model_2_path_y(ceil(index/5)), 'filled', 'MarkerFaceColor', 'c');
%     drawnow();
%     pause(0.1);
    
    F_out(index) = getframe(gcf);
    
    delete(prototype_head);
    delete(model_1_head);
    delete(model_2_head);
    index = index + 1;
        
end

% Get ready to start capturing the fnished video
v_out = VideoWriter('Ramp, prototype and models', 'MPEG-4');
v_out.FrameRate = 30;
open(v_out)
writeVideo(v_out, F_out);
close(v_out)

function alt = model_sensor(x, y, ramp_x, ramp_y)
if (x >= ramp_x(2))
    alt = y - ramp_y(1);
    return
end

if (x <= ramp_x(3))
    alt = y - ramp_y(5);
    return
end

alt = y + (ramp_x(2) - x) * (ramp_y(3) - ramp_y(2))/(ramp_x(3) - ramp_x(2)) - ramp_y(1);

end
