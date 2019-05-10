% Script to generate a graphic displaying a video of the drone hovering,
% synced with the data collected while hovering
clear
% Get the data collected
drone_graphics_data;

% Calculate PID data
setpoint = 55; %cm
setpoint_x = [time_elapsed_seconds(1) time_elapsed_seconds(end)];
setpoint_y = [setpoint setpoint];
start_x = [19.4 19.4];
start_y = [-1000 1000];
end_x = [42.25 42.25];
end_y = start_y;

P = setpoint - alt_cm;
I = NaN(length(P),1);
D = NaN(length(P),1);
I(1) = 90;
D(1) = 0;
for i = 2:length(P)
    I(i) = I(i - 1) + P(i) * time_interval_seconds(i);
    D(i) = (P(i) - P(i - 1)) / time_interval_seconds(i);
end

figure('Name', 'Hover, with data graphics');
set(gcf, 'Position',  [25, 25, 1100, 600])

subplot(2,3,1);
plot(time_elapsed_seconds, alt_cm)
title('Reported Altitude')
xlabel('Time (s)');
ylabel('Altitude (cm)');
grid on;
hold on;
plot(setpoint_x, setpoint_y, 'Color', 'b');
plot(start_x, start_y,  '--', 'Color', [0.4940 0.1840 0.5560]);
plot(end_x, end_y, '--', 'Color', [0.4940 0.1840 0.5560]);
alt_line = animatedline('Color', 'r');
start_txt = text(19.4, 90, 'Start', 'Color', [0.4940 0.1840 0.5560], 'clipping','on');
end_txt = text(42.25, 90, 'End', 'Color', [0.4940 0.1840 0.5560], 'clipping','on');
alt_txt = text(24.4, 45, {'Altitude', 'Setpoint = 55'}, 'Color', 'b', 'clipping','on');

subplot(2,3,3);
plot(time_elapsed_seconds, output)
title('Throttle Output')
xlabel('Time (s)');
ylabel('Throttle (%)');
grid on;
hold on;
plot(start_x, start_y, '--', 'Color', [0.4940 0.1840 0.5560]);
plot(end_x, end_y,  '--', 'Color', [0.4940 0.1840 0.5560]);
output_line = animatedline('Color', 'r');
start_txt = text(19.4, 90, 'Start', 'Color', [0.4940 0.1840 0.5560], 'clipping','on');
end_txt = text(42.25, 90, 'End', 'Color', [0.4940 0.1840 0.5560], 'clipping','on');

subplot(2, 3, 4);
plot(time_elapsed_seconds, P)
title('P Term')
xlabel('Time (s)');
ylabel('Weight');
grid on;
hold on;
plot(start_x, start_y, '--', 'Color', [0.4940 0.1840 0.5560]);
plot(end_x, end_y,  '--', 'Color', [0.4940 0.1840 0.5560]);
P_line = animatedline('Color', 'r');
start_txt = text(19.4, 90, 'Start', 'Color', [0.4940 0.1840 0.5560], 'clipping','on');
end_txt = text(42.25, 90, 'End', 'Color', [0.4940 0.1840 0.5560], 'clipping','on');

subplot(2, 3, 5);
plot(time_elapsed_seconds, I)
title('I Term')
xlabel('Time (s)');
ylabel('Weight');
grid on;
hold on;
plot(start_x, start_y, '--', 'Color', [0.4940 0.1840 0.5560]);
plot(end_x, end_y, '--', 'Color', [0.4940 0.1840 0.5560]);
I_line = animatedline('Color', 'r');
start_txt = text(19.4, -10, 'Start', 'Color', [0.4940 0.1840 0.5560], 'clipping','on');
end_txt = text(42.25, -10, 'End', 'Color', [0.4940 0.1840 0.5560], 'clipping','on');

subplot(2, 3, 6);
plot(time_elapsed_seconds, D)
title('D Term')
xlabel('Time (s)');
ylabel('Weight');
grid on;
hold on;
plot(start_x, start_y, '--', 'Color', [0.4940 0.1840 0.5560]);
plot(end_x, end_y, '--', 'Color', [0.4940 0.1840 0.5560]);
D_line = animatedline('Color', 'r');
start_txt = text(19.4, 160, 'Start', 'Color', [0.4940 0.1840 0.5560], 'clipping','on');
end_txt = text(42.25, 160, 'End', 'Color', [0.4940 0.1840 0.5560], 'clipping','on');

% Start playing the video and animating the graphs
vid_plot = subplot(2,3,2);
v = VideoReader('hover_footage_trim.mp4');
start_seconds = 17;
v.CurrentTime = start_seconds;
current_data_frame = 1;
data_frames = length(time_elapsed_seconds);
time_width = 5; %how far back and forward to show

frame_out_index = 0;
frame_in_index = 0;

while hasFrame(v)
    
    frame_in_index = frame_in_index + 1;
    
    %display the next frame
    vidFrame = readFrame(v);
    video_seconds = v.CurrentTime;
    
    %update alt_line axes
    subplot(2,3,1);
    set(gca, 'XLim', [(video_seconds - time_width) (video_seconds + time_width)], 'YLim', [0 100]);
    
    %update output_line axes
    subplot(2,3,3);
    set(gca, 'XLim', [(video_seconds - time_width) (video_seconds + time_width)], 'YLim', [0 100]);
    
    %update P_line axes
    subplot(2,3,4);
    set(gca, 'XLim', [(video_seconds - time_width) (video_seconds + time_width)], 'YLim', [-100 100]);
   
    %update I_line axes
    subplot(2,3,5);
    set(gca, 'XLim', [(video_seconds - time_width) (video_seconds + time_width)], 'YLim', [-100 0]);
    
    %update D_line axes
    subplot(2,3,6);
    set(gca, 'XLim', [(video_seconds - time_width) (video_seconds + time_width)], 'YLim', [-200 200]);
    
    image(vidFrame(351:890, 481:1440, 1:3), 'Parent', vid_plot); %crop
    vid_plot.Visible = 'off';
    
    
    %if time has progressed enough, display the next data frame    
    while ((current_data_frame < data_frames) && (video_seconds >= time_elapsed_seconds(current_data_frame)))
        current_data_frame = current_data_frame + 1;
        
        if(exist('head_alt'))
            delete(head_alt);
            delete(head_output);
            delete(head_P);
            delete(head_I);
            delete(head_D);
        end
            
        
        %update alt_line plot
        subplot(2,3,1);
        %set(gca, 'XLim', [(time_elapsed_seconds(current_data_frame) - time_width) (time_elapsed_seconds(current_data_frame) + time_width)], 'YLim', [0 100]);
        addpoints(alt_line, time_elapsed_seconds(current_data_frame), alt_cm(current_data_frame));
        head_alt = scatter(time_elapsed_seconds(current_data_frame), alt_cm(current_data_frame), 'filled', 'MarkerFaceColor', 'r');
        
        %update output_line plot
        subplot(2,3,3);
        %set(gca, 'XLim', [(time_elapsed_seconds(current_data_frame) - time_width) (time_elapsed_seconds(current_data_frame) + time_width)], 'YLim', [0 100]);
        addpoints(output_line, time_elapsed_seconds(current_data_frame), output(current_data_frame));
        head_output = scatter(time_elapsed_seconds(current_data_frame), output(current_data_frame), 'filled', 'MarkerFaceColor', 'r');
        
        %update P_line plot
        subplot(2,3,4);
        %set(gca, 'XLim', [(time_elapsed_seconds(current_data_frame) - time_width) (time_elapsed_seconds(current_data_frame) + time_width)], 'YLim', [-100 100]);
        addpoints(P_line, time_elapsed_seconds(current_data_frame), P(current_data_frame)); 
        head_P = scatter(time_elapsed_seconds(current_data_frame), P(current_data_frame), 'filled', 'MarkerFaceColor', 'r');
        
        %update I_line plot
        subplot(2,3,5);
        %set(gca, 'XLim', [(time_elapsed_seconds(current_data_frame) - time_width) (time_elapsed_seconds(current_data_frame) + time_width)], 'YLim', [-100 0]);
        addpoints(I_line, time_elapsed_seconds(current_data_frame), I(current_data_frame)); 
        head_I = scatter(time_elapsed_seconds(current_data_frame), I(current_data_frame), 'filled', 'MarkerFaceColor', 'r');
        
        %update D_line plot
        subplot(2,3,6);
        %set(gca, 'XLim', [(time_elapsed_seconds(current_data_frame) - time_width) (time_elapsed_seconds(current_data_frame) + time_width)], 'YLim', [-200 200]);
        addpoints(D_line, time_elapsed_seconds(current_data_frame), D(current_data_frame));
        head_D = scatter(time_elapsed_seconds(current_data_frame), D(current_data_frame), 'filled', 'MarkerFaceColor', 'r');
        
        drawnow();
    end
    
    
    
    % Every third frame of footage in, produce one frame out (30fps/2 = 10)
    if (mod(frame_in_index, 2) == 1)
        frame_out_index = frame_out_index + 1;        
        F_out(frame_out_index) = getframe(gcf);
    end
end

% Get ready to start capturing the fnished video
v_out = VideoWriter('Hover with data graphics', 'MPEG-4');
v_out.FrameRate = 15;
open(v_out)
writeVideo(v_out, F_out);
close(v_out)
