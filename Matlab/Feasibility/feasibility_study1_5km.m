% This first study takes variables and ideas from the
% article by Collins, Twining and Wells at Planck Aerosystems
% https://ieeexplore.ieee.org/document/8085014?fbclid=IwAR2H60hFzE6XRrPXwAemlRFwGuxJx47tzeFImufbPNqVmFmh74uMaWm0md0

% UNITS:
% distance - km Kilometre
% money - $ USD
% time - hr hours

% Variables 

draw = 0;       %graph toggle
simulations = 1000;       %number of simulations (keep this low if draw = 1)
demonstation_step = 0;      %wait for human input at given points
simulation_print = 0;   %print stats from each simulation

%fuel burn rate
ship_speed = 7 * (1/0.539957);          % 7NM/hr * (1/0.54NM/km)
drone_speed = 30 * (1/0.539957);
ship_cost_per_distance = 3 * 0.539957;          % 3$/NM * 0.54NM/Km
drone_cost_per_hr = (0.02 * 0.539957) * drone_speed;    %cost_per_distance * speed

% tips 
ship_tip_visible_range = 0.5;                   % estimated visible range of 0.5km for tips
tip_accuracy = 0.2;             % 20% of tips have fish
ship_tip_sensitivity = 1;       % no false negatives from ship NOT USED IN THIS REV
ship_tip_specificity = 0;       % no negative detection from ship NOT USED IN THIS REV
drone_tip_sensitivity = 1;      % no false negatives from drone NOT USED IN THIS REV
drone_tip_specificity = 1;      % no false positives from drone NOT USED IN THIS REV

%fish
fraction_fish_visible_air = 0.2;    % 20% of fish can be seen without sonar
drone_fish_visible_range = 0.25;                 % estimated range of 0.25km for sonar
fish_density = (1 * tip_accuracy / fraction_fish_visible_air) / (0.5 * 2 * 5 * (1/0.539957)); % estimated density of "fish tips" using the mean average distance between tips

%drones
drone_max_flight_time = 2;      % 2 hours
drone_scan_time = 4 / 60;       % 4min / 60min/hr
number_of_drones = 20;
distance_between_scans = 0.5; 
search_radius = 5;
drone_search_pattern = 'triangle';
centre_scan = 1;

%model
ship_location = 0;                  %ship at origin
fish_field_size = 20;               %20km side square
fish_field_type = 'uniform';        %uniform (grid) of fish

% Generate drones
drones = generate_drones(number_of_drones, distance_between_scans, search_radius, drone_search_pattern, centre_scan);

% Fish finding simuation
cost = NaN(1, simulations);
time = NaN(1, simulations);
drone_passes = zeros(1, simulations);
for i = [1:simulations]
    if simulation_print == 1 disp(['Simulation ', num2str(i)]), end
    % this simulation uses the idea that if one pass of the drones does not
    % find any fish, by the time a second pass happens the fish will have moved
    % enough to be accurately moddeled by another independent fish field
    fish_found = 0;
    cost(i) = 0;
    time(i) = 0;
    drone_passes(i) = 0;
    while fish_found == 0
        % Generate tips
        fish = generate_fish(fish_density, fish_field_size, fish_field_type);
        % Calculate cost and time to find fish and location
        [nearest_fish, search_time, search_cost, scan_index] = search(drones, fish, drone_speed, drone_cost_per_hr, drone_fish_visible_range, drone_scan_time, distance_between_scans, ship_location);
        cost(i) = cost(i) + search_cost;
        time(i) = time(i) + search_time;
        if length(nearest_fish) > 0         %stop looking for fish if they have been found
            fish_found = 1;
        end
        drone_passes(i) = drone_passes(i) + 1;    %increment drone_passes
    end
    if simulation_print == 1 disp(['Finding fish: t = ', num2str(time(i)), 'hr, c = ', num2str(cost(i)), '$, drone_passes = ', num2str(drone_passes(i))]), end
    % Calculate cost and time to get to find fish
    [sail_time, sail_cost] = sail(ship_speed, ship_cost_per_distance, ship_location, nearest_fish);
    if simulation_print == 1 disp(['Sailing to fish: t = ', num2str(sail_time), 'hr, c = ', num2str(sail_cost), '$']), end
    cost(i) = cost(i) + sail_cost;
    time(i) = time(i) + sail_time;
    if simulation_print == 1 disp(['Total: t = ', num2str(time(i)), 'hr, c = ', num2str(cost(i)), '$']), end
    
    %Plot
    if draw == 1
        clf         %clear previous data
        hold on;    %retain data between plots
        axis equal;     %set x and y axis to same scale
        axis([(-1.5 * search_radius) (1.5 * search_radius) (-1.5 * search_radius) (1.5 * search_radius)]);  %set boundaries of plot
        %plot ship
        plot(0, 0, 'rd');
        if demonstation_step == 1 pause, end
        %plot fish
        plot(fish, 'xm');
        if demonstation_step == 1 pause, end
        %plot scan radius
        viscircles([0, 0], search_radius, 'Color', 'r', 'LineWidth', 0.25);
        if demonstation_step == 1 pause, end
        %plot drone routes
        for i = [1:number_of_drones]
            plot(drones(i).route, '--b'); %draw drone route
        end
        if demonstation_step == 1 pause, end
        %plot drone scan radius
        for i = [1:number_of_drones]
            %draw a circle with the drones radius of effect at all areas
            %scanned
            viscircles(transpose([real(drones(i).route(1:scan_index)); imag(drones(i).route(1:scan_index))]), drone_fish_visible_range * ones(size(transpose(drones(i).route(1:scan_index)))), 'Color', 'b', 'LineWidth', 0.25);
        end
        if demonstation_step == 1 pause, end
        %plot nearest fish
        plot(nearest_fish, '*r');
        if demonstation_step > 0 pause, end
    end
end

%Cost taken from paper at top
no_drone_cost = (5 / 0.539957) * ship_cost_per_distance;
no_drone_time = (5 / 0.539957) / ship_speed;

disp(['No drone expected: t = ', num2str(no_drone_time), 'hr, c = ', num2str(no_drone_cost), '$']);
disp(['Expected total: t = ', num2str(mean(time)), 'hr, c = ', num2str(mean(cost)), '$, drone_passes = ', num2str(mean(drone_passes))]);
expected_cost_savings = no_drone_cost - mean(cost);
expected_time_savings = no_drone_time - mean(time);
disp(['Expected savings: t = ', num2str(expected_time_savings), 'hr, c = ', num2str(expected_cost_savings), '$']);
disp([' ']);    %white space

% Functions
% return the cost and time take to get to sail to a location
function [t, c] = sail(ship_speed, ship_cost_per_distance, ship_location, nearest_fish)
distance = seperation(ship_location, nearest_fish);     %done this way for clarity
c = distance * ship_cost_per_distance;
t = distance / ship_speed;
end

% return the nearest fish, how many scans it took to find and the time and cost to find 
function [nf, t, c, si] = search(drones, fish, drone_speed, drone_cost_per_hr, drone_fish_visible_range, drone_scan_time, distance_between_scans, ship_location)
nf = 0;
t = 0;
c = 0;
si = 0;
number_of_drones = length(drones);
number_of_scans_per_drone = length(drones(1).route);        
number_of_fish = length(fish);                          %done this way for clarity
for si = [1:number_of_scans_per_drone]
    fish_found = [];
    for di = [1:number_of_drones]
        for fi = [1:number_of_fish]
            if seperation(fish(fi), drones(di).route(si)) <= drone_fish_visible_range
                fish_found = [fish_found, fish(fi)];
            end
        end
    end
    if length(fish_found) > 0       % stop scanning if 1 or more fish have been found during a round of scans
        break
    end
end
nf = get_closest(fish_found, ship_location);            %closest fish out of fish found at the same time
l_there = NaN(1, length(drones));                           %pre-allocate length arrays
l_there_and_back = NaN(1, length(drones));
for di = [1:length(drones)]
    l_there(di) = trip_length(0, drones(di).route, drones(di).route(end));
    l_there_and_back(di) = l_there(di) + seperation(drones(di).route(end), 0);
end
t = (max(l_there) / drone_speed + drone_scan_time * si);      %time taken to get to the furthest scan in the batch
c = (sum(l_there_and_back) / drone_speed + si * length(drones) * drone_scan_time) * drone_cost_per_hr; %cost to get to and from all of the scan spots for each drone
end

% calculate the shortest path connecting all points in order
function tl = trip_length(start, route, finish)
last_location = start;
tl = 0;
for i = [1:length(route)]
    tl = tl + seperation(last_location, route(i));
    last_location = route(i);
end
tl = tl + seperation(finish, last_location);
end

% return the closest element in locations to location, return [] if
% length(locations) == 0
function c = get_closest(locations, location)
c = [];
for i = [1:length(locations)]
    if i == 1
        c = locations(i);           % set first element to the closest c
    else
        if seperation(c, location) > seperation(locations(i), location)
            c = locations(i);       % if another element is closer, update c
        end
    end
end
end

%return the cartesian distance between two pints
function d = seperation(location1, location2)     %distance between two complex points
d = abs(location1 - location2);
end

%randomly rotate and translate all fish locations
function rtf = fish_rotate_translate(fish, translation_limit)
rotate = exp(1i * (pi/2) * rand);
translate = (rand + rand * 1i) * translation_limit;
rtf = rotate * fish + translate;
end

%generate a uniform grid of fish with random rotation and translation
function uf = generate_uniform_fish(fish_density, fish_field_size)
fish_spacing = (fish_density^-1)^0.5;         % distance between tips on a square grid for given density
n = floor(fish_field_size / (fish_density^-1)^0.5);   % number of tips in a row at this grid size
x_grid = ones([n + 1 ,1]) * [0:n] - floor(n/2)* ones(n + 1);
y_grid = flip(transpose(x_grid));
grid = x_grid + y_grid*1i;
locations = grid * fish_spacing;
uf = [];
for i = [1:(n+1)^2]
    new_fish = [locations(i)];      %used this method to build uf as fish will have attributes in the next model     
    uf = [uf, new_fish];
end
uf = fish_rotate_translate(uf, fish_spacing);
end

function sf = generate_scatter_fish(fish_density, fish_field_size)
n = fish_field_size^2 * fish_density;
n_decimal = n - floor(n);       %take the stuff after the decimal point
round = floor(rand / (1 - n_decimal));  %1 or 0 with a distribution with e(x) = n_decimal
n = floor(n) + round;           % n is number of ish to randomly generate within the fish field

sf = NaN(1, n);             % pre-allocate sf
for i = [1:n]
    %consider that fish may have attributes in later models (how long to
    %remain/ accuracy of detection)
    new_fish = ((rand - 0.5) + 1i * (rand - 0.5)) * fish_field_size;    
    sf(i) = new_fish;
end
end

%generate the fish locations
function f = generate_fish(fish_density, fish_field_size, mode)
f = 0;
if (mode == 'uniform')
    f = generate_uniform_fish(fish_density, fish_field_size);
else if (mode == 'scatter')
        f = generate_scatter_fish(fish_density, fish_field_size);
    end
end
end

%generate triangular drone route
function tr = triangle_route(drone_i, number, distance_between_scans, search_radius, centre_scan)
if (drone_i == 1) && (centre_scan == 1)                     % first drone scans by boat
    out_offset = -1;
else
    out_offset = 0;
end
    
theta = 2 * pi * drone_i / number;
d_theta = 2 * pi / (number * 2);

scans = floor(search_radius / distance_between_scans);
out_route = exp(1i * theta) * [1 + out_offset : scans + out_offset] * distance_between_scans;
in_route = exp(1i * (theta + d_theta)) * flip([1:scans]) * distance_between_scans;
tr = [out_route, in_route];

end

%generate spiral drone route
function sr = spiral_route(drone_i, number, distance_between_scans, search_radius, centre_scan)

theta0 = 2 * pi * drone_i / number;
scans = 50;

r_coefficient = distance_between_scans * number / (2 * pi);

theta = NaN(1, scans);
theta(1) = 1;
for i = [2:scans]
    d_theta = 2 * pi * (distance_between_scans) / (theta(i - 1) * r_coefficient * 2 * pi);
    theta(i) = theta(i - 1) + d_theta;
end

if (drone_i == 1) && (centre_scan == 1)                     % first drone scans by boat (so add a scan at 0 + 0i)
    theta = [0, theta(1 : scans -1)];
end

sr = spiral(theta, theta0, r_coefficient);
end

function s = spiral(theta, theta0, r_coefficient)
r = r_coefficient * theta;
s = r .* exp(1i * (theta + theta0));
end

%generate specified number of drones
function drones = generate_drones(number, distance_between_scans, search_radius, drone_search_pattern, centre_scan)
drones = [];
for i = 1:number
    if strcmp(drone_search_pattern, 'triangle')
        drones = [drones, Drone(0, triangle_route(i, number, distance_between_scans, search_radius, centre_scan))];
    else if strcmp(drone_search_pattern, 'spiral')
            drones = [drones, Drone(0, spiral_route(i, number, distance_between_scans, search_radius, centre_scan))];
        end
    end
   
end
end

