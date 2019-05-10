function Drone_Calib_polyfit()
%nth order polynomial fit
distance = [20	30	40	50	60	70	80	90	100	110	120	130	140	150];
measurement = [532.48	421.888	315.392	253.952	206.848	176.128	151.552	143.36	124.928	110.592	100.352	90.112	83.968	79.872];
min_measurement = 80;
max_measurement = 533;
measurement_estimate = min_measurement:max_measurement;
n = 5; %5th order

p = polyfit(measurement, distance, n)

distance_estimate = polyval(p, measurement_estimate);

plot(distance, measurement,'b*')
hold on;
plot(distance_estimate,measurement_estimate,'b-')
hold off;

end
