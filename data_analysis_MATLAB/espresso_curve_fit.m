
function [finalTime, finalTemp] = espresso_curve_fit(datafile,temp_range)

data = csvread(datafile);
time = data(:,1);


[IndexS01, IndexE01, nTime01] = extractTemperature( data(:,1), data(:,2),temp_range);
parsed_brew = data(IndexS01:IndexS01+50,2);

parsed_time = time(IndexS01:IndexS01+50,:);
parsed_time = (parsed_time - parsed_time(1))/1000;


smoothedTemperature5 = movmean(parsed_brew, 5);
smoothedTemperature10 = movmean(parsed_brew, 10);
finalTemp = smoothedTemperature10;
finalTime = parsed_time;




figure
hold on
plot(parsed_time, parsed_brew)
plot(parsed_time, smoothedTemperature5)
plot(parsed_time, smoothedTemperature10,'m')
title('Brew Temperature During Heat Up')
legend('Raw Signal','Filtered Signal, 5pts Used','Filtered Signal, 10pts Used')
xlabel('Time (second)')
ylabel('Temperature (deg C)')
hold off
