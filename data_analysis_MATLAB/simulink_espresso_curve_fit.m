% data = csvread('espresso_test.csv');
% boiler_temp = data(:,9);
% time = data(:,1);
% figure
% plot(time, boiler_temp)
% title('Boiler Temperature During Heat Up')
% 
% parsedtime = time(1:1429,:);
% parsedtime = (parsedtime-999)/1000; 
% parsedboiler = boiler_temp(1:1429,:);
% 
% 
% parsedtime = reshape(parsedtime,[],1);
% parsedboiler = reshape(parsedboiler,[],1);
% % [f,g]=fit(parsedtime,parsedboiler,'poly2');
% p = polyfit(parsedtime,parsedboiler,2);
% 
% testoutput = p(1).*parsedtime.^2 + p(2) .*parsedtime + p(3);
% 
% figure
% hold on
% plot(parsedtime, parsedboiler);
% plot(parsedtime, testoutput);
% title('Boiler Temperature During Heat Up for Fitting')

function espresso_curve_fit(datafile)

data = csvread(datafile);
time = data(:,1);
% plot(time, brew_temp)
% hold
% plot(time, boiler_temp)
% title('Boiler Temperature vs Time')
% 
% xlabel('Time (millisecond)')
% ylabel('Temperature (deg C)')



[IndexS01, IndexE01, nTime01] = extractTemperature( data(:,1), data(:,2));
parsed_brew = data(IndexS01:IndexS01+50,2);

parsed_time = time(IndexS01:IndexS01+50,:);
parsed_time = (parsed_time - parsed_time(1))/1000;


smoothedTemperature5 = movmean(parsed_brew, 5);
smoothedTemperature10 = movmean(parsed_brew, 10);




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



% testoutput = p(1).*parsedtime.^2 + p(2) .*parsedtime + p(3);

% figure
% hold on
% plot(parsedtime, parsedbrew);
% plot(parsedtime, testoutput);