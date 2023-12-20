function [xArray, yArray,temp_average] = getArray (MatlabFigure)


open(MatlabFigure);
a = get(gca,'Children');
xdata = get(a, 'XData');
ydata = get(a, 'YData');

%%% Take the moving average data set%%%
y3 = ydata(3);
y3 = cell2mat(y3);
yArray = y3;

x3 = xdata(3);
x3 = cell2mat(x3);
xArray = x3;
temp_average = mean(y3);



