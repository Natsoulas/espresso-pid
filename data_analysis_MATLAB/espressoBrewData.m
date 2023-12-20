function espressoBrewData

range0 = [90,100];
range1 = [94,100];
range2 = [84, 89];

% espresso_curve_fit('PID_Channel_94_01.csv',range0);
% title('Brew Temperature During Heat Up PID Run 01')
% 
% espresso_curve_fit('PID_Channel_94_02.csv',range0);
% title('Brew Temperature During Heat Up PID Run 02')
% 
% espresso_curve_fit('PID_Channel_94_03.csv',range0);
% title('Brew Temperature During Heat Up PID Run 03')
% 
% espresso_curve_fit('PID_Channel_94_04.csv',range0);
% title('Brew Temperature During Heat Up PID Run 04')
% 
% 
% espresso_curve_fit('PID_Channel_94_05.csv',range0);
% title('Brew Temperature During Heat Up PID Run 05')

espresso_curve_fit('PID_Channel_Set106_01.csv',range1);
title('Brew Temperature During Heat Up PID Setpoint 106 Run 01')

espresso_curve_fit('PID_Channel_Set106_02.csv',range1);
title('Brew Temperature During Heat Up PID Setpoint 106 Run 02')

espresso_curve_fit('PID_Channel_Set106_03.csv',range1);
title('Brew Temperature During Heat Up PID Setpoint 106 Run 03')


espresso_curve_fit('OG_Channel_01.csv',range1);
title('Brew Temperature During Heat Up OG Machine Run 01')

espresso_curve_fit('OG_Channel_02.csv',range2);
title('Brew Temperature During Heat Up OG Machine Run 02')

espresso_curve_fit('OG_Channel_03.csv',range1);
title('Brew Temperature During Heat Up OG Machine Run 03')




