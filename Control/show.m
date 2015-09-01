close all;
run('C:\Users\Filip\Documents\Git\drone\Control\data.m')

avg = filter(ones(1, 20)/20, 1, roll);
hold on
plot(roll, 'b');
plot(roll_avg,'k');
%plot(avg,'r');
hold off

figure
hold on;
plot(chan1);
plot(chan2, 'r');
plot(chan3,'c');
plot(chan4,'k');
hold off