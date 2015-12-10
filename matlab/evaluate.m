clc;
close all;

mean_perf1 = mean(metric1);
mean_perf2 = mean(metric2);

std_dev1 = std(metric1);
std_dev2 = std(metric2);

figure;
subplot(1,2,1);
boxplot( metric1 );

subplot(1,2,2);
boxplot( metric2 );


figure
subplot(1,2,1);
errorbar(1,mean_perf1,std_dev1,'rx');
axis([0.95 1.05 0.215 0.23]);

subplot(1,2,2);
errorbar(1,mean_perf2,std_dev2,'x');
axis([0.95 1.05 0.5 2]);