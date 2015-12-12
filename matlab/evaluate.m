clc;
close all;

% Concatenate data
perf1 = [privFixed1_metric1' privFixed2_metric1' privFixed3_metric1' privFixed4_metric1' privFixed5_metric1' privFixed6_metric1'];
perf2 = [privFixed1_metric2' privFixed2_metric2' privFixed3_metric2' privFixed4_metric2' privFixed5_metric2' privFixed6_metric2'];
comps = 1:1:length(perf1(1,:));

mean_perf1 = mean(perf1);
mean_perf2 = mean(perf2);
std_dev1 = std(perf1);
std_dev2 = std(perf2);

means1 = [mean_perf1' std_dev1'];
means2 = [mean_perf2' std_dev2'];

% Plot the results
figure;
subplot(2,1,1);
err1 = std_dev1.*comps;
hold on
for i=1:1:length(perf1(1,:))
    errorbar(comps(i), mean_perf1(i), err1(i), 'x');
end
title('First metric');
ylabel('Events handled per time unit');
legend('Private, fixed threshold = 5', 'Private, fixed threshold = 8');


subplot(2,1,2);
err2 = std_dev2.*comps;
hold on
for i=1:1:length(perf2(1,:))
    errorbar(comps(i), mean_perf2(i), err2(i), 'x');
end
title('Second metric');
ylabel('Distance traveled per event handled');
legend('Private, fixed threshold = 5', 'Private, fixed threshold = 8');