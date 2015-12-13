clc;
close all;

% Concatenate data
perf1 = [privFixed1_metric1' privFixed2_metric1' privFixed3_metric1' privFixed4_metric1' privFixed5_metric1' privFixed6_metric1' privVar1_metric1' privVar2_metric1' privVar3_metric1' privVar4_metric1' privVar5_metric1' privVar6_metric1' privVar7_metric1' privVar8_metric1'];
perf2 = [privFixed1_metric2' privFixed2_metric2' privFixed3_metric2' privFixed4_metric2' privFixed5_metric2' privFixed6_metric2' privVar1_metric2' privVar2_metric2' privVar3_metric2' privVar4_metric2' privVar5_metric2' privVar6_metric2' privVar7_metric2' privVar8_metric2'];

perf3 = perf1.*180;
% perf3 = perf1.*(1-perf2).*(1-perf2).*180;

comps = 1:1:length(perf1(1,:));

mean_perf1 = mean(perf1);
mean_perf2 = mean(perf2);
mean_perf3 = mean(perf3);
std_dev1 = std(perf1);
std_dev2 = std(perf2);
std_dev3 = std(perf3);

results1 = [mean_perf1' std_dev1'];
results2 = [mean_perf2' std_dev2'];

% Plot the results
figure;
subplot(3,1,1);
hold on
for i=1:1:length(perf1(1,:))
    errorbar(comps(i), mean_perf1(i), std_dev1(i), 'x');
end
ylabel('First metric');
title('Events handled per time unit');


subplot(3,1,2);
hold on
for i=1:1:length(perf2(1,:))
    errorbar(comps(i), mean_perf2(i), std_dev2(i), 'x');
end
ylabel('Second metric');
title('Distance traveled per event handled');

subplot(3,1,3);
hold on
for i=1:1:length(perf1(1,:))
    errorbar(comps(i), mean_perf3(i), std_dev3(i), 'x');
end
ylabel('Third metric');
title('Events handled');