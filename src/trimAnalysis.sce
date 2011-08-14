scf(1); clf(1);
nPlots = 2;
subplot(1,nPlots,1);
data = read('f16.log',-1,1);
plot(log(1:size(data,1),log(data));
xlabel('iteration');
ylabel('log(cost)');
title('f16 - level, 500 kts');
legend('newton raphson')

subplot(1,nPlots,2);
data = read('c172p.log',-1,1);
plot(log(1:size(data,1)),log(data));
xlabel('iteration');
ylabel('log(cost)');
title('c172p - level, 20 kts');
legend('newton raphson')
