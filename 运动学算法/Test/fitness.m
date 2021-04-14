clear all
clc
figure(1)
load('Data_fitness.txt');
X=1:length(Data_fitness);
Y=Data_fitness(:,1);
plot(X,Y,'LineWidth',2)
xlabel('迭代次数/n');ylabel('取得最优解时的函数值')

figure(2)
load('fitness_1.txt');
X_1 = 50:49+length(fitness_1);
Y_1 = fitness_1(:,1);
plot(X_1,Y_1,'LineWidth',2)
% axis square

figure(3)
load('manipulability.txt');
X_2 = 1:length(manipulability);
Y_2 = manipulability(:,1);
plot(X_2,Y_2,'LineWidth',2)
xlabel('迭代次数/n');ylabel('速度方向可操作度')

figure(4)
load('manipulability_2.txt');
X_3 = 50:49+length(manipulability_2);
Y_3 = manipulability_2(:,1);
plot(X_3,Y_3,'LineWidth',2)
% axis([50 100 0.0796 0.0806])
axis square

% ness = randi([79800000 80500000],100,1);
% rand_fitness = ness/1000000000
