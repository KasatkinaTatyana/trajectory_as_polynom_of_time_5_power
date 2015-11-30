function [time, F, U] = t5_modeling(Y_0, t_0, t_end)
% � ������ ������� ������������ �������������� ������� �� �������
global control_arr
global count

count = length(control_arr(:,1));

control_arr = sortrows(control_arr,1);

options = odeset('RelTol',1e-6,'AbsTol',[1e-6 1e-6]);
global Mas_u % ������ ����������
[T, Y] = ode45(@t5_control_system, [t_0, t_end], [Y_0(1) Y_0(2)], options);

F = Y;
time = T;
U = Mas_u;
