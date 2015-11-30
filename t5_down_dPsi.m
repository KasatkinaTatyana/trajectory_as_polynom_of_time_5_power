function [F1, F2, params, coefs] = t5_down_dPsi(Y_0, Y_end, t_0, t_end, dt, dy_constr1)
% Функция генерирует кривую, отрезок, на котором она строится, и ее
% производную таким образом, чтобы выполнялось ограничение dPsi(t) / dt >
% dy_constr1

% F1 соответсвует первой кривой (левой),
% F2 - правой, они имеют следующую структуру
% массив t - массив функции Psi(t) - массив dPsi / dt

% params и coefs нужны для того, чтобы затем однозначно восстановить кривую
% и построить нужное управление

c_t = t5_calc_coefs(Y_0, Y_end, t_0, t_end);
c0 = c_t(1);
c1 = c_t(2);
c2 = c_t(3);
c3 = c_t(4);
c4 = c_t(5);
c5 = c_t(6);

d0 = 4.375;
d1 = -5.25;
d2 = 1.875;

t = t_0:dt:t_end;

Psi = c0 + c1/(t_end - t_0)*(t - t_0) + c2/(t_end - t_0)^2*(t - t_0).^2 + c3/(t_end - t_0)^3*(t - t_0).^3 + ...
      c4/(t_end - t_0)^4*(t - t_0).^4 + c5/(t_end - t_0)^5*(t - t_0).^5;
dPsi = c1/(t_end - t_0) + 2*c2/(t_end - t_0)^2*(t - t_0) + 3*c3/(t_end - t_0)^3*(t - t_0).^2 + ...
       4*c4/(t_end - t_0)^4*(t - t_0).^3 + 5*c5/(t_end - t_0)^5*(t - t_0).^4;

N_shift = 20;

N = (t_end - t_0)/dt + 1;

flag = 0;
% Ищем границы интервала, на котором нарушается ограничение
for i=1:N
    % ------- -- dy < constr -----------------------------------
    if ((flag == 0)&&(dPsi(i) <= dy_constr1))
    % -----------------------------------------------------------------
        if ((i - N_shift) < 1)
            t_left = t_0;
            Y_left = Y_0;
        else
            t_left = t(i - N_shift);
            y_left = Psi(i - N_shift);
            dy_left = dPsi(i - N_shift);
        end
        flag = 1;
    end
    % --------- dy > constr ------------------------------------
    if ((flag==1)&&(dPsi(i) > dy_constr1))
    % -----------------------------------------------------------------
        if ((i + N_shift) > N + 1)
            t_right = t_end;
            Y_right = Y_end;
        else
            t_right = t(i + N_shift);
            y_right = Psi(i + N_shift);
            dy_right = dPsi(i + N_shift);
        end
        break;
    end
end

t_1 = t_left:dt:t_right;
t_norm = (t_1 - t_left) / (t_right - t_left);     
h_d = 0.001;
for d = 0:h_d:(1)
    Psi_1 = c0 + c1/(t_end - t_0)*(t_1 - t_0) + c2/(t_end - t_0)^2*(t_1 - t_0).^2 + c3/(t_end - t_0)^3*(t_1 - t_0).^3 +...
            c4/(t_end - t_0)^4*(t_1 - t_0).^4 + c5/(t_end - t_0)^5*(t_1 - t_0).^5 ...
            + d*(t_norm.^3).*(d0 + d1*t_norm.^2 + d2*t_norm.^4);
    dPsi_1 = c1/(t_end - t_0) + 2*c2/(t_end - t_0)^2*(t_1 - t_0) + 3*c3/(t_end - t_0)^3*(t_1 - t_0).^2 + ...
             4*c4/(t_end - t_0)^4*(t_1 - t_0).^3 + 5*c5/(t_end - t_0)^5*(t_1 - t_0).^4 ...
             + d*(3*d0*(t_1 - t_left).^2/(t_right - t_left)^3 + 5*d1*(t_1 - t_left).^4/(t_right - t_left)^5 + ...
             7*d2*(t_1 - t_left).^6/(t_right - t_left)^7);
            
    if (min(dPsi_1) > dy_constr1)
        break;
    end
end
ddPsi_1 = 2*c2/(t_end - t_0)^2 + 6*c3/(t_end - t_0)^3*(t_1 - t_0) + ...
          12*c4/(t_end - t_0)^4*(t_1 - t_0).^2 + 20*c5/(t_end - t_0)^5*(t_1 - t_0).^3 + ...
          d*(6*d0*(t_1 - t_left) / (t_right - t_left)^3 + 20*d1*(t_1 - t_left).^3 / (t_right - t_left)^5 + ...
          42*d2*(t_1 - t_left).^5 / (t_right - t_left)^7);

Y_1_end = [Psi_1(end) dPsi_1(end) ddPsi_1(end)];

replace_part = t5_trajectory_synthesis(Y_1_end, Y_end, t_right, t_end, dt);
coefs_2 = t5_calc_coefs(Y_1_end, Y_end, t_right, t_end);
t_2 = t_right:dt:t_end;

Psi_2 = replace_part(1,:);
dPsi_2 = replace_part(2,:);
ddPsi_2 = 2*coefs_2(3)/(t_2(end)- t_2(1))^2 + 6*coefs_2(4)/(t_2(end) - t_2(1))^3*(t_2 - t_2(1)) + ...
          12*coefs_2(5)/(t_2(end) - t_2(1))^4*(t_2 - t_2(1)).^2 + 20*coefs_2(6)/(t_2(end) - t_2(1))^5*(t_2 - t_2(1)).^3;

F1 = [t_1; Psi_1; dPsi_1; ddPsi_1];
F2 = [t_2; Psi_2; dPsi_2; ddPsi_2];

params = [d; t_0; t_end; 0];

coefs = [c0 c1 c2 c3 c4 c5; coefs_2'; 0 0 0 0 0 0;0 0 0 0 0 0];