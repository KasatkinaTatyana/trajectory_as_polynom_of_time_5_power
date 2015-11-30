function F = t5_trajectory_synthesis(Y_0, Y_end, t_0, t_end, dt)

c_t = t5_calc_coefs(Y_0, Y_end, t_0, t_end);
c0 = c_t(1);
c1 = c_t(2);
c2 = c_t(3);
c3 = c_t(4);
c4 = c_t(5);
c5 = c_t(6);

t = t_0:dt:t_end;

Psi = c0 + c1/(t_end - t_0)*(t - t_0) + c2/(t_end - t_0)^2*(t - t_0).^2 + c3/(t_end - t_0)^3*(t - t_0).^3 + ...
      c4/(t_end - t_0)^4*(t - t_0).^4 + c5/(t_end - t_0)^5*(t - t_0).^5;
dPsi = c1/(t_end - t_0) + 2*c2/(t_end - t_0)^2*(t - t_0) + 3*c3/(t_end - t_0)^3*(t - t_0).^2 + ...
       4*c4/(t_end - t_0)^4*(t - t_0).^3 + 5*c5/(t_end - t_0)^5*(t - t_0).^4;
ddPsi =  2*c2/(t_end - t_0)^2 + 6*c3/(t_end - t_0)^3*(t - t_0) + ...
         12*c4/(t_end - t_0)^4*(t - t_0).^2 + 20*c5/(t_end - t_0)^5*(t - t_0).^3;

F = [Psi; dPsi; ddPsi];