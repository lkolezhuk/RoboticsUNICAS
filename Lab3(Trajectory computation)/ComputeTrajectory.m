function [ pos, vel, acc ] = ComputeTrajectory( x_in, x_fin, t_fin, v_cruise, t_curr, t_sampling)


vel = 0;


tc = (1/v_cruise) * (x_in - x_fin + v_cruise * t_fin);
acc_max = power(v_cruise,2) / (x_in - x_fin + v_cruise * t_fin);

t = t_curr:t_sampling:t_fin;
N = length(t) - 1;
acc = zeros(N,size(v_cruise,1));
pos = zeros(N,size(x_in,1));
vel = zeros(N,size(v_cruise,1));
for i=1:N
    ti = t_curr + i * t_sampling;
    if(ti < tc) 
        acc(i) = acc_max;
        pos(i) = x_in + 0.5 * acc_max * ti^2;
        vel(i) = acc_max * ti;
    elseif(ti > t_fin - tc)
        acc(i) = - acc_max;
        pos(i) = x_fin - 0.5 * acc_max * (t_fin - ti)^2;
        vel(i) = v_cruise - acc_max * (-t_fin + tc + ti);
    else
        pos(i) = x_in + acc_max * tc * (ti - tc/2);
        vel(i) = acc_max * tc;
    end
    
    
    
    
end

