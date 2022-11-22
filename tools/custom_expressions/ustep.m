function out = ustep(x,trig)
    % Approximates a uniform step function using a sigmoid
    c1 = 1e6;
    out = 1./(1 + exp(-c1*(x-trig)));
   
end
