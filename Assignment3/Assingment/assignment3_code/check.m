wp_all = [1,2,3,4,5;6,7,8,9,0;1,2,3,4,5];
wp = wp_all(1,1:end);
n = size(wp,2)-1;

% We have to find minimum snap trajectory, so number of unknown coefficents = 8
% Also, Since we have 8 unknown coefficents, we need to have 8 boundary
% conditions
% for each pair of waypoints i.e (x1,t1) & (x2,t2) we need matrix 8 x 8
% according to its boundary condition
% We make equation in the form A*c = b where, c = coefficents, A & b are
% matrix representing constraints.

A = zeros(8*n,8*n);
b = zeros(8*n,1);

% constraint 1: p_i(Si-1) = wi-1
% Position constraint
for i = 1:n
    x_shift = 8*(i-1);
    A(i,x_shift+1) = 1;
    b(i) = wp(i);
end
%%
% Constraint 2: p_i(Si) = wi
% Position constraint
y_shift = n;
for i = 1:n
    x_shift = 8*(i-1);
    A(y_shift+i,x_shift+1:x_shift+8)=1;
    b(y_shift+i) = wp(i+1);
end
%%
% constraint 3: p_1^k(S0) = 0
% Velocity, acceleration, jerk at starting = 0
    % values:
    % 0 1 0 0 0 0 0 0
    % 0 0 2 0 0 0 0 0
    % 0 0 0 6 0 0 0 0
    y_shift = 2*n;
    for k = 1:3
        A(y_shift+k,1:8) = getPolyCoefficients(8, k, 0);
    end

%%   
% constraint 4: p_n^k(Sn) = 0
% Velocity, acceleration, jerk at time = T is 0
    % values:
    % 0 1 0 0 0 0 0 0
    % 0 0 2 0 0 0 0 0
    % 0 0 0 6 0 0 0 0
y_shift = 2*n+3;
for k = 1:3
    A(y_shift+k, end-7:end) = getPolyCoefficients(8, k, 1);
end

%%
% constraint 5: p_i^k(Si) = p^k_i+1(Si)
    for i = 1:n-1
        x_shift = 8*(i-1);
        for k = 1:6
            y_shift = 2*n+6 + 6*(i-1);
            A(y_shift+k, x_shift+1:x_shift+16) = [getPolyCoefficients(8, k, 1), -1 * getPolyCoefficients(8, k, 0)];
        end
    end

%%
function [elements] = getPolyCoefficients(length, derivative, value)
    coefficients = ones(1, length);
    exponents = zeros(1, length);
    for i = 1:length
        exponents(1,i) = i-1;
    end
    for d = 1:derivative
        for i = 1:length
            coefficients(1,i) = coefficients(1,i) * exponents(1,i);
            exponents(1,i) = max(0, exponents(1,i) - 1);
        end
    end
    elements = coefficients .* (value .^ exponents);
end
