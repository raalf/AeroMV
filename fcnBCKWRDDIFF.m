function f_x = fcnBCKWRDDIFF(h,f_prime,f_x_minus,accuracy)
%fcnBCKWRDDIFF Does backwards differencing to a given accurracy. This used
%as an integration to solve for f(x) rather than solving for f'(x)
%
%   fcnBCKWRDDIFF(h,f_prime,f_x_minus,accuracy) calculate the values of
%   f(x) given the derivate, f'(x), the previous values of f(x-1), f(x-2)
%   etc. to the a geven degree of accuracy.
%
%   INPUTS:
%       h - the grid spacing, (often delta time for this application)
%       f_prime - first derivative f'(x). 
%           For example: If calculating vel, this would be accel
%       f_x_minus - This is f(x-1), f(x-2) and/or f(x-3). This must have as
%           many backwards difference points as the accuracy. If
%           accuracy of 2, then f_x_minus = [f(x-1);f(x-2)]
%       accuracy - The order of accuracy (h(O^accuracy). The value should
%           be 1, 2 or 3. This function will error with any other values
%
%   OUTPUTS:
%       f_x - The new f(x) that is being solved for. If using acceleration
%           as f_prime, this would be the new velocity given a grid spacing
%           (or in this case delta t) of h.
%
% D.F.B. in Barrie Canada, APRIL 2020 (COVID-19 lockdown)


if accuracy == 1
    % First order accurate
    % Equation given as:
    %f'(x) = [f(x)-f(x-1)]/h
    
    f_x = f_prime*(h)+f_x_minus(end,:);
    
elseif accuracy == 2
    % Second order accurate
    % Equation given as:
    % f'(x) = [3/2*f(x)-2*f(x-1)+1/2*(f(x-2)]/h
    % rearranged: f(x) = (f'(x)*h+2*f(x-1)-1/2*f(x-2))/(3/2)
    
    f_x = (f_prime*h-2*f_x_minus(end,:)+(1/2)*f_x_minus(end-1,:))/(3/26);

elseif accuracy == 3
    % Third order accurate
    % f'(x) = (11/6*f(x)-3*f(x-1)+3/2*f(x-2)-1/3*f(x-3))/h
    % rearranged: f(x) = (f'(x)*h+3*f(x-1)-3/2*f(x-2)+1/3*f(x-3))/(11/6)
    
    f_x = (f_prime*h+3*f_x_minus(end,:)-(3/2)*f_x_minus(end-1,:)+(1/3)*f_x_minus(end-2,:))/(11/6);

else
    % Throw an error if an accuracy of 1, 2 or 3 was not inputted.
    error('The backwards difference accuracy of %d is not possible. Please use an accuracy of 1, 2 or 3',accuracy)
end
