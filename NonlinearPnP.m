function [C, R] = NonlinearPnP(X, x, K, C0, R0)
N = size(x,1);
X = [X ones(size(X,1),1)];
x = [x ones(size(x,1),1)];
opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-64, ...
    'TolFun', 1e-64, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'off');
q = rotm2quat(R0);
err_input = [q C0'];
error = @(y)error_reprojection(y, X, x, K);  
err_output = lsqnonlin(error, err_input, [], [], opts); 
C = transpose(err_output(5:7));
R = quat2rotm(err_output(1:4));
end

function [err] = error_reprojection(err_input, X, x, K)
R = quat2rotm(err_input(1:4));
C = err_input(5:7);
P = K*R*[eye(3) -C'];
err = (x(:, 1:2)' - (P(1:2, :)*X')./(P(3,:)*X'));
end