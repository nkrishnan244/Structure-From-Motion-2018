function [X] = nonlinearFunc(K, C1, R1, C2, R2, x1, x2, X0)
% A pseudo code for nonlinear optimization
% Input - arguments you need
% Output - refine_var to represent the refine variables

%% Pre-process if necessary 
.......

%% Define an optimizer 
% feel free to change parameters or use other optimizers 
opts = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'TolX', 1e-64, ...
    'TolFun', 1e-64, 'MaxFunEvals', 1e64, 'MaxIter', 1e64, 'Display', 'off');


%% Construct objective function (e.g. reprojection error) for minimizing
% e.g. 
% Triangulation: the variable is 3D point cloud estimated from Linear Triangulation 
% below, iterate all 3D points one by one
x1 = [x1 ones(size(x1,1),1)];
x2 = [x2 ones(size(x1,1),1)];
P1 = K*R1*[eye(3) -C1];
P2 = K*R2*[eye(3) -C2];
N = size(X0,1);
X = zeros(size(X0));

for i = 1 : N
  PC_3D_i = X0(i,:);

  % reprojection error computation, x is the variable you want to optimize
  error_i = @(x)error_reprojection(x, P1, P2, x1(i,:), x2(i,:));  

  PC_3D_i_refine = lsqnonlin(error_i, PC_3D_i, [], [], opts); 
  X(i,:) = PC_3D_i_refine;
end


% %% Post-process if necessary
% .......
% 
% 
% refine_var = .......; 
end


%% compute reprojection error 
function [err] = error_reprojection(X, P1, P2, x1, x2)
% % err should be an array including all reprojection errors 
% % in all observed frames (e.g. 1, 2, 3, ..., n)
% 
% .......
% 
Xones = [X ones(size(X,1),1)];
err_1 = x1(:,1:2) - transpose((P1(1:2,:)* Xones')./(P1(3,:)* Xones'));
err_2 = x2(:,1:2) - transpose((P2(1:2,:)* Xones')./(P2(3,:)* Xones'));
err = [err_1, err_2];
end