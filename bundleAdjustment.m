function [Cset_refine, Rset_refine, Xset_refine] = bundleAdjustment(Cset, Rset, X, K, traj, V)
%% construct parameters for optimization function
% e.g. for BA, the parameter should be a vector contains camera and 3D points information
% training parameter's size should be (7 x number of cameras) + (4 x number of 3D points)
num_cameras = size(Cset,3);
num_3Dpoint = size(X,1);
%% Encode the input arguments into a parameter vector for the optimization
% params0 = zeros(1, 7 * num_camera + 3 * num_points);

Cpoints = size(Cset,1)*size(Cset,2)*size(Cset,3);
Rpoints = size(Rset,1)*size(Rset,2)*size(Rset,3);
Xpoints = size(X,1)*size(X,2)*size(X,3);
params0 = zeros(1,(Cpoints + Rpoints + Xpoints));

params0(1:Xpoints) = X(:);
params0((Xpoints + 1):(Cpoints + Xpoints)) = Cset(:);
params0((Cpoints + Xpoints + 1):end) = Rset(:);
% ........
% 
% 
% %% Construct Jacobian Matrix
[Jaco] = createJacobianBA(params0);
% 
% 
%% Define an optimizer (feel free to change below parameters)
opts = optimoptions(@fsolve, 'Display', 'iter', 'JacobPattern', Jaco, 'Algorithm', ...
             'trust-region-reflective', 'PrecondBandWidth', 1 , 'MaxIter', 60);
% 
% %% Objective function (e.g. reprojection error)
f = @(params)err_reproject_BA(params, K, V);   % the params should have the same format as params0 you defined
% 
% %% Solver
params_opt = fsolve(f, params0, opts);
% 
% %% Decode parameters into rotation, translation and 3D point cloud so on
% ........
% 
Cset_refine = reshape(params_opt((Xpoints + 1):(Cpoints + Xpoints)),size(Cset,1),size(Cset,2),size(Cset,3));
Rset_refine = reshape(params_opt((Xpoints + 1):(Cpoints + Xpoints)),size(Cset,1),size(Cset,2),size(Cset,3));
Xset_refine = reshape(params_opt(1:Xpoints),size(X,1),size(X,2));
% 
% end
% 
% 
 %% function to compute reprojection error
function [err] = err_reproject_BA(params, K, V)
Xvals = reshape(params(1:Xpoints),size(X,1),size(X,2));
Cvals = reshape(params((Xpoints + 1):(Cpoints + Xpoints)),size(Cset,1),size(Cset,2),size(Cset,3));
Rvals = reshape(params((Cpoints + Xpoints + 1):end),size(Rset,1),size(Rset,2),size(Rset,3));
err = [];
% 
 for i = 1 : num_3Dpoint
    Xc = Xvals(i,:);
    v = traj(i,:);
   for j = 1 : num_cameras
    R = Rset(:,:,j);
    C = Cset(:,:,j);
    P = K*R*[eye(3) -C];
    calculated_val = P*transpose([Xc 1]); 
    new_error = norm(v - calculated_val/calculated_val(3));
    err = [err, new_error];
   end
% 
 end

end

end


%% Jacobian construction
function [J] = createJacobianBA(params0)
J = sparse(params0);
% % use the sparse matrix to build the Jacobian
% % reference link: https://www.mathworks.com/help/matlab/ref/sparse.html
% 
% .......
% 
end
