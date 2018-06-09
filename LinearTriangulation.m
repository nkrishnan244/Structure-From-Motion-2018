function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)

% (INPUT) C1 and R1: the first camera pose
% (INPUT) C2 and R2: the second camera pose
% (INPUT) x1 and x2: two N ×2 matrices whose row represents correspondence between the first
% and second images where N is the number of correspondences.
% (OUTPUT) X: N × 3 matrix whose row represents 3D triangulated point.
N = size(x1,1);
I = eye(size(C1,1));
P1 = K*R1*[I -C1];
P2 = K*R2*[I -C2];
X = zeros(N,3);
for i = 1:N
    x1Local = [x1(i,:) 1];
    x2Local = [x2(i,:) 1];
    skew1 = Vec2Skew(x1Local);
    skew2 = Vec2Skew(x2Local);
    
    A = [skew1*P1; skew2*P2];
    [U,D,V] = svd(A);
    xVect = V(:,end)/V(end,end); 
    X(i,:) = xVect(1:3);
end


end

function skew = Vec2Skew(v)
skew = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end