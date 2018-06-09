function [C,R] = LinearPnP(X, x, K)
A = [];

X = [X ones(size(x,1),1)];
x = [x ones(size(x,1),1)];
Xmat = zeros(3,12);
%Find correspondences
for i = 1:size(x,1)
    Xmat(1, 1:4) = X(i,:);
    Xmat(2, 5:8) = X(i,:);
    Xmat(3, 9:12) = X(i,:);
    %Creat matrix to solve for P
    A = [A; Vec2Skew(K\x(i,:)')*Xmat];
end

[U,D,V] = svd(A);
P = transpose(reshape(V(:,end),4,3));
Pinvert = P(:,1:3);
[U,D,V] = svd(Pinvert);
R = U*V';
if det(R) > -1.1 && det(R) < -0.9
    R = -R;
end

C = -R*P(:,4)/D(1,1);

end

function skew = Vec2Skew(v)
skew = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end