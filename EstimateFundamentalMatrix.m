function [F] = EstimateFundamentalMatrix(x1,x2)
N = size(x1,1);
A = zeros(N,9);

for i = 1:N
    x1x = x1(i,1);
    x1y = x1(i,2);
    
    x2x = x2(i,1);
    x2y = x2(i,2);
    
    A(i,:) = [x2x*x1x x2x*x1y x2x x2y*x1x x2y*x1y x2y x1x x1y 1];
end

[U,D,V] = svd(A);
F = reshape(V(:,9), 3, 3)';
[U,D,V] = svd(F);
D(3,3) = 0;
F = U*D*V';
end

