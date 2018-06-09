function [Cnew, Rnew, S] = PnRANSAC(X, x, K)
iterations = 3000;
threshold = 25;
N = size(x,1);
n = 0;
S = [];
for i = 1:iterations
    indexMatrix = zeros(N,1);
    %Randomly sample 6 points of data
    sampleIndex = datasample(1:N,6);
    XRand = X(sampleIndex,:);
    xRand = x(sampleIndex,:);
    [C R] = LinearPnP(XRand, xRand, K);
    P = K*R*[eye(3) -C];
    for j = 1:size(x,1)
        x_curr = [x(j,:) 1];
        X_curr = [X(j,:) 1];
        new_x = P*X_curr';
        new_x = new_x/new_x(3);
        pointError = norm(x_curr - new_x');
        if pointError < threshold
            indexMatrix(j) = 1;
        end
    end
    if sum(indexMatrix) > n
        n = sum(indexMatrix);
        S = indexMatrix;
        Cnew = C;
        Rnew = R;
    end
    
end