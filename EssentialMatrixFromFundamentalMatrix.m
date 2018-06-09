function E = EssentialMatrixFromFundamentalMatrix(F, K)
    [U,S,V] = svd(K'*F*K);
    S = [1 0 0; 0 1 0; 0 0 0 ];
    E = U*S*V';
end

% (INPUT) K: 3×3 camera intrinsic parameter
% (INPUT) F: fundamental matrix
% (OUTPUT) E: 3×3 essential matrix with singular values (1,1,0)