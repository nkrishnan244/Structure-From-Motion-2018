function [Cset, Rset] = ExtractCameraPose(E)

[U,S,V] = svd(E);
W = [0 -1 0; 1 0 0; 0 0 1];

t1 = U(:,3);
t2 = -U(:,3);
t3 = U(:,3);
t4 = -U(:,3);

R1 = U*W*V';
R2 = U*W*V';
R3 = U*W'*V';
R4 = U*W'*V'; 

t = [t1 t2 t3 t4];
Rset = zeros(3,3,4);
Rset(:,:,1) = R1;
Rset(:,:,2) = R2;
Rset(:,:,3) = R3;
Rset(:,:,4) = R4;

Cset = zeros(3,1,4);
C1 = -R1'*t1;
C2 = -R2'*t2;
C3 = -R3'*t3;
C4 = -R4'*t4;

Cset(:,:,1) = C1;
Cset(:,:,2) = C2;
Cset(:,:,3) = C3;
Cset(:,:,4) = C4;

for j = 1:4
    if det(Rset(:,:,j)) < -0.9 && det(Rset(:,:,j)) > -1.1
        Cset(:,:,j) = -Cset(:,:,j);
        Rset(:,:,j) = -Rset(:,:,j);
    end
end

end
% (INPUT) E: essential matrix
% (OUTPUT) Cset and Rset: four configurations of camera centers and rotations, i.e., Cset{i}=Ci
% and Rset{i}=Ri
% 
% 1. t1 = U(:, 3) and R1 = UWVT
% 2. t2 = ?U(:, 3) and R2 = UWVT
% 3. t3 = U(:, 3) and R3 = UWTVT
% 4. t4 = ?U(:, 3) and R4 = UWTVT.