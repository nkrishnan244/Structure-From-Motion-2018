clear; clc; close all;

%% for all possible pairs of images
[Mu, Mv, V, RGB] = createMatrix('Matching1.xlsx');
[Mu2, Mv2, V2, RGB2] = createMatrix('Matching2.xlsx');
[Mu3, Mv3, V3, RGB3] = createMatrix('Matching3.xlsx');
[Mu4, Mv4, V4, RGB4] = createMatrix('Matching4.xlsx');
[Mu5, Mv5, V5, RGB5] = createMatrix('Matching5.xlsx');

%% Define X1 and X2
x1 = [];
x2 = [];
image1 = 1;
image2 = 2;
image3 = 3;
image4 = 4;
image5 = 5;
image6 = 6;

for i = 1:size(V,1)
    if V(i,image1) == 1 && V(i,image2) == 1
        x1 = [x1; Mu(i,image1) Mv(i,image1)];
        x2 = [x2; Mu(i,image2) Mv(i,image2)];
    end
end

%% 1.1.2 Match Outlier Rejection via RANSAC
[y1, y2, idx, F] = GetInliersRANSAC(x1,x2);
rgbReal = RGB(idx);

%% 1.2.1 Essential Matrix Estimation
K = [568.996140852 0 643.21055941;
    0 568.988362396 477.982801038;
    0 0 1];
E = EssentialMatrixFromFundamentalMatrix(F, K);

%% 1.2.2 Camera Pose Extraction
[Cset, Rset] = ExtractCameraPose(E);

%% 1.3.1 Linear Triangulation
Xset = zeros(size(y1,1),3,4);
C1 = [0; 0; 0];
R1 = eye(3);
for i = 1:4
    C2 = Cset(:,:,i);
    R2 = Rset(:,:,i);
    X = LinearTriangulation(K, C1, R1, C2, R2, y1, y2);
    Xset(:,:,i) = X;
end

%% 1.3.2 Camera Pose Disambiguation
[C, R, X0, negativeFilter] = DisambiguateCameraPose(Cset, Rset, Xset);

%% Filter Out All Points behind camera
X0 = X0(negativeFilter==1,:);
y1 = y1(negativeFilter==1,:);
y2 = y2(negativeFilter==1,:);
rgbReal = rgbReal(negativeFilter==1,:);

cameraMat = zeros(3,1,2);
rotationMat = zeros(3,3,2);

%Put the camera in the proper format for plotting
cameraMat(:,:,1) = C1;
cameraMat(:,:,2) = C;

rotationMat(:,:,1) = R1;
rotationMat(:,:,2) = R;

%% Nonlinear Triangulation
X = nonlinearFunc(K, zeros(3,1), eye(3), C, R, y1, y2, X0);

%% Filter Very Large Points
outlierDetect = zeros(size(X,1),1);
threshold = 25; % Messed Around with
for i = 1:size(X,1)
    if abs(sum(X(i,:))) < threshold
        outlierDetect(i) = 1;
    end
end
outlierDetect = logical(outlierDetect);
X0 = X0(outlierDetect==1,:);
X = X(outlierDetect==1,:);
y1 = y1(outlierDetect==1,:);
y2 = y2(outlierDetect==1,:);
rgbReal = rgbReal(outlierDetect==1,:);

%% Graph Plots
PC3Dshow(X,cameraMat,rotationMat,rgbReal);
figure;
PC3Dshow(X0,cameraMat,rotationMat,rgbReal);
plot_reprojection('data/image0000002.bmp',R,C,K,X0,y2);
figure
plot_reprojection('data/image0000002.bmp',R,C,K,X,y2);

%% Store matches with other images
imageArray = [image3; image4; image5; image6];
x1_match = cell(1,4);
x2_match = cell(1,4);
for j = 1:4
    images = imageArray(j);
    for i = 1:size(V,1)
        if V(i,image1) == 1 && V(i,images) == 1
            x1_match{j} = [x1_match{j}; Mu(i,image1) Mv(i,image1)];
            x2_match{j} = [x2_match{j}; Mu(i,images) Mv(i,images)];
        end
    end
end

storedX = cell(1,2);
y = y1;
%% Register Camera and Add 3d points 
for i = 1:2
    % Find points also in correspondence to 1 and 2
    [~, index_1, index_2] =  intersect(y1, x1_match{i}, 'rows');
    Xboth = X(index_1,:);
    xim_current = x2_match{i};
    x2both = xim_current(index_2,:);
    yboth = y1(index_1,:);
    
    % Run the rest of the loop
    [Cnew, Rnew] = PnRANSAC(Xboth, x2both,  K);
    [Cnew, Rnew] = NonlinearPnP(Xboth, x2both, K, Cnew, Rnew);
    Cset(:,:, i+3) = Cnew;
    Rset(:,:, i+3) = Rnew;
    Xnew = LinearTriangulation(K, zeros(3,1), eye(3), Cnew, Rnew, x1_match{i}, x2_match{i});
    Xnew = nonlinearFunc(K, zeros(3,1), eye(3), Cnew, Rnew, x1_match{i}, x2_match{i}, Xnew);
    X = [X; Xnew];
    [Cset, Rset, X] = BundeAdjustment(Cset, Rset, X, K, traj, V);
end
close all; 

%% Filter Very Large Points
outlierDetect = zeros(size(X,1),1);
threshold = 25; % Messed Around with
for i = 1:size(X,1)
    if abs(sum(X(i,:))) < threshold
        outlierDetect(i) = 1;
    end
end
outlierDetect = logical(outlierDetect);
X = X(outlierDetect==1,:);