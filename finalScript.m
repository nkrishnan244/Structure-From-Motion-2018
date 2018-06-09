init;
disp('Initialization Complete!');
for m = 1:5
    x1 = x1vals{m};
    x2 = x2vals{m};
    RGBloop = RGBreduced{m};
    
    %% 1.1.2 Match Outlier Rejection via RANSAC
    [y1, y2, idx, F] = GetInliersRANSAC(x1,x2);
    rgbReal = RGBloop(idx,:);
    
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
    threshold = 50; % Messed Around with
    for i = 1:size(X,1)
        if norm(X(i,:)) < threshold
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
    %     PC3Dshow(X,cameraMat,rotationMat,rgbReal);
    %     figure;
    %     PC3Dshow(X0,cameraMat,rotationMat,rgbReal);
    %     plot_reprojection('image0000002.bmp',R,C,K,X0,y2);
    %     figure
    %     plot_reprojection('image0000002.bmp',R,C,K,X,y2);
    
    %% Store matches with other images
    imageArray = (m+2):6;
    x1_match = cell(1,5-m); % all the image1 variable matches
    x2_match = cell(1,5-m); % all the image2 variable matches
    rgb_match = cell(1,5-m);
    Vim1 = V{m};
    Muim1 = Mu{m};
    Mvim1 = Mv{m}; % Make sure current image is being used
    RGBim1 = RGB{m}; %RGB of first of matching image (1,2,3,4,5)
    for j = 1:(5-m)
        images = imageArray(j);
        for i = 1:size(Vc,1)
            if Vim1(i,m) == 1 && Vim1(i,images) == 1
                x1_match{j} = [x1_match{j}; Muim1(i,m) Mvim1(i,m)];
                x2_match{j} = [x2_match{j}; Muim1(i,images) Mvim1(i,images)];
                rgb_match{j} = [rgb_match{j}; RGBim1(i,:)];
            end
        end
    end
    
    %% Register Camera and Add 3d points
    for i = 1:(5-m)
        if isempty(x1_match{i})
            continue;
        end
        % Find points also in correspondence to 1 and 2
        [~, index_1, index_2] =  intersect(y1, x1_match{i}, 'rows');
        %RGB of current image being tested
        Xboth = X(index_1,:);
        xim_current = x2_match{i};
        x2both = xim_current(index_2,:);
        % Run the rest of the loop
        [Cnew, Rnew, S] = PnRANSAC(Xboth, x2both,  K);
        S = logical(S);
        Xboth = Xboth(S==1,:);
        x2both = x2both(S==1,:);
        [Cnew, Rnew] = NonlinearPnP(Xboth, x2both, K, Cnew, Rnew);
        Cset(:,:, i+3) = Cnew;
        Rset(:,:, i+3) = Rnew;
        Xnew = LinearTriangulation(K, zeros(3,1), eye(3), Cnew, Rnew, x1_match{i}, x2_match{i});
        Xnew = nonlinearFunc(K, zeros(3,1), eye(3), Cnew, Rnew, x1_match{i}, x2_match{i}, Xnew);
        rgbReal = [rgbReal; rgb_match{i}];
        %[Cset, Rset, Xnew] = bundleAdjustment(Cset, Rset, Xnew, K, x1_match{i}, V{m});
        X = [X; Xnew];
        traj = [y1; x1_match{i}];
    end
    
    %% Filter Very Large Points
    outlierDetect = zeros(size(X,1),1);
    threshold = 100; % Messed Around with
    for i = 1:size(X,1)
        if norm(X(i,:)) < threshold
            outlierDetect(i) = 1;
        end
    end
    outlierDetect = logical(outlierDetect);
    X = X(outlierDetect==1,:);
    rgbReal = rgbReal(outlierDetect==1,:);
    Xcloud = [Xcloud; X];
    rgbCloud = [rgbCloud; rgbReal];
    %     PC3Dshow(Xcloud,C1,R1,rgbCloud);
end

%% Remove Repeat Points
[~, indeX] = unique(Xcloud,'rows');
Xcloud = Xcloud(indeX,:);
rgbCloud = rgbCloud(indeX,:);

%% Remove Points Behind Camera
outlierDetect = zeros(size(Xcloud,1),1);
for i = 1:size(Xcloud,1)
    if Xcloud(i,3) > 0
        outlierDetect(i) = 1;
    end
end
outlierDetect = logical(outlierDetect);
Xcloud = Xcloud(outlierDetect==1,:);
rgbCloud = rgbCloud(outlierDetect == 1,:);


%% Plot Entire Cloud
PC3Dshow(Xcloud,C1,R1,rgbCloud);
