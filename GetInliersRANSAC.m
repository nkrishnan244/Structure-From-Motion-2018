function [y1, y2, idx, F_real] = GetInliersRANSAC(x1, x2)

N = size(x1,1);
threshold = 0.01;
M = 2000;
prevInCounter = 0;
for i = 1:M
    
    %Randomly sample 8 points of data
    sampleIndex = datasample(1:N,8);
    x1Rand = x1(sampleIndex,:);
    x2Rand = x2(sampleIndex,:);
    
    %Initialize Matrices and variables
    y1Temp = [];
    y2Temp = [];
    currentID = zeros(N,1);
    inCounter = 0;
    
    %Find F for sampled points
    F = EstimateFundamentalMatrix(x1Rand, x2Rand);
    for j = 1:N
        %Check Each point to see how well they match against F
        if abs([x2(j,:) 1]*F*[x1(j,:)'; 1]) < threshold
            %If the point matches, store the values
            inCounter = inCounter + 1;
            currentID(j) = 1;
            y1Temp = [y1Temp; x1(j,:)];
            y2Temp = [y2Temp; x2(j,:)];
        end
    end
    
    %If this correspondence works better than the last, keep all matrices
    if inCounter > prevInCounter
        y1 = y1Temp;
        y2 = y2Temp;
        idx = currentID;
        F_real = F;
        prevInCounter = inCounter;
    end
    %Store the counter from the previous loop
end
idx = logical(idx);
end


% (INPUT) x1 and x2: N×2 matrices whose row represents a correspondence.
% (OUTPUT) y1 and y2: Ni×2 matrices whose row represents an inlier correspondence where Ni
% is the number of inliers.
% (OUTPUT) idx: N×1 vector that indicates ID of inlier y1.