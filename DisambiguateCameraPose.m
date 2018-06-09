function [C, R, X0, finalNegativeFilter] = DisambiguateCameraPose(Cset, Rset, Xset)

% Define standard camera 1 position
C1 = [0; 0; 0]; 
R1 = eye(3);
prevSatCounter = 0;

for i = 1:4 % Go through all camera poses
    negativeFilter = zeros(size(Xset,1),1);
    C2 = Cset(:,:,i);
    R2 = Rset(:,:,i);
    X2 = Xset(:,:,i);
    satCounter = 0;
    % Go through all poins within a camera pose
    for j = 1:size(Xset,1)
        X = X2(j,:);
        if R2(3,:)*(X'-C2) > 0 && R2(3,:)*(X'-C1) > 0
            satCounter = satCounter + 1;
            negativeFilter(j) = 1;
        end
    end
    if satCounter > prevSatCounter
        C = C2;
        R = R2;
        X0 = X2;
        finalNegativeFilter = negativeFilter;
        prevSatCounter = satCounter;
    end
end

end