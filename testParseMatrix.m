matching1Matrix = importdata('data/matching1.txt',' ', 1);
% Your Code Goes Here


matching1Matrix.data(isnan(matching1Matrix.data)) = 0;
rgb = matching1Matrix.data(:,2:4);

Mu = zeros(size(matching1Matrix.data,1),3);
Mv = Mu(:,:);
v = Mv(:,1:2);

for i = 1:size(Mu,1)
    Mu(i,1) = matching1Matrix.data(i,5);
    Mu(i,2) = matching1Matrix.data(i,8);
    Mu(i,3) = matching1Matrix.data(i,11);
    
    Mv(i,1) = matching1Matrix.data(i,6);
    Mv(i,2) = matching1Matrix.data(i,9);
    Mv(i,3) = matching1Matrix.data(i,12);
    
    v(i,1) = matching1Matrix.data(i,7);
    v(i,2) = matching1Matrix.data(i,10);
end


