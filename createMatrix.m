%% encode data function
function [Mu, Mv, v, rgb] = createMatrix(FileName, image1)
%filename, image_idx, nImages
matching1Matrix = xlsread(FileName);
% Your Code Goes Here
N = size(matching1Matrix,1);

matching1Matrix(isnan(matching1Matrix)) = 0;
rgb = matching1Matrix(:,2:4);

Mu = zeros(N,6);
Mv = Mu(:,:);
v = Mu(:,:);

for i = 1:N
    v(i, image1) = 1;
    Mu(i, image1) = matching1Matrix(i,5);
    Mv(i, image1) = matching1Matrix(i,6);
    for j = 1:(matching1Matrix(i,1)-1)
        Mu(i,matching1Matrix(i,(7+3*(j-1)))) = matching1Matrix(i,(8+3*(j-1)));   
        Mv(i,matching1Matrix(i,(7+3*(j-1)))) = matching1Matrix(i,(9+3*(j-1)));    
        v(i,matching1Matrix(i,7+3*(j-1))) = 1;
    end
end

end




