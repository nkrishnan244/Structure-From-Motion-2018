clear; clc; close all;

%% for all possible pairs of images
Mu = cell(1,5);
Mv = cell(1,5);
V = cell(1,5);
RGB = cell(1,5);
Xcloud = [];
rgbCloud = [];

[Mu{1}, Mv{1}, V{1}, RGB{1}] = createMatrix('Matching1.xlsx', 1);
[Mu{2}, Mv{2}, V{2}, RGB{2}] = createMatrix('Matching2.xlsx', 2);
[Mu{3}, Mv{3}, V{3}, RGB{3}] = createMatrix('Matching3.xlsx', 3);
[Mu{4}, Mv{4}, V{4}, RGB{4}] = createMatrix('Matching4.xlsx', 4);
[Mu{5}, Mv{5}, V{5}, RGB{5}] = createMatrix('Matching5.xlsx', 5);

%% Define X1 and X2, as well as current variables
x1 = [];
x2 = [];
x1vals = cell(1,5); %1,2,3,4,5
x2vals = cell(1,5); %2,3,4,5,6
RGBreduced = cell(1,5); %1,2,3,4,5

%% This for loop creates all the correspondences for the first half of the code ie. 1-2, 2-3, 3-4 etc
for i = 1:5
    currX1 = [];
    currX2 = [];
    currRGB = [];
    Muc = Mu{i};
    Mvc = Mv{i};
    RGBc = RGB{i};
    Vc = V{i};
    for j = 1:size(Vc,1)
        if Vc(j,i+1) == 1 && Vc(j,i+1) == 1
            currX1 = [currX1; Muc(j,i) Mvc(j,i)];
            currX2 = [currX2; Muc(j,i+1) Mvc(j,i+1)];
            currRGB = [currRGB; RGBc(j,:)];
        end
    end
    x1vals{i} = currX1;
    x2vals{i} = currX2;
    RGBreduced{i} = currRGB; 
end
