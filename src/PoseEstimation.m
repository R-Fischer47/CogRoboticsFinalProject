%This is een matlab implemantation of 3d pose estimation using pointclouds
%A pointcloud is loaded in and after that the pointcloud is subsampeled to
%reduce the resolution, since the gathered images have a too high
%resolution. For each of the point pair it's features are calculated. Given
%by the ecluadian distance of the vector between the two points, the angle
%of the vector with the first normal, the angle with the second normal and
%the angle of the two normals with each other.

hashmap = containers.Map;
samplePercentage = 0.75;
pbOriginal = pcread('Bottle_Object7.pcd');
pb = pcdownsample(pbOriginal,'random',samplePercentage);
N = pb.Count;
pb.Normal = pcnormals(pb);

%This is the partition and codebook for the quantisation of the distance.
%The partition array indicates at which regions we want to split the
%values, the codebook tells the program which value to put instead of the
%original value for each split. 
partitionD = zeros(1,500);
codebookD = zeros(1,501);
index = 1;
for i = 0:0.02:10
    partitionD(index) = i;
    codebookD(index) = i;
    index = index+1;
end 
codebookD(index) = 10.02;

%Quantisation of angles in degrees. Since the authors of the
%state-of-the-art paper used 10 degrees, we wanted to retain consistency.
partitionA = zeros(1,37);
codebookA = zeros(1,38);
index = 1;
for i = 0:10:360
    partitionA(index) = i;
    codebookA(index) = i;
    index = index+1;
end 
codebookA(index) = 360;

for i = 1:N
    pi = pb.Location(i,:);
    pin = pb.Normal(i,:);
    for j =1:N
        if i~=j
            pj = pb.Location(j,:);
            pjn = pb.Normal(j,:);
            d = pj-pi;
            dist = sqrt(sum((pi - pj).^ 2)); %the ecluadiant distance
            [indexQd, distQ] = quantiz(dist, partitionD, codebookD); %returns distQ: the quantized distance between pi and pj (using codebookD and partitionD)
            angle1 = rad2deg(acos(dot(pin,d) /(norm(pin)*norm(d)))); %the angle between the vector and first normal
            [indexQa1, angleQ1] = quantiz(angle1, partitionA, codebookA);%Quantisation of angle based on codebookA and partitionA
            angleQ1 = deg2rad(angleQ1);
            angle2 = rad2deg(acos(dot(pjn,d) /(norm(pjn)*norm(d)))); %angle between the vector and second normal
            [indexQa2, angleQ2] = quantiz(angle2, partitionA, codebookA);%Quantisation of angle based on codebookA and partitionA
            angleQ2 = deg2rad(angleQ2);%retransform angle to radians
            angle3 = acos(dot(pin,pjn) /(norm(pin)*norm(pjn))); % angle between the two normals
            [indexQa3, angleQ3] = quantiz(real(angle3), partitionA, codebookA);%Quantisation of angle based on codebookA and partitionA
            angleQ3 = deg2rad(angleQ3);
            vector = [dist,angleQ1,angleQ2,angleQ3]; %this is combined as a point pair feature vector
            I = strjoin(string(vector)); %transformed to an index for a hash table
            alpha = atan(-d(3)/d(2)); % the angle is calculated of the point pair with the xy plane
            alpha(isnan(alpha))=0;
            hashmap(I) = [alpha,i]; %the angle is stored under the unique point pair feature
        end
    end
end
%Here we load in the scene object. This is the object that is being
%compared to the first object to determine the pose transformation.
A = containers.Map;
pbTransformed = pcread('Bottle_Object_with_normal_XYZ.pcd');
pb2 = pcdownsample(pbTransformed,'random',samplePercentage);
Ns = pb2.Count;
poselist = containers.Map;
pb2.Normal = pcnormals(pb2);

%Here we initiate the partition and codebook for the quantization of the
%transformation matrix for pose estimation (this is a deviation of the original pseudocode)
partitionT = zeros(1,100);
codebookT = zeros(1,101);
index = 1;
for i = -1:0.01:1
    partitionT(index) = i;
    codebookT(index) = i;
    index = index+1;
end 
codebookT(index) = 1.02;
tic
%The same point pair features are calculated for the second object and
%checked in the hashmap if we have a match
for h = 1:Ns
    i = randi([1 Ns],1,1);
    pi = pb2.Location(i,:);
    pin = pb2.Normal(i,:);
    for j = 1:Ns
        if i~=j
            pj = pb2.Location(j,:);
            pjn = pb2.Normal(j,:);
            d = pj - pi; %the vector between the two points
            dist = sqrt(sum((pi - pj).^ 2)); %the ecludian distance
            [indexQd, distQ] = quantiz(dist, partitionD, codebookD);
            angle1 = rad2deg(acos(dot(pin,d) /(norm(pin)*norm(d)))); %the angle between the vector and the first normal
            [indexQa1, angleQ1] = quantiz(angle1, partitionA, codebookA);
            angleQ1 = deg2rad(angleQ1);
            angle2 = rad2deg(acos(dot(pjn,d) /(norm(pjn)*norm(d)))); %the angle between the vector and the second normal
            [indexQa2, angleQ2] = quantiz(angle2, partitionA, codebookA);
            angleQ2 = deg2rad(angleQ2);
            angle3 = acos(dot(pin,pjn) /(norm(pin)*norm(pjn))); %the angle between the two normals
            [indexQa3, angleQ3] = quantiz(real(angle3), partitionA, codebookA);
            angleQ3 = deg2rad(angleQ3);
            vector = [dist,angleQ1,angleQ2,angleQ3]; %the resulting point pair feature vector
            I = strjoin(string(vector)); %transformed to be an index for the hashmap
            if isKey(hashmap,I) %if there is a case where we have a match
                output = hashmap(I);
                alphaS = atan(-d(3)/d(2)); 
                [indexQalphaS, alphaS] = quantiz(alphaS, partitionA, codebookA);
                alphaS(isnan(alphaS))=0;
                alpha = output(1) - alphaS; %the difference between the two angles between the vector and xy plane is calculated
                Ialpha = alpha;
                [indexQalphaI, Ialpha] = quantiz(Ialpha, partitionA, codebookA);
                Inew = [output(2),Ialpha];
                Istring = strjoin(string(Inew));
                if isKey(A,Istring) %each matching point pair feature casts a vote the angle it found
                    A(Istring) = A(Istring)+1;
                else
                    A(Istring) = 1;
                end
            end
        end        
    end
    %for each of the points we find the highest angles. and calculate the
    %resulting transformation matrix for the pose
    keySet = A.keys; %the keys of A are retrieved. They represent point pair features
    valueSet = string(values(A,keySet));%retrieve keys of A. They represent the vote for each feature
    [sortedValues, sortIdx] = sort( valueSet, 'descend' );%sort votes descending and output the sorting index sortIdx
    sortedKeys = keySet( sortIdx );%sort the keys according to sortIdx to retain the key-value format
    sortedValues = str2double(sortedValues);
    numerical = split(sortedKeys(1)); %we transform the unique index of the hashtable back to usable data
    alpha = str2double(numerical(2));
    Mindex = str2double(numerical(1));
    %calculate the transformatrix from the objects to a generic pose coordinate
    %system
    %to calculate this transform we need to align the normal of the point
    %with the x-axis and set the point to the origin. To align the vector
    %with the normal we first need to lay it flay on the xy plane, so we
    %get the angle when z is 0. When we get the angle we need to rotate it
    %around the z axis to align it with the x-axis. Therefore we need to
    %put y to 0.
    piS = pb2.Location(i,:);
    normalS = pb2.Normal(i,:);
    piM = pb.Location(Mindex,:);
    normalM = pb.Normal(Mindex,:);
    alphaM = atan(-normalM(3)/normalM(2)); %calculate the angle between the normal and xy plane
    omegaM = atan(-(cos(alphaM)*normalM(2) - sin(alphaM)*normalM(3))/normalM(1)); %calculate the angle between the normal and the x-axis
    alphaS = atan(-normalS(3)/normalS(2)); %%calculate the angle between the normal and xy plane
    alphaS(isnan(alphaS))=0;
    omegaS = atan(-(cos(alphaS)*normalS(2) - sin(alphaS)*normalS(3))/normalS(1));%calculate the angle between the normal and the x-axis
    rotationS = [cos(alphaS), -sin(alphaS)*cos(omegaS), sin(alphaS)*sin(omegaS); sin(alphaS), cos(alphaS)*cos(omegaS), -cos(alphaS)*sin(omegaS); 0, sin(omegaS), cos(omegaS)];%calculate the rotation matrix based on the angles calculated before hand
    rotationM = [cos(alphaM), -sin(alphaM)*cos(omegaM), sin(alphaM)*sin(omegaM); sin(alphaM), cos(alphaM)*cos(omegaM), -cos(alphaM)*sin(omegaM); 0, sin(omegaM), cos(omegaM)];%calculate the rotation matrix based on the angles calculated before hand
    Rmatrix = [1,0,0;0,cos(alpha),-sin(alpha);0,sin(alpha),cos(alpha)]; %the rotation matrix for the x-axis calcluted by the angle difference between the point pair features of hash object and the scene object
    transformS = zeros(4); %generate the transform matrix
    transformM = zeros(4);
    transformR = zeros(4);
    %fill in the rotation matrix for each transform
    for i = 1:3
        for j = 1:3
            transformS(i,j) = rotationS(i,j);
            transformM(i,j) = rotationM(i,j);
            transformR(i,j) = Rmatrix(i,j);
        end
    end
    %add the negative point to place the point at the origin
    for i = 1:3
        transformS(i,4) = -piS(i);
        transformM(i,4) = -piM(i);
    end
    %by convention the last row is given by (0,0,0,1)
    transformS(4,4) = 1;
    transformM(4,4) = 1;
    transformR(4,4) = 1;
    pose = pinv(transformS)*transformR*transformM; %calculate the final transformation matrix for the original frame
    %there is some small noise error for the translation, but since objects
    %are only rotated this can be just discared without information loss
    for i = 1:3
        pose(i,4) = 0;
    end
    poseQ = zeros(4,4);
    %Below, every retrieved transformation matrix is quantized to allow the
    %voting procedure for finding the best pose
    for i = 1:4
        for j = 1:4
            [indexQa1, poseQ(j,i)] = quantiz(pose(j,i), partitionT, codebookT);
        end
    end
    % Below is the voting procedure that checks if the transformation matrix
    % under scrutiny is already in the map container 'poselist'. If it is, 
    % it adds a vote, if not it adds the first vote.
    I = strjoin(string(poseQ));
    if isKey(poselist,I)
        poselist(I) = poselist(I)+1;
    else
        poselist(I) = 1;
    end
end

keySet = poselist.keys;%the keys of poselist are retrieved. They represent point pair features
valueSet = string(values(poselist,keySet));%retrieve keys of poselist. They represent the vote for each feature
[sortedValues, sortIdx] = sort( valueSet, 'descend' );%sort votes descending and output the sorting index sortIdx
sortedKeys = keySet( sortIdx );%sort the keys according to sortIdx to retain the key-value format
bestT = str2double(split(sortedKeys(1))); %we transform the unique index of the hashtable back to usable data
T = reshape(bestT,[4,4])';%reshape the transformation matrix values to a 4x4 Matrix

for i = 1:3
   T(i,4) = 0;
end
T(4,4) = 1;


trans = [0, 0, 0];
rot =  zeros(3);
for i = 1:3
    for j = 1:3
        rot(i,j) = T(i,j);
    end
end

tform = affine3d(T);
ptCloudTrans = pctransform(pbOriginal,tform);
subplot(1,3,1)
pcshow(pbOriginal)
title('Original')
subplot(1,3,2)
pcshow(pbTransformed)
title('Transformed')
subplot(1,3,3)
pcshow(ptCloudTrans)
title('estimated pose')

toc
