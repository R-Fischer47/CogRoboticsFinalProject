
%Read the bottle object
%used Bottle_Object7
ptCloud = pcread('Knife_Object0.pcd');

%Transformation of the object

%X-axis Rotation
thetaX = pi/4;

%Y-axis Rotation
thetaY = pi/4;

%Z-axis Rotation
thetaZ = pi/4;
rotX = [1 0 0; ...
    0 cos(thetaX) -sin(thetaX); ...
    0 sin(thetaX) cos(thetaX)];
           
rotY = [cos(thetaY) 0 sin(thetaY); ...
    0 1 0; ...
    -sin(thetaY) 0 cos(thetaY)];
rotZ = [cos(thetaZ) sin(thetaZ) 0; ...
      -sin(thetaZ) cos(thetaZ) 0; ...
      0 0 1];
trans = [0, 0, 0];
rot = rotX * rotY * rotZ;
tform = rigid3d(rot,trans);
ptCloudTrans = pctransform(ptCloud,tform);
subplot(1,3,1)
pcshow(ptCloud)
title('Original')
subplot(1,3,2)
pcshow(ptCloudTrans)
title('Transformed')

%add surfacenormals to the pointcloud
ptCloudTrans.Normal = pcnormals(ptCloudTrans);
pcwrite(ptCloudTrans,"Knife_Object0_with_normal_XYZ.pcd",'Encoding','ascii')
