

# 6D Pose Estimation of Objects Using the GOOD descriptor

This repository is the work of Richard Fischer, Bharath Raj, Ruben Spolmink and Bharath Musaudhi Rajan for the course Cognitive Robotics (WMAI003-05.2020-2021.1A) at the University of Groningen.

In this projec twe  try  to  enhance  an  existing  object  recognizer  algorithm  with an additional feature of orientation validation. Our idea for this implementation  is  to  couple  a  voting-based  point  pair  feature(PPF) pose estimation algorithm (Drost et al. 2010) with a state of the art object descriptor - Global orthographic object descriptor(GOOD)  (Kasaei  et  al.  2016)


## Prerequisites to run the model
  - Make sure you have a base point cloud of an object for reference (our default base model is "Bottle_Object7.pcd")
  - Make sure you have Matlab2019a or later version with the packages (Image processing toolbox, Communication toolbox)
  
## Steps to run the model
  - Run "src/rigid3dTransformation.m" 
    Here we read the "Bottle_Object7.pcd" and transform it in the X,Y and(or) Z axis rotations. The output image serves as the input to the second script.

  - Run "src/PoseEstimation.m"
    Here we take the base object "Bottle_Object7.pcd" and the reference object as inputs. The model calculates the pose transformation of the second object compared to the base object. The output is displayed in a windowed fashion
