Nikhil Krishnan
CIS 580
5/08/2018

In order to run the code, run "finalScript.m". A plot of the 3D point cloud should appear. Essentially, the code does the following:

  1) Find the fundamental matrix, and thus, the essential matrix, that maps two images onto each other. 
  2) Use linear triangulation to find the location of points shared by the two images in the 3D cloud. 
  3) Use these same points and find the overlap with other images to find a transformation matrix that directly maps these new points 
     onto the 3D point cloud.
  4) Use non-linear triangulation for the other cameras to add points.
  5) Repeat these steps for each pair of images. 
  
  The original images can be found as well and the final report is under "WriteUp.pdf"
