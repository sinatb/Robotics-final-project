# Robotics-final-project

This is the code of the final project of Robotics Course at SBU. It consists of 3 main parts : 

+ The Controller
+ The Convolutional Neural Network
+ Image Processing

## The Controller

This Robot uses a PID controller to navigate to the 5 predefined targets. The image processing part required a straight photo to extract the 28 by 28 pixels image for the CNN. For doing this 5 additional points were added so the robot could reach the main points from those additional points with a 90 degrees angle.

## The CNN

The CNN used for this project uses 6 Layerd architecture to classify the images. In the end an accuracy of 98 percent was reached in 75 epochs.

## Image Processing

A 4 level procedure was Done to extract the 28 by 28 pixel images. At first the images were cropped, then the main block that contained the picture box was found using contours, Then a border was added to the image and at last all the remaining background was turned to black and the image resized to 28 by 28. 

