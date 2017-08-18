# Advanced Lane Finding Project

## Change log

V3: Fixed the way how resulting mask is applied in a pipeline. Now undistorted image is used.

V2: For resubmit code was refactored and splitted into several files.
Main code extracted to lane_finder.py. It contains single class that combines all needed methods for
lane detection.


## Camera calibration
The very first step for camera calibration is to find chessboard corners at every image.
During the experiment it showed that OpenCV is not able to detect corners on every image in the test set. 
Images [1, 4, 5] were left undetected. Most likely cause is because of the lack of white border around the chessboard.
This is a requirement from openCV.

Other images were plotted with respective corners drawn on them.
[corner detection result](https://www.dropbox.com/s/cwtdkqmr5uypwfr/Screenshot%202017-07-31%2011.42.21.png?dl=0)

Those unrecognized images came in handy to validate undistortion process (like the validation set in DL process). 
[undist results](https://www.dropbox.com/s/8fvh8ipm9l42bc4/Screenshot%202017-07-31%2013.03.29.png?dl=0)


# Pipeline

## 1. Provide an example of a distortion-corrected image. 
[Chessboard showing results of distortion correction](https://www.dropbox.com/s/z2avmxp60qbq7qz/Screenshot%202017-08-09%2023.10.57.png?dl=0)
In code use `calibrate()` to obtain distortion correction parameters 
and then later on `undistort_transform()` function uses those parameters to perform transformation in video and images.


## 2. Thresholding
 
Values for various thresholds were selected as a result of experiments where
thresholds were iterated like range(10, 150, 10) and then compared visually

[Example of different threshold values](https://www.dropbox.com/s/0a74ug230mwj3qg/Screenshot%202017-08-05%2013.33.33.png?dl=0)

Thresholds applied:
* x gradient 
* gradient magnitude
* gradient direction
* color thresholding by S and H channels in HLS color schema


One of the interesting parts - removing shadows.
S channel of HLS color schema gives good results in detecting colors other than white. Which is good for yellow.
But it is not very good at filtering shadows. L channel came handy. Shadows in L channel can be easily removed with 
low threshold values (5, 100)
[Before L channel filtering](https://www.dropbox.com/s/izj9i52x7wb82au/Screenshot%202017-08-05%2011.07.59.png?dl=0)
[After L channel filtering](https://www.dropbox.com/s/kfrdy1szix31k0a/Screenshot%202017-08-05%2011.07.32.png?dl=0)

In code, see `processImage()` function

## Image transformations
Warping to top-down view

[Example of warped image](https://www.dropbox.com/s/ly0bo7o9swbuz5e/Screenshot%202017-08-09%2023.23.58.png?dl=0)

Important aspect: how to select proper source and destination coordinates for transformation matrix 
in order to preserve scale and so avoid measurement errors.
Solution: had to experiment with y coordinates of the destination in order to make sure that size of dashed lane 
markings are equal along the whole picture.

`getTransformationMatrix()` function has all the code to obtain warp matrix.

Points used as source: (704, 460), (580, 460), (273, 672), (1032, 672)
Destination points were created as follows:
```
(src_pt1[3, 0], top_y), (src_pt1[2, 0], top_y), 
(src_pt1[2, 0], src_pt1[3, 1]), (src_pt1[3, 0], src_pt1[3, 1])
```

To verify transformation I draw a rectangle with points and visualy compared to transformed image.



## 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?
 
In code all debug output shown at block with `comb = processImage(tst_img, debug=True)`. This showed all intermediate
results that function was producing.

My method included buidling a [histogram](https://www.dropbox.com/s/soetbpl3zk8iu03/Screenshot%202017-08-09%2023.33.18.png?dl=0)
of bottom part of the image to identify initial points for lane markings.
Then I used sliding window method to filter out pixels that most likely belong to lane marks and not to some other
objects on the road or image overall. This created to regions of pixes on the left and right. 
[Debug image](https://www.dropbox.com/s/6n9ew70om41bbgu/Screenshot%202017-08-09%2023.33.45.png?dl=0)

## 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center. 

All lane related code is concentrated in `def findLaneLines(bin_image, debug=False)`

Last part of that function marked as `# Curvature radius` has logic to calculate radius of the curvature.
I've measure amount of pixels on the image that correspond to lane width and took the value for lane length from 
course material (to simplify measurement - had issues with real image).
That was a basis for px to meters conversion.
Then I fitted a polynom using px-to-m scaling. After that just used a formula to calculate curvature based on known
 polynomial coefficients.
 
## 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly. 

[Mask and resulting image](https://www.dropbox.com/s/mwfbk5wkw1sev39/Screenshot%202017-08-09%2023.40.55.png?dl=0)


# Pipeline video

[v3 video](https://youtu.be/bQdpb8ASjOE)

[v2 updated] (https://youtu.be/-OMvELCcWvk)

[v1 Link - had issues](https://youtu.be/I0SjT4m12Xo)

# Discussion
 
1. Problem 1. Case that did not have a reliable answer is how to define transformation matrix for top-down view. 
Doing it visually is not a very reliable method

2. State. What I planned to do but did not get to was to wrap the pipeline code into a class. That would provide 
a good container to save state in between frame processing and allow for transparent logic of smoothing


