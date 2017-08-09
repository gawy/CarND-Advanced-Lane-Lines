# Advanced Lane Finding Project


## Camera calibration
The very first step for camera calibration is to find chessboard corners at every image.
During the experiment it showed that OpenCV is not able to detect corners on every image in the test set. 
Images [1, 4, 5] were left undetected. Most likely cause is because of the lack of white border around the chessboard.
This is a requirement from openCV.

Other images were plotted with respective corners drawn on them.
[corner detection result](https://www.dropbox.com/s/cwtdkqmr5uypwfr/Screenshot%202017-07-31%2011.42.21.png?dl=0)

Those unrecognized images came in handy to validate undistortion process (like the validation set in DL process). 
[undist results](https://www.dropbox.com/s/8fvh8ipm9l42bc4/Screenshot%202017-07-31%2013.03.29.png?dl=0)


## Thresholding
 
Values for various thresholds were selected as a result of experiments where
thresholds were iterated like range(10, 150, 10) and then compared visually

[Example of different threshold values](https://www.dropbox.com/s/0a74ug230mwj3qg/Screenshot%202017-08-05%2013.33.33.png?dl=0)

One of the interesting parts - removing shadows.
S channel of HLS color schema gives good results in detecting colors other than white. Which is good for yellow.
But it is not very good at filtering shadows. L channel came handy. Shadows in L channel can be easily removed with 
low threshold values (5, 100)
[Before L channel filtering](https://www.dropbox.com/s/izj9i52x7wb82au/Screenshot%202017-08-05%2011.07.59.png?dl=0)
[After L channel filtering](https://www.dropbox.com/s/kfrdy1szix31k0a/Screenshot%202017-08-05%2011.07.32.png?dl=0)

## Image transformations
Warping to top-down view

Important aspect: how to select proper source and destination coordinates for transformation matrix 
in order to preserve scale and so avoid measurement errors.
Solution: this does not actually seem to be problem as we are dealing with just a vertical axis 
and if scaling or aspect ration is not preserved well - that would not influence correlation
between pixel and real units (meters/miles).
