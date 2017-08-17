import matplotlib.pyplot as plt
import cv2
import numpy as np
from line import Line

COLOR_INFO_TEXT = (218, 217, 108)


class LaneFinder:

    def __init__(self, camera_mtx, dist_coef):
        self.camera_mtx = camera_mtx
        self.dist_coef = dist_coef
        self.warp_mtx = self.get_transformation_matrix()

        self.left_line = Line()
        self.right_line = Line()

    def process_image(self, image, debug=False):
        """
        Take the frame of road view and overlay it with detected lane region.

        :param image: RGB image of the car camera
        :param debug: enable debug output

        :returns: initial image overlayed with region of the lane
        """
    #     image = cv2.undistort(image, mtx, dist, None, None)

        # perform perspective transformation
        undist = cv2.undistort(image, self.camera_mtx, self.dist_coef, None, None)
        warped = self.undistort_transform(undist)

        if debug:
            plt.imshow(warped)
            plt.show()

        img = warped
        absx_binary = self.abs_sobel_thresh(img, orient='x', sobel_kernel=3, thresh=(20, 130))
        absy_binary = self.abs_sobel_thresh(img, orient='y', sobel_kernel=3, thresh=(30, 120))
        mag_binary = self.mag_thresh(img, sobel_kernel=3, mag_thresh=(30, 130))
        dir_binary = self.dir_threshold(img, sobel_kernel=9, thresh=(0.7, 1.3))
        hls_binary = self.hls_threshold(img, 's', thresh=(100,240))
        hlsh_binary = self.hls_threshold(img, 'h', thresh=(5,100))

    #     stacked = np.dstack((np.zeros_like(hls_binary), hls_binary, mag_binary))

        combined = np.zeros_like(dir_binary, np.uint8)
        combined[
            ((absx_binary == 1) | ((mag_binary == 1) & (dir_binary == 1)))
                 | ((hls_binary == 1) & (hlsh_binary == 1))
        ] = 1

        if debug:
            plt.imshow(combined, cmap='gray')
            plt.show()


        lane_mask, lr, rr, displacement = self.find_lane_lines(combined, debug=debug)

        res = cv2.addWeighted(undist, 1, lane_mask, 0.8, 0)
        image_text = 'Radius l={:.1f}m, r={:.1f}m'.format(lr, rr)
        res = cv2.putText(res, image_text, (int(res.shape[0] / 5), int(res.shape[1] * 0.05)),
                          cv2.FONT_HERSHEY_PLAIN, 4.0, COLOR_INFO_TEXT, thickness=2)

        txt2 = 'Car is {:.1f}m from center'.format(displacement)
        res = cv2.putText(res, txt2, (int(res.shape[0] / 5), int(res.shape[1] * 0.05 + 50)),
                          cv2.FONT_HERSHEY_PLAIN, 4.0, COLOR_INFO_TEXT, thickness=2)
    #     plot_image_pair(image, path, undist, cmap2='gray')
    #     combined = np.reshape(combined, (combined.shape[0], combined.shape[1], 1))
    #     print(combined.shape)
        return res



    def process_video_frame(self, image):
        """
        Helper function used for debuggin purposes, so if processImage function returns binary mask -
        it can be used for video processin
        """
        return cv2.cvtColor(self.process_image(image) * 255, cv2.COLOR_GRAY2RGB)


    def find_lane_lines(self, bin_image, debug=False):
        """
        Analyzes image binary map for lane lines.
        Input should be a filtered image, so any non-zero pixel treated as valuable information.

        :param bin_image: binary image array to process.
        :returns: binary mask or detected lane region
                  radius of left lane curvature (m)
                  radius of right lane curvature (m)
                  displacement of car relative to center (m)
        """

        nonzero = np.nonzero(bin_image)
        nonzeroy = nonzero[0]
        nonzerox = nonzero[1]

        lane_pts_l = []
        lane_pts_r = []

        if debug:
            out_img = np.dstack((bin_image, bin_image, bin_image)) * 255
        else:
            out_img = None

        WIN_PADDING = 100
        if self.left_line.detected & self.right_line.detected:

            ll = self.left_line.best_fit
            lr = self.right_line.best_fit


            lane_pts_l = (ll[0]*nonzeroy**2 + ll[1]*nonzeroy + ll[2] - WIN_PADDING < nonzerox) \
                         & (nonzerox < ll[0]*nonzeroy**2 + ll[1]*nonzeroy + ll[2] + WIN_PADDING)
            lane_pts_r = (lr[0]*nonzeroy**2 + lr[1]*nonzeroy + lr[2] - WIN_PADDING < nonzerox) \
                         & (nonzerox < lr[0]*nonzeroy**2 + lr[1]*nonzeroy + lr[2] + WIN_PADDING)

        else:
            self.find_poly_with_window(bin_image, lane_pts_l, lane_pts_r, nonzerox, nonzeroy, debug, out_img,
                                       win_padding=WIN_PADDING)
            lane_pts_l = np.concatenate(lane_pts_l)
            lane_pts_r = np.concatenate(lane_pts_r)

        lane_left_x = nonzerox[lane_pts_l]
        lane_left_y = nonzeroy[lane_pts_l]

        lane_left_coef = np.polyfit(lane_left_y, lane_left_x, 2)

        lane_right_x = nonzerox[lane_pts_r]
        lane_right_y = nonzeroy[lane_pts_r]

        lane_right_coef = np.polyfit(lane_right_y, lane_right_x, 2)

        if debug:
            # visualize detected pixels
            ploty = np.linspace(0, out_img.shape[0] - 1, out_img.shape[0])
            plot_lane_left = lane_left_coef[0] * ploty**2 + lane_left_coef[1] * ploty + lane_left_coef[2]
            plot_lane_right = lane_right_coef[0] * ploty**2 + lane_right_coef[1] * ploty + lane_right_coef[2]

            out_img[lane_left_y, lane_left_x] = (255, 0, 0)
            out_img[lane_right_y, lane_right_x] = (0, 0, 255)

            plt.figure(figsize=(13,7))
            splt = plt.subplot(111)

            splt.imshow(out_img)

            splt.plot(plot_lane_left, ploty, 'y')
            splt.plot(plot_lane_right, ploty, 'y')

            plt.show()


        # Curvature radius
        y_px2m = 30/720
        x_px2m = 3.7/756

        lane_real_left_coef = np.polyfit(lane_left_y * y_px2m, lane_left_x * x_px2m, 2)
        lane_real_right_coef = np.polyfit(lane_right_y * y_px2m, lane_right_x * x_px2m, 2)

        y_eval = bin_image.shape[0] * y_px2m
        left_curverad = ((1 + (2*lane_real_left_coef[0]*y_eval + lane_real_left_coef[1])**2)**1.5) / np.absolute(2*lane_real_left_coef[0])
        right_curverad = ((1 + (2*lane_real_right_coef[0]*y_eval + lane_real_right_coef[1])**2)**1.5) / np.absolute(2*lane_real_right_coef[0])

        #where lane lines are at the bottom of the image
        y_at_bottom = bin_image.shape[0]
        left_lane_pos = lane_left_coef[0] * y_at_bottom**2 + lane_left_coef[1] * y_at_bottom + lane_left_coef[2]
        right_lane_pos = lane_right_coef[0] * y_at_bottom**2 + lane_right_coef[1] * y_at_bottom + lane_right_coef[2]
        lane_center = (right_lane_pos + left_lane_pos) / 2
        car_center = bin_image.shape[1] / 2

        car_displacement = car_center - lane_center
        if debug:
            print('Displacement; left_lane_pos={}, right_lane_pos={}, lane_center={}, car_center={}'
                  .format(left_lane_pos, right_lane_pos, lane_center, car_center))

        if self.is_polinoms_parallel(lane_left_coef, lane_right_coef, max_y=y_at_bottom):

            self.left_line.detected = True
            self.left_line.add_fit(lane_left_coef)

            self.right_line.detected = True
            self.right_line.add_fit(lane_right_coef)
        else:
            if debug: print('Too bad, lines does not seem to be parallel.')
            self.left_line.detected = False
            self.right_line.detected = False

        if debug: print('Radius l={:.1f}, r={:.1f}, lane_center={:.1f}, car_delta={:.2f}m'
                        .format(left_curverad, right_curverad, lane_center, car_displacement * x_px2m))

        lane_mask = self.get_lane_region(bin_image.shape, lane_left_coef, lane_right_coef, self.get_transformation_matrix(reverse=True), debug=debug)

        return lane_mask, left_curverad, right_curverad, car_displacement * x_px2m


    def is_polinoms_parallel(self, line_left_coef, line_right_coef, max_y, delta=100):
        """
        Check if two polinoms are relatively parallel.
        Does it by calculating x value at 5 equally distributed y locations and comparing distance differences with delta.
        If distance is larger than delta - lines are off parallel course.

        :param line_left_coef: coefficients of left line polinom
        :param line_right_coef: coefficients of right line polinom
        :param max_y: maximum value of Y coordinate of the image
        :param delta: maximum allowed difference of distances between points of polinoms
        :return: True if lines are relatively parallel, otherwise False
        """
        deltas = []
        for y in range(0, max_y, max_y // 5):
            xl = line_left_coef[0]*y**2 + line_left_coef[1]*y + line_left_coef[2]
            xr = line_right_coef[0]*y**2 + line_right_coef[1]*y + line_right_coef[2]

            deltas.append(xr - xl)

        max_d = np.max(deltas)
        min_d = np.min(deltas)

        return max_d - min_d < delta


    def find_poly_with_window(self, bin_image, lane_pts_l, lane_pts_r, nonzerox, nonzeroy, debug, debug_img, WIN_N=10,
                              win_padding=100, WIN_MIN_PX_TO_RECENTER=300):
        """
        Use sliding window to find lane lines.
        This function first does search for start points using historgram of active pixels of the bottom 3rd of image.
        All image is divided vertically in WIN_N windows.
        If window has more than WIN_MIN_PX_TO_RECENTER - that window is added to a resulting array of active pixels.

        :param bin_image: binary image of active pixels (main image)
        :param lane_pts_l:
        :param lane_pts_r:
        :param nonzerox:
        :param nonzeroy:
        :param debug: debug mode on (True) or off (False). Produces extra output. Designed to be run in ipynb to show images.
        :param debug_img: debug canvas to paint on. This canvas is used to paint resulting mask. Intermediate output
        is sent directly ot plt.
        :param win_height:
        :param WIN_N:
        :param win_padding:
        :param WIN_MIN_PX_TO_RECENTER:
        :return:
        """
        lane_hist = np.sum(bin_image[int(bin_image.shape[0]/3):], axis=0)

        if debug:
            plt.plot(lane_hist)
            plt.show

        # find where to start searching for lanes
        l_start = np.argmax(lane_hist[:int(lane_hist.shape[0]//2)])
        r_start = np.argmax(lane_hist[int(lane_hist.shape[0]//2):]) + int(lane_hist.shape[0]//2)
        if debug: print('Starting lane search from left={}, right={}'.format(l_start, r_start))

        win_height = int(bin_image.shape[0] / WIN_N)
        if debug: print('Win height: {}, n_windows={}'.format(win_height, WIN_N))

        # window center positions
        center_x_left = l_start
        center_x_right = r_start
        if debug: print('left_center_x={}'.format(center_x_left))
        for win_y_idx in range(WIN_N * win_height, 0, -win_height):

            # left lane line
            win_left_x_low = center_x_left - win_padding
            win_left_x_high = center_x_left + win_padding

            if debug:
                cv2.rectangle(debug_img, (win_left_x_low, win_y_idx), (win_left_x_high, win_y_idx - win_height),
                              (0, 255, 0))

            left_px_found_mask = ((win_left_x_low < nonzerox) & (nonzerox < win_left_x_high)
                                  & (win_y_idx - win_height <= nonzeroy) & (nonzeroy < win_y_idx)).nonzero()[0]

            lane_pts_l.append(left_px_found_mask)
            if len(left_px_found_mask) > WIN_MIN_PX_TO_RECENTER:
                # set mean point
                center_x_left = int(np.mean(nonzerox[left_px_found_mask]))

            ### Right lane line
            win_right_x_low = center_x_right - win_padding
            win_right_x_high = center_x_right + win_padding

            if debug:
                cv2.rectangle(debug_img, (win_right_x_low, win_y_idx), (win_right_x_high, win_y_idx - win_height),
                              (0, 255, 0))

            right_px_found_mask = ((win_right_x_low < nonzerox) & (nonzerox < win_right_x_high)
                                   & (win_y_idx - win_height <= nonzeroy) & (nonzeroy < win_y_idx)).nonzero()[0]

            lane_pts_r.append(right_px_found_mask)
            if len(right_px_found_mask) > WIN_MIN_PX_TO_RECENTER:
                # set mean point
                center_x_right = int(np.mean(nonzerox[right_px_found_mask]))

            if debug: print(
                'win_y={}, left_center_x={}, right_center_x={}'.format(win_y_idx, center_x_left, center_x_right))


    def get_transformation_matrix(self, reverse=False, top_y=50):
        """
        Return transformation matrix for top-down view.
        :param reverse: True/False flag indicating whether matrix should be for 'to top-down' or 'from top-down'
        :returns: warp matrix to get image to and from top-down view
        """
        src_pt1 = np.array([(704, 460), (580, 460), (273, 672), (1032, 672)], np.float32)
        dst_pt1 = np.array([(src_pt1[3, 0], top_y), (src_pt1[2, 0], top_y),
                            (src_pt1[2, 0], src_pt1[3, 1]), (src_pt1[3, 0], src_pt1[3, 1])], np.float32)

        a = src_pt1
        b = dst_pt1
        if reverse:
            a = dst_pt1
            b = src_pt1

        return cv2.getPerspectiveTransform(a, b)


    def undistort_transform(self, img):
        """
        Takes img and then warps it according to warp_martix.
        @param mtx
        @param dist, Distortion coefficients
        @param img, source image to perform all transformation
        @return Transformed image
        """
        # undist = cv2.undistort(img, self.camera_mtx, self.dist_coef, None, None)

    #     src_pt1 = np.array([(704, 460), (580, 460), (273, 672), (1032, 672)], np.float32)

    #     pts = np.int32(src_pt1)
    #     undist = cv2.polylines(undist, [pts], True, (255,0,0))

        im_shape = (img.shape[1], img.shape[0])
        transformed = cv2.warpPerspective(img, self.warp_mtx, im_shape, flags=cv2.INTER_NEAREST)
        return transformed


    def get_lane_region(self, img_shape, lane_left_coef, lane_right_coef, warp_reverse_matrix, debug=False):
        """
        Create a mask for an image based on two limiting polinoms. Fills region between those polinoms.
        Two polinoms are defined by their arrays of coefficients.

        :param img_shape: shape of the canvas to use
        :param lane_left_coef: coefficients of 2dn degree polinom that defines left boundary
        :param lane_right_coef: coefficients of 2dn degree polinom that defines right boundary
        :params warp_reverse_matrix: warp martix to be used to transform image from top-down view to camera view

        :returns: binary mask

        """
        ploty = np.linspace(0, img_shape[0] - 1, img_shape[0])
        plot_lane_left = lane_left_coef[0] * ploty**2 + lane_left_coef[1] * ploty + lane_left_coef[2]
        plot_lane_right = lane_right_coef[0] * ploty**2 + lane_right_coef[1] * ploty + lane_right_coef[2]

        # Get proper coordinates
        line_left = np.transpose(np.stack((plot_lane_left, ploty)))
        line_right = np.flipud(np.transpose(np.stack((plot_lane_right, ploty))))
        pts = np.int_([np.concatenate((line_left, line_right))])

        canvas = np.zeros((img_shape[0],img_shape[1],3), np.uint8)

        cv2.fillPoly(canvas, pts, (0, 255, 0))

        im_shape = (img_shape[1], img_shape[0])
    #     print(im_shape)
        canvas = cv2.warpPerspective(canvas, warp_reverse_matrix, im_shape)

        if debug:
            plt.imshow(canvas)
            plt.show()

        return canvas


    # Threasholding logic

    def abs_sobel_thresh(self, img, orient='x', sobel_kernel=3, thresh=(0, 255)):
        """
        Absolute gradient thresholding.
        Take gradient in the direction of orient
        :param img: source image to perform gradient operation on
        :param orient: direction in which gradient is taken. Possible values 'x' and 'y'
        :param sobel_kernel: int defining size of the kernel
        :param thresh: tuple of min and max value for a threshold
        :return: binary matrix with white pixels that satisfy thresh value
        """
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        # Apply x or y gradient with the OpenCV Sobel() function
        # and take the absolute value

        if orient == 'x':
            abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0))
        else:
            abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1))
        # Rescale back to 8 bit integer
        scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
        # Create a copy and apply the threshold
        binary_output = np.zeros_like(scaled_sobel)
        # Here I'm using inclusive (>=, <=) thresholds, but exclusive is ok too
        binary_output[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1

        # Return the result
        return binary_output


    def mag_thresh(self, image, sobel_kernel=3, mag_thresh=(0, 255)):
        """
        Take gradient in both direction and then threshold it's averaged value.
        Value is calculated as sqrt(gradx**2 + grady**2)
        """

        # Apply the following steps to img
        # 1) Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        # 2) Take the gradient in x and y separately
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
        # 3) Calculate the magnitude
        sobel = np.sqrt(sobelx**2 + sobely**2)
        # 4) Scale to 8-bit (0 - 255) and convert to type = np.uint8
        scaled = np.uint8(sobel/np.max(sobel) * 255)
        # 5) Create a binary mask where mag thresholds are met
        binary = np.zeros_like(sobel)
        binary[(scaled >= mag_thresh[0]) & (scaled <= mag_thresh[1])] = 1
        # 6) Return this mask as your binary_output image
        binary_output = binary # Remove this line
        return binary_output


    def dir_threshold(self, image, sobel_kernel=3, thresh=(0, np.pi/2)):
        """
        Take gradient in x and y directions, find its direction (arctan2(y,x)) and threashold its value.
        """
        # Apply the following steps to img
        # 1) Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        # 2) Take the gradient in x and y separately
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
        # 3) Take the absolute value of the x and y gradients
        sobelx = np.absolute(sobelx)
        sobely = np.absolute(sobely)
        # 4) Use np.arctan2(abs_sobely, abs_sobelx) to calculate the direction of the gradient
        direction = np.arctan2(sobely, sobelx)
        # 5) Create a binary mask where direction thresholds are met
        binary = np.zeros_like(direction)
        binary[(direction >= thresh[0]) & (direction <= thresh[1])] = 1
        # 6) Return this mask as your binary_output image
        binary_output = binary # Remove this line
        return binary_output


    def hls_threshold(self, image, channel='s', thresh=(0,255)):
        """
        Performs Color channel thresholding.

        :param image: image to process
        :param channel: possible values 'h','l','s'
        :param thresh: tuple of min and max value for color thresholding
        """
        hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    #     hls = np.uint8(hls*255)
        if channel=='s':
            ch = hls[:,:,2]
        elif channel=='h':
            ch = hls[:,:,0]
        else:
            ch = hls[:,:,1]

        binary = np.zeros_like(ch)
        binary[(ch > thresh[0]) & (ch <= thresh[1])] = 1
    #     print(ch)

        return binary


    def reset(self):
        """ Reset state """
        self.left_line = Line()
        self.right_line = Line()
