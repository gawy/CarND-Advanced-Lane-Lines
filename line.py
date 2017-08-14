import numpy as np


class Line:

    def __init__(self):

        # was the line detected in the last iteration?
        self.detected = False

        # x values of the last n fits of the line
        self.recent_xfitted = []
        # average x values of the fitted line over the last n iterations
        self.bestx = None
        # polynomial coefficients averaged over the last n iterations
        self.best_fit = None
        # polynomial coefficients for the most recent fit
        self.current_fit = [np.array([False])]

        # radius of curvature of the line in Meters
        self.radius_of_curvature = None

        # x values for detected line pixels
        self.allx = None
        # y values for detected line pixels
        self.ally = None
