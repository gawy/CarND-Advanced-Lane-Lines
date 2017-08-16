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
        """ polynomial coefficients for the most recent fit """
        self.current_fit = [np.array([False])]

        # radius of curvature of the line in Meters
        self.radius_of_curvature = None

        # x values for detected line pixels
        self.allx = None
        # y values for detected line pixels
        self.ally = None

    def add_fit(self, poly_fit):
        self.current_fit = poly_fit

        if self.best_fit is not None:
            self.best_fit = (self.best_fit + poly_fit) / 2
        else:
            self.best_fit = poly_fit
