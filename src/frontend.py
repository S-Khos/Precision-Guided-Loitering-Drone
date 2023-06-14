import cv2
import numpy as np
import time
import threading
import math


class FrontEnd(object):

    FRAME_WIDTH = 960
    FRAME_HEIGHT = 720
    CENTRE_X = int(FRAME_WIDTH / 2)
    CENTRE_Y = int(FRAME_HEIGHT / 2)
    FONT = cv2.FONT_HERSHEY_COMPLEX
    FONT_SCALE = .6
    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    BLUE = (255, 0, 0)
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    UI_COLOUR = WHITE
    LINE_THICKNESS = 1

    def __init__(self, backend):
        self.backend = backend
        self.frame = None
        self.designator_frame = None

    def get_feed_size(self):
        return (FrontEnd.FRAME_WIDTH, FrontEnd.FRAME_HEIGHT)

    def get_feed_centre(self):
        return (FrontEnd.CENTRE_X, FrontEnd.CENTRE_Y)
