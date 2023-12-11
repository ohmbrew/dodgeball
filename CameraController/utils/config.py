"""Configures and stores constants for running the autonomous pinball backend."""
from pathlib import Path

# Datasets
# Resolve changes symlinks to absolute path
DATASET_DIRECTORY_PATH = Path("./datasets/").resolve()
"""Path: path to any datasets."""
VIDEO_PATH: Path = (DATASET_DIRECTORY_PATH / "green.mp4").resolve()
"""Path: path to any test videos."""

# Functional
UPDATE_PLAYFIELD_INTERVAL = 60
"""int: number of frames per update of corners of playfield."""
VIDEO_CAPTURE_NUMBER = 1
"""int: the capture camera number to try to read from."""
VIDEO_CAPTURE_INPUT = str(VIDEO_CAPTURE_NUMBER)
"""Any: the actual input to the cv2.VideoCapture function. Can be swapped between VIDEO_CAPTURE_NUMBER, VIDEO_PATH, 
IMG_PATH, etc. """

# Playfield Corner Detection Tunable Parameters
GAUSSIAN_BLUR_KERNEL_SIZE = (5, 5)
"""Tuple[int, int]: kernel size of the Gaussian Blur."""
CONTOUR_APPROXIMATION_COEFFICIENT = 0.1
"""float: coefficient of epsilon in polygon approximation of the contour."""
CORNERS_BINARY_THRESHOLD_MIN = 20
"""int: minimum threshold for the binary thresholding of the playfield corners."""
CORNERS_BINARY_THRESHOLD_MAX = 255
"""int: maximum threshold for the binary thresholding of the playfield corners."""
CORNER_CONTOUR_AREA_MIN = 30
"""int: minimum area of the contour of the playfield corners."""
CORNER_CONTOUR_AREA_MAX = 250
"""int: maximum area of the contour of the playfield corners."""


# HSV color mapping intervals to find pinball playfield corners
CORNER_LOWER_YELLOW = [20, 130, 130]
"""List[int]: lower bound of the yellow color range."""
CORNER_UPPER_YELLOW = [30, 255, 255]
"""List[int]: upper bound of the yellow color range."""
CORNER_LOWER_BLUE = [210, 100, 100]
"""List[int]: lower bound of the blue color range."""
CORNER_UPPER_BLUE = [240, 255, 255]
"""List[int]: upper bound of the blue color range."""

# From test.py, this range of red DEFINITELY covers the pinball. but has extra stuff. But let's start here

#RED_MIN = np.array([150, 65, 65],np.uint8)
#RED_MAX = np.array([180, 255, 255],np.uint8)

# HSV color mapping intervals for different pinball colors
BALL_LOWER_GREEN = [40, 0, 80]
BALL_UPPER_GREEN = [60, 255, 255]
BALL_LOWER_ORANGE = [10, 130, 130]
BALL_UPPER_ORANGE = [25, 255, 255]
BALL_LOWER_RED = [160, 60, 60]
BALL_UPPER_RED = [180, 255, 255]
BALL_LOWER_BLUE = [0,0,0]
BALL_UPPER_BLUE = [255,255,255]


# Assign pinball color to look for
PINBALL_COLOR = "RED"
"""str: the color of the pinball to look for."""
PINBALL_COLOR_RANGES = {
    "GREEN": [BALL_LOWER_GREEN, BALL_UPPER_GREEN],
    "ORANGE": [BALL_LOWER_ORANGE, BALL_UPPER_ORANGE],
    "BLUE": [BALL_LOWER_BLUE, BALL_UPPER_BLUE],
    "RED": [BALL_LOWER_RED, BALL_UPPER_RED]
}
"""str: color of the pinball to find."""
[PINBALL_LOWER_COLOR, PINBALL_UPPER_COLOR] = PINBALL_COLOR_RANGES[PINBALL_COLOR]
"""List[int]: lower and upper bounds of pinball color to look for."""

# Pinball Detection Tunable Parameters
PINBALL_CONTOUR_MIN_AREA = 200
"""int: minimum area of the contour of the pinball."""
PINBALL_CONTOUR_MAX_AREA = 2500
"""int: maximum area of the contour of the pinball."""
PINBALL_CONTOUR_MAX_PERIMETER = 2000
"""int: maximum perimeter of the contour of the pinball."""
PINBALL_CONTOUR_MAX_SIDES = 255

