import cv2
import time
import numpy as np
from utils import *

img = cv2.imread('redball.jpg')

RED_MIN = np.array([150, 65, 65],np.uint8)
RED_MAX = np.array([180, 255, 255],np.uint8)

hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

frame_threshed = cv2.inRange(hsv_img, RED_MIN, RED_MAX)
cv2.imwrite('output.jpg', frame_threshed)


# prev_time = 0
# new_time = 0
# fond = cv2.FONT_HERSHEY_DUPLEX
# playfield_corners = None
# prev_frame = None
# runs = 0

# rotated_frame = DisplayUtils.rotate_frame_counterclockwise(frame=hsv_img)

# # Recalculate playfield corner coordinates every 30 frames
# if runs % 30 == 0 or playfield_corners is None:
#     playfield_corners_temp = PinballUtils.get_playfield_corners(rotated_frame)
#     if len(playfield_corners_temp) == 4:
#         print(f"Playfield corners found - Run: {runs}")
#         playfield_corners = playfield_corners_temp

# # Warp frame to playfield
# if playfield_corners is not None:
#     warped_frame = PinballUtils.warp_frame(rotated_frame, playfield_corners)
# else:
#     warped_frame = rotated_frame

# # Pinball detection
# if runs == 0:
#     print(f"Looking for {config.PINBALL_COLOR} pinball")
# pinball_coordinates = PinballUtils.get_pinball_coordinates(warped_frame, prev_frame)
# print(pinball_coordinates)
# # Draw pinball coordinates
# #for pc in pinball_coordinates:
# DisplayUtils.draw_circles(warped_frame, pinball_coordinates, radius=20)

# new_time = time.time()
# fps = f"FPS: {int(1 / (new_time - prev_time))}"
# cv2.putText(warped_frame, fps, (10, 30), fond, 1, (255, 255, 255), 2, cv2.LINE_AA)
# prev_time = new_time
# prev_frame = rotated_frame

# cv2.imwrite('output.jpg', warped_frame)
# # Display frame
# DisplayUtils.display_frame(warped_frame, "Post-processed frame")

# runs += 1

# while True:
#     pass