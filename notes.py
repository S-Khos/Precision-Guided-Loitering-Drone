# if event == cv2.EVENT_LBUTTONDOWN:
#     if point_counter == 0:
#         # print("[TRACK] - P1: ({}, {})".format(x,y))
#         first_point = (x, y)
#         point_counter += 1
#     if point_counter == 1 and (x, y) != first_point:
#         # print("[TRACK] - P2: ({}, {})".format(x,y))
#         second_point = (x, y)
#         point_counter += 1
#     if tracking and point_counter == 2:
#         point_counter = 0
#         first_point = None
#         second_point = None
#         tracking = False
#         reset_track = True
#         # print("[TRACK] - ROI RESET")
#     if point_counter == 2:
#         reset_track = False
#         tracker = cv2.legacy.TrackerCSRT_create()
#         tracker.init(empty_frame, (first_point[0], first_point[1], abs(
#             second_point[0] - first_point[0]), abs(second_point[1] - first_point[1])))
#         tracking = True


# def mouse_event_handler(event, x, y, flags, param):
#     global tracker, tracking, reset_track, cursor_pos, manual_control
#     cursor_pos[0] = x - manual_control.designator_roi_size[0] // 2
#     cursor_pos[1] = y - manual_control.designator_roi_size[1] // 2
#     if event == cv2.EVENT_LBUTTONDOWN:
#         if (not tracking and reset_track):
#             reset_track = False
#             tracker = cv2.legacy.TrackerCSRT_create()
#             tracker.init(
#                 empty_frame, (cursor_pos[0], cursor_pos[1], manual_control.designator_roi_size[0], manual_control.designator_roi_size[1]))
#             tracking = True
#         else:
#             reset_track = True
