    if event == cv2.EVENT_LBUTTONDOWN:
        if point_counter == 0:
            # print("[TRACK] - P1: ({}, {})".format(x,y))
            first_point = (x, y)
            point_counter += 1
        if point_counter == 1 and (x, y) != first_point:
            # print("[TRACK] - P2: ({}, {})".format(x,y))
            second_point = (x, y)
            point_counter += 1
        if tracking and point_counter == 2:
            point_counter = 0
            first_point = None
            second_point = None
            tracking = False
            reset_track = True
            # print("[TRACK] - ROI RESET")
        if point_counter == 2:
            reset_track = False
            tracker = cv2.legacy.TrackerCSRT_create()
            tracker.init(empty_frame, (first_point[0], first_point[1], abs(
                second_point[0] - first_point[0]), abs(second_point[1] - first_point[1])))
            tracking = True