from djitellopy import Tello
import cv2
import time
import threading


def main():

    drone = Tello()

    try:
        drone.connect()
        if (drone.get_battery() > 25):
            drone.streamon()
            frame_read = drone.get_frame_read()
            cv2.namedWindow("FEED", cv2.WINDOW_NORMAL)
            fps_init_time = time.time()
            while frame_read:
                frame = frame_read.frame
                elapsed_time = time.time() - fps_init_time
                fps = int(1 / elapsed_time)
                fps_init_time = time.time()
                frame = cv2.circle(frame, (400, 200), 100, (0, 0, 255), 2)
                frame = cv2.putText(frame, "FPS  {}".format(
                    fps), (10, 25), cv2.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 255), 1)

                cv2.imshow("FEED", frame)

                if (cv2.waitKey(1) & 0xff) == 27:
                    break

            cv2.destroyAllWindows()
            drone.streamoff()
            drone.end()

        else:
            print("[DRONE] - Low battery")
            drone.end()

    except Exception as error:
        print("[DRONE] - ", error)
        drone.end()

    print("[DRONE] - CONNECTION TERMINATED")


if __name__ == "__main__":
    main()
