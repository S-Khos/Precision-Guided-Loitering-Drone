 while tracking and tracker_ret and manual_control == False:
        x, y, w, h = [int(value) for value in roi]
        flt_ctrl_lock.acquire()
        cur_x_error = (x + w // 2) - centreX
        cur_y_error = (y + h // 2) - centreY
        cur_time = time.time()

        # kp * error + kd * (error - prev_error)
        # p = kp * error
        # i = i + ki * error * (cur_time - prev_time)
        # d = kd * (error - prev_error) / (cur_time - prev_time)
        # pid = p + i + d
        # prev_error = error
        # prev_time = cur_time

        p = yaw_pid[0] * cur_x_error
        i = i + yaw_pid[1] * cur_x_error * (cur_time - prev_time)
        d = yaw_pid[2] * (cur_x_error - prev_x_error) / (cur_time - prev_time)


        #r_spd = yaw_pid[0] * cur_x_error + yaw_pid[1] * (cur_x_error - prev_x_error)
        #x_spd = x_pid[0] * cur_x_error + x_pid[1] * (cur_x_error - prev_x_error)
        #y_spd = y_pid[0] * cur_y_error + y_pid[1] * (cur_y_error - prev_y_error)
        #r_spd = int((np.clip(r_spd, -100, 100)) // 1.2)
        r_spd = p + i + d

        r_spd = int((np.clip(r_spd, -100, 100)))
        #y_spd = int(np.clip(y_spd, -100, 100))
        prev_x_error = cur_x_error
        #prev_y_error = cur_y_error
        prev_time = cur_time
        flt_ctrl_lock.release()
        print("[PID]  X: {}".format(r_spd))
        #if tello.send_rc_control:
            #tello.send_rc_control(x_spd, 0, 0, 0)
        time.sleep(0.01)





    



    #prev_time = time.time()
    #r_spd_bar = 0
    #r_i = 0
    #x_i = 0
    #y_i = 0
    prev_x_error = 0
    prev_y_error = 0
    pid_yaw = PID(0.35,0.2,0.2,setpoint=0,output_limits=(-100,100))
    pid_y = PID(0.3,0.3,0.3,setpoint=0,output_limits=(-100,100))
    pid_x = PID(0.2,0.2,0.2,setpoint=0,output_limits=(-100,100))

    while tracking and tracker_ret and manual_control == False:
        x, y, w, h = [int(value) for value in roi]
        cur_x_error = (x + w // 2) - centreX
        cur_y_error = (y + h // 2) - centreY
        #cur_time = time.time()

        #r_p = yaw_pid[0] * cur_x_error
        #r_i = yaw_pid[1] * cur_x_error * (cur_time - prev_time)
        #r_d = yaw_pid[2] * (cur_x_error - prev_x_error) / (cur_time - prev_time)
        
        #r_spd = r_spd_bar + r_p + r_i + r_d
        #r_spd = int((np.clip(r_spd, -100, 100)))
        r_spd = int(pid_yaw(cur_x_error))
        x_spd = int(pid_yaw(cur_x_error))
        y_spd = int(pid_y(cur_y_error))


        #prev_x_error = cur_x_error
        #prev_time = cur_time
        #print("[PID]  X: {}".format(r_spd))
        if tello.send_rc_control:
            tello.send_rc_control(-x_spd, 7, y_spd, -r_spd)
        time.sleep(0.01)

    pid_yaw.reset()
    pid_y.reset()
    pid_x.reset()
    flt_ctrl_active = False
    print("[FLT CTRL] - TERMINATED")