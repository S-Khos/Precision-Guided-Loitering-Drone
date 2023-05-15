def flight_controller():
    global centreX, centreY, drone, yaw_pid, y_pid, x_pid, roi, tracker_ret, flt_ctrl_lock, flt_ctrl_active, tracking, manual_control
    print("[FLT CTRL] - ACTIVE")

    #prev_time = time.time()
    
    #prev_x_error = 0
    #prev_y_error = 0
    
    pid_yaw = PID(0.35,0.2,0.2,setpoint=0,output_limits=(-100,100))
    #pid_y = PID(0.3,0.3,0.3,setpoint=0,output_limits=(-80,80))
    #pid_x = PID(0.2,0.2,0.2,setpoint=0,output_limits=(-100,100))

    while tracking and tracker_ret and manual_control == False:
        x, y, w, h = [int(value) for value in roi]
        cur_x_error = (x + w // 2) - centreX
        cur_y_error = (y + h // 2) - centreY
        #cur_time = time.time()

#        r_p = yaw_pid[0] * cur_x_error
#        r_i = yaw_pid[1] * cur_x_error * (cur_time - prev_time)
#        r_d = yaw_pid[2] * (cur_x_error - prev_x_error) / (cur_time - prev_time)
#        prev_x_error = cur_x_error
        
#        r_spd = r_p + r_i + r_d
#        r_spd = int((np.clip(r_spd, -90, 90)))

        r_spd = int(pid_yaw(cur_x_error))
        #x_spd = int(pid_yaw(cur_x_error))
        #y_spd = int(pid_y(cur_y_error))

        #print("[PID]  X: {}".format(r_spd))
        if drone.send_rc_control:
            drone.send_rc_control(0, 0, 0, -r_spd)
        time.sleep(0.01)

    pid_yaw.reset()
    pid_y.reset()
    pid_x.reset()
    flt_ctrl_lock.acquire()
    flt_ctrl_active = False
    flt_ctrl_lock.release()
    print("[FLT CTRL] - TERMINATED")
