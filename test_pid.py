    def init_PID(self):
        
        def proportional():
            Vx = 0
            Vy = 0
            prev_time = time.time()
            Ix = 0
            Iy = 0
            ex_prev = 0
            ey_prev = 0
            while True:
                #yield an x and y velocity from xoff, yoff, and distance
                xoff, yoff, distance = yield Vx, Vy

                #PID Calculations
                ex = xoff - distance
                ey = yoff - distance
                current_time = time.time()
                delta_t = current_time - prev_time
                
                #Control Equations, constants are adjusted as needed
                Px = 0.1*ex
                Py = 0.1*ey
                Ix = Ix + -0.001*ex*delta_t
                Iy = Iy + -0.001*ey*delta_t
                Dx = 0.01*(ex - ex_prev)/(delta_t)
                Dy = 0.01*(ey - ey_prev)/(delta_t)

                Vx = Px + Ix + Dx
                Vy = Py + Iy + Dy

                #update the stored data for the next iteration
                ex_prev = ex
                ey_prev = ey
                prev_time = current_time
        self.PID = proportional()
        self.PID.send(None)    


    def process_frame(self, frame):
        """convert frame to cv2 image and show"""
        image = cv2.cvtColor(numpy.array(
            frame.to_image()), cv2.COLOR_RGB2BGR)
        image = self.write_hud(image)
        if self.record:
            self.record_vid(frame)
        distance = 0

        xoff, yoff = self.colortracker.track(image)
        
        image = self.colortracker.draw_arrows(image)
        Vx,Vy=self.PID.send([xoff, yoff, distance])        
        # print("TARGET_V: ({Vx},{Vy})".format(Vx=Vx,Vy=Vy)) # Print statement to ensure Vx and Vy are reasonable values (<50)   
                
        # Create a loop to implement the Vx and Vy as a command to move the drone accordingly
        cmd = "" 
        speed = 0

        if self.tracking:
            if abs(Vx) > abs(Vy):
                if Vx > 0:
                    cmd = "right"
                    speed = abs(Vx)
                elif Vx < 0:
                    cmd = "left"
                    speed = abs(Vx)
            else:
                if Vy > 0:
                    cmd = "up"
                    speed = abs(Vy)
                elif Vy < 0:
                    cmd = "down"
                    speed = abs(Vy)