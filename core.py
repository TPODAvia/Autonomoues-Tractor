from Global_Planning.GridBasedSweepCPP.grid_based_sweep_coverage_path_planner import *
from Global_Planning.AStar.a_star import *
from Local_Planning.model_predictive_speed_and_steer_control.model_predictive_speed_and_steer_control import *
from Local_Planning.DWA.demo import *

try:
    from Motor_Control.motordriver.motor_driver import MotorDriver
except:
    print("Simulation mode On")
    print("No motor driver is activated")


odrawing = False

# mouse callback function
def draw_circle(event,x,y,flags,param):
    global odrawing
    global ogoal
    if event == cv2.EVENT_LBUTTONDOWN:
        odrawing = True

    if event == cv2.EVENT_MOUSEMOVE:
        if odrawing:
            if [x, y] not in odraw_points:
                odraw_points.append([x, y])
                opoint_cloud.append([x/10, y/10])
                ogoal = None
        else:
            ogoal = (x/10, y/10)
    elif event == cv2.EVENT_LBUTTONUP:
        odrawing = False


if __name__ == '__main__':

    import argparse
    parser = argparse.ArgumentParser(description='DWA Demo')
    parser.add_argument('--save', dest='save', action='store_true')
    parser.set_defaults(save=False)
    args = parser.parse_args()
    if args.save:
        import imageio
        writer = imageio.get_writer('./dwa.gif', mode='I', duration=0.05)

    # 1 SENSOR READING
    print("Sensor reading is not implemented")


    # 2 LOCALISATION(This need to be implemented SLAM)
    goalx = 0.0
    goaly = 0.0

    startx = 50.0
    starty = 50.0


    # 3 MAPPING
    modeMAP = "SQUARE" # OBSTACLE SQUARE

    if modeMAP == "OBSTACLE":
        # set obstacle positions
        x1, y1 = [], []
        for i in range(-10, 60,3):
            x1.append(i)
            y1.append(-10.0)
        for i in range(-10, 60,3):
            x1.append(60.0)
            y1.append(i)
        for i in range(-10, 61,3):
            x1.append(i)
            y1.append(60.0)
        for i in range(-10, 61,3):
            x1.append(-10.0)
            y1.append(i)
        for i in range(-10, 40,3):
            x1.append(20.0)
            y1.append(i)
        for i in range(0, 40,3):
            x1.append(40.0)
            y1.append(60.0 - i)

    elif modeMAP == "SQUARE":
        x1, y1 = [], []
        for i in range(-5, 60, 4):
            x1.append(-5.0)
            y1.append(i)
        for i in range(-5, 60, 4):
            x1.append(i)
            y1.append(60.0)
        for i in range(-5, 60, 4):
            x1.append(60.0)
            y1.append(55.0-i)
        for i in range(-5, 60, 4):
            x1.append(55.0-i)
            y1.append(-5.0)

        x1.append(-5)
        y1.append(-5)

        # print(x1)
        # print(y1)

    # 4 GLOBAL PATH PLANNING

    print("Start Gloabl Planning")
    modeGP = "DRIVING" # "FARMING" "DRIVING" "MANUAL"


    if modeGP == "MANUAL":
        print("Manual modeGP")

    elif modeGP == "DRIVING":
        
        print(__file__ + " driving start!!")

        # start and goal position
        sx = goalx  # [m]
        sy = goaly  # [m]
        gx = startx  # [m]
        gy = starty  # [m]

        grid_size = 2.0  # [m]
        robot_radius = 5.0  # [m]

        if show_animation:  # pragma: no cover
            plt.plot(x1, y1, ".k")
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")

        a_star = AStarPlanner(x1, y1, grid_size, robot_radius)
        ax, ay = a_star.planning(sx, sy, gx, gy, animation = False)

        if show_animation:  # pragma: no cover
            plt.plot(ax, ay, "-r")
            plt.pause(0.001)
            plt.show()


    elif modeGP == "FARMING":

        print(__file__ + " farming start!!")

        # Padding

        pad=3
        x2, y2 = [], []
        for i in range(-5+pad, 60-pad, 4):
            x2.append(-5.0+pad)
            y2.append(i)
        for i in range(-5+pad, 60-pad, 4):
            x2.append(i)
            y2.append(60.0-pad)
        for i in range(-5+pad, 60-pad, 4):
            x2.append(60.0-pad)
            y2.append(55.0-i)
        for i in range(-5+5, 60-5, 4):
            x2.append(55.0-i)
            y2.append(-5.0+pad)

        x2.append(0)
        y2.append(0)

        print(x2)
        print(y2)


        
        resolution = 2
        ax, ay = planning(x2, y2, resolution)


    # planning_animation(x1, y1, resolution)
    # sys.exit()

    # # 5 LOCAL PATH PLANNING

    modeLP = "DWA" # "MPC" "DWA"


    if modeGP == "MANUAL":
        print("Manual modeLP")

    elif modeLP == "MPC":
        print(__file__ + "  MPC start!!")

        dl = 1.0  # course tick
        cx, cy, cyaw, ck = get_straight_course3(dl,ax,ay)
        
        sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

        initial_state = State(x=cx[0], y=cy[0], yaw=0.0, v=0.0)


        """
        cx: course x position list
        cy: course y position list
        cy: course yaw position list
        ck: course curvature list
        sp: speed profile
        dl: course tick [m]
        """

        goal = [cx[-1], cy[-1]]

        state = initial_state

        # initial yaw compensation
        if state.yaw - cyaw[0] >= math.pi:
            state.yaw -= math.pi * 2.0
        elif state.yaw - cyaw[0] <= -math.pi:
            state.yaw += math.pi * 2.0

        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        d = [0.0]
        a = [0.0]
        target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

        odelta, oa = None, None

        cyaw = smooth_yaw(cyaw)
        
        while MAX_TIME >= time:
            xref, target_ind, dref = calc_ref_trajectory(
                state, cx, cy, cyaw, ck, sp, dl, target_ind)

            x0 = [state.x, state.y, state.v, state.yaw]  # current state

            oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
                xref, x0, dref, oa, odelta)

            if odelta is not None:
                di, ai = odelta[0], oa[0]

            state = update_state(state, ai, di)
            time = time + DT

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
            d.append(di)
            a.append(ai)


            # 6 MOTOR DRIVER
            try:
                MotorDriver.set_cmd_vel(sp, yaw)
            except:
                pass

            if check_goal(state, goal, target_ind, len(cx)):
                print("Goal")
                print("Task finished")
                break

            if show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.plot(x1, y1, ".k")
                plt.plot(ax, ay, "-r")

                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                if ox is not None:
                    plt.plot(ox, oy, "xr", label="MPC")
                plt.plot(cx, cy, "-r", label="course")
                plt.plot(x, y, "ob", label="trajectory")
                plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plot_car(state.x, state.y, state.yaw, steer=di)
                plt.axis("equal")
                plt.grid(True)
                plt.title("Time[s]:" + str(round(time, 2))
                        + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
                plt.pause(0.0001)

    elif modeLP == "DWA":

        print(__file__ + " DWA start!!")

        cv2.namedWindow('cvwindow')
        cv2.setMouseCallback('cvwindow', draw_circle)

        opoint_cloud = []
        odraw_points = []
        

        ovel = (0.0, 0.0)
        opose = (startx, starty, 0)

        ogoal = None
        obase = [-3.0, -2.5, +3.0, +2.5]
        oconfig = dwa.Config(
                    max_speed = 1.0,
                    min_speed = -0.5,
                    max_yawrate = np.radians(40.0),
                    max_accel = 15.0,
                    max_dyawrate = np.radians(110.0),
                    velocity_resolution = 0.1,
                    yawrate_resolution = np.radians(1.0),
                    dt = 0.1,
                    predict_time = 3.0,
                    heading = 0.15,
                    clearance = 1.0,
                    velocity = 1.0,
                    base = obase)
        
        import argparse
        parser = argparse.ArgumentParser(description='DWA Demo')
        parser.add_argument('--save', dest='save', action='store_true')
        parser.set_defaults(save=False)
        args = parser.parse_args()




        i = 0
        while i < len(x1):
            odraw_points.append([int(x1[i])*8+100, int(y1[i])*8+100])
            opoint_cloud.append([(int(x1[i])*8+100)/10, (int(y1[i])*8+100)/10])
            i=i+1

        i = 0
        count = 0


        while True:
            prev_time = time.time()
            omap = np.zeros((600, 600, 3), dtype=np.uint8)


            for point in odraw_points:
                cv2.circle(omap, tuple(point), 4, (255, 255, 255), -1)


            if i < len(ax) and count == 20 :
                i=i+1
                count = 0
            cv2.circle(omap, (int(ax[i]*8+100), int(ay[i]*8+100)),
                    4, (0, 255, 0), -1)
            ogoal = ((int(ax[i])*8+100)/10, (int(ay[i])*8+100)/10)
            count = count + 1




            if ogoal is not None:
                # cv2.circle(omap, (int(ogoal[0]*10), int(ogoal[1]*10)),
                #         4, (0, 255, 0), -1)
                if len(opoint_cloud):
                    # Planning
                    ovel = dwa.planning(opose, ovel, ogoal,
                            np.array(opoint_cloud, np.float32), oconfig)
                    # Simulate motion
                    opose = dwa.motion(opose, ovel, oconfig.dt)

            # 6 MOTOR DRIVER
            try:
                MotorDriver.set_cmd_vel(ovel[0], ovel[1])
            except:
                pass


            pose = np.ndarray((3,))
            pose[0:2] = np.array(opose[0:2]) * 10
            pose[2] = opose[2]

            base = np.array(obase) * 10
            base[0:2] += pose[0:2]
            base[2:4] += pose[0:2]

            # Not the correct rectangle but good enough for the demo
            width = base[2] - base[0]
            height = base[3] - base[1]
            rect = ((pose[0], pose[1]), (width, height), np.degrees(pose[2]))
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(omap,[box],0,(0,0,255),-1)

            # Prevent divide by zero
            fps = int(1.0 / (time.time() - prev_time + 1e-10))
            cv2.putText(omap, f'FPS: {fps}', (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.putText(omap, f'Point Cloud Size: {len(opoint_cloud)}',
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            if args.save:
                writer.append_data(omap)

            cv2.imshow('cvwindow', omap)
            key = cv2.waitKey(1)
            if key == 27:
                break
            elif key == ord('r'):
                opoint_cloud = []
                odraw_points = []

        if args.save:
            writer.close()
