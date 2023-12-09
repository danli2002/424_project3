"""
Lane-Keeping RC Car

This project powers a lane-keeping RC car using a Raspberry Pi 4 and 
utilizes the OpenCV library for computer vision tasks, specifically
lane detection. The car is controlled using an Adafruit DAC by setting
the voltage of the throttle and steering channels.

Authors: Daniel Li, Daniel Choi, Desmond Roberts, Akshay Shyam, Samatar Dalmar

December 6, 2023
"""
from collections import deque
from motor_control import MotorControl
import cv2
import numpy as np
import math
import sys
import time

STEERING_CONSTANT = 60
DEAD_ZONE = 10



def setup_video():
    """
    Sets up the video capture device and configures the frame size.

    Returns:
        video (cv2.VideoCapture): The video capture device.

    """

    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # set the width to 320 p
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # set the height to 240 p
    return video

def save_image(image, filename):
    """
    Saves an image to the images folder.

    Parameters:
        image (numpy.ndarray): The image to save.
        filename (str): The name of the file to save the image to.
    """

    cv2.imwrite("images/" + filename, image)

def convert_to_HSV(frame):
    """
    Converts an input frame from BGR color space to HSV color space.

    Parameters:
        frame (numpy.ndarray): The input frame in BGR color space.

    Returns:
        numpy.ndarray: The converted frame in HSV color space.
    """
    
    # convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow("HSV",hsv)
    save_image(hsv, "hsv.jpg")
    return hsv

def detect_edges(frame):
    """
    Detects edges in an input frame using Canny edge detection.

    Parameters:
        frame (numpy.ndarray): The input frame in HSV color space.

    Returns:
        numpy.ndarray: The frame with detected edges.
    """

    # define the blue limits for lane detection
    lower_blue = np.array([90, 120, 0], dtype="uint8") # lower limit of the blue color
    upper_blue = np.array([150, 255, 255], dtype="uint8") # upper limit of the blue color
    mask = cv2.inRange(frame, lower_blue, upper_blue) # mask the frame to get only blue colors
    # save_image(mask, "mask.jpg")
    # cv2.imshow("mask", mask)
    # detect edges
    edges = cv2.Canny(mask, 50, 100)
    # cv2.imshow("edges", edges)
    save_image(edges, "edges.jpg")
    return edges

def region_of_interest(edges):
    """
    Defines the region of interest in the frame for lane detection.

    Parameters:
        edges (numpy.ndarray): The frame with detected edges.

    Returns:
        numpy.ndarray: The frame with the region of interest defined.
    """

    height, width = edges.shape # get the height and width of the frame
    mask = np.zeros_like(edges) # create a mask of the same size as the frame
    
    # only focus on the bottom half of the frame
    # define the polygon by defining a four-sided polygon
    polygon = np.array([[
        (0, height),
        (0, height / 2),
        (width, height / 2),
        (width, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255) # fill the polygon with blue
    cropped_edges = cv2.bitwise_and(edges, mask) # mask the edges to get only the region of interest
    # save_image(cropped_edges, "roi.jpg")
    # cv2.imshow("roi",cropped_edges)
    return cropped_edges

def detect_line_segments(cropped_edges):
    """
    Detects line segments in an image using the Hough transform.

    Parameters:
    - cropped_edges: A binary image containing edges.

    Returns:
    - line_segments: A list of line segments represented by their endpoints.
    """

    rho = 1
    theta = np.pi / 180
    min_threshold = 10
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=150)
    # save_image(line_segments, "line_segments.jpg")
    # cv2.imshow("segments", line_segments)
    return line_segments

def average_slope_intercept(frame, line_segments):
    """
    Calculate the average slope and intercept of the detected line segments.

    Args:
        frame (numpy.ndarray): The input image frame.
        line_segments (list): A list of line segments detected in the frame.

    Returns:
        list: A list of lane lines represented by their slope and intercept.
    """

    lane_lines = []

    if line_segments is None:
        # print("no line segments detected")
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1/3  # the boundary for the left and right lane

    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on right 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                
                continue
                # print("skipping vertical line segment (slope = infinity)")
                # continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))
            
    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))
    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    """
    Convert a line represented in slope and intercept into pixel points.

    Args:
        frame (numpy.ndarray): The input image frame.
        line (list): A list of the slope and intercept of the line.

    Returns:
        list: A list of pixel points representing the line.
    """

    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    if slope == 0:
        slope = 0.1
    x1 = int((y1  - intercept) / slope)
    x2 = int((y2  - intercept) / slope)

    return [[x1, y1, x2, y2]]

def detect_stopsign(image):
    hsv = convert_to_HSV(image)
    
    #red color has two ranges in hsv
    lower_red1 = np.array([0, 70, 20])
    upper_red1 = np.array([20, 255, 255])
    
    lower_red2 = np.array([130, 70,20])
    upper_red2 = np.array([189,255,255])
    
    red_lower = cv2.inRange(hsv, lower_red1, upper_red1)
    red_upper = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = red_lower + red_upper
    red = cv2.bitwise_and(image, image, mask = red_mask)
    total_pixels = red.size
    #if there is a significant amount of red, recognize it as a stopsign/light
    red_pixels = np.count_nonzero(red)
    percent = (red_pixels / total_pixels) * 100
    if percent > 30:
        return True
    return False

def display_lines(frame, lines, line_color=(0,255,0), line_width=6):
    """
    Display lines on an image.

    Args:
        frame (numpy.ndarray): The input image frame.
        lines (list): A list of lines represented by their endpoints.
        line_color (tuple): The color of the lines.
        line_width (int): The width of the lines.
    """

    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1,y1), (x2,y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    # save_image(line_image, "line_image.jpg")
    # cv2.imshow("lane lines", line_image)

def get_steering_angle(frame, lane_lines, default_angle = 0):
    """
    Calculates the steering angle based on the detected lane lines.

    Args:
        frame (numpy.ndarray): The input frame/image.
        lane_lines (list): A list of lane lines detected in the frame.
        (optional) last_steering_angle: if no line is found just default to this value

    Returns:
        int: The steering angle needed by the vehicle's front wheel.
    """
    
    height, width, _ = frame.shape

    if len(lane_lines) == 2: # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
    
    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)
    
    elif len(lane_lines) == 0:

        #SAMATAR
        # When no line is detected we want to continue at the same deviation, not reset to 0!
        return default_angle * 0.8
        """
        x_offset = 0
        y_offset = int(height / 2)
        """
    angle_to_mid_radian = math.atan(x_offset / y_offset) # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi) # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90 # this is the steering angle needed by picar front wheel

    return steering_angle

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    """
    Display a heading line on the given frame indicating the steering angle.

    Args:
        frame (numpy.ndarray): The input frame to display the heading line on.
        steering_angle (float): The steering angle in degrees.
        line_color (tuple, optional): The color of the heading line in BGR format. Defaults to (0, 0, 255).
        line_width (int, optional): The width of the heading line. Defaults to 5.

    Returns:
        numpy.ndarray: The frame with the heading line displayed.
    """
    
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    if math.tan(steering_angle_radian) == 0:
        return frame
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    # print("x2", x2)
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
    # cv2.imshow("heading line", heading_image)
    save_image(heading_image, "heading_line.jpg")
    return heading_image

def write_list_to_file(numbers, name):
    # Convert the list of numbers to a string with commas
    numbers_string = ','.join(map(str, numbers))

    # Write the string to a file
    with open(f'{name}.txt', 'w') as file:
        file.write(numbers_string)
    

def test_image_checkpoints():
    video = setup_video()
    ret, frame = video.read()
    hsv = convert_to_HSV(frame)
    edges = detect_edges(hsv)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    lane_lines = average_slope_intercept(frame, line_segments)
    steering_angle = get_steering_angle(frame, lane_lines)

    heading_image = display_heading_line(frame, steering_angle)

def run():
    """
    Runs the lane-keeping RC car.
    """
    queue = []
    motor_control = MotorControl()
    video = setup_video()
    start = time.time()
    steering_angle = 0
    last_steering_angle = 0
    deviation = 0
    last_deviation = 0
    speed = 8
    lastTime = 0
    lastError = 0
    stopSignCounter = 1
    isStop = False
    passedFirstStop = False
    lastStopTime = 0
    error_vals = []
    p_vals = []
    d_vals = []
    speed_vals = []
    steer_vals = []

    kp = 0.8 / 24 
    kd = kp * 0.06

    time.sleep(1)

    while True and time.time() - start < 60:
        ret, frame = video.read()
        if not ret:
            video = setup_video()
            continue
        frame = cv2.resize(frame, (160,120))

        hsv = convert_to_HSV(frame)
        edges = detect_edges(hsv)
        roi = region_of_interest(edges)
        line_segments = detect_line_segments(roi)
        lane_lines = average_slope_intercept(frame, line_segments)
        diff = 5
        with open("/sys/module/gpiod_driver/parameters/elapsed_ns", "r") as elapsed_file:
            time_diff = int(elapsed_file.read()) / 100000
        print(f'time_diff: {time_diff}')

        # if time_diff >= 28:
        #     motor_control.increment_speed_value(1000)
        # elif time_diff < 30 and time_diff > 5:
        #     motor_control.decrement_speed_value(1000)

        # Whenever we cannot determine a steering angle we default to the last steering angle
        steering_angle = get_steering_angle(frame, lane_lines, last_steering_angle)
        last_steering_angle = steering_angle

        heading_image = display_heading_line(frame, steering_angle)
        display_lines(frame, lane_lines)
        # cv2.imshow("heading line", heading_image)
  
        stopInterval = time.time() - lastStopTime
        if(stopInterval > 8 and detect_stopsign(frame)):
            lastStopTime = time.time()
            motor_control.stop()
            time.sleep(3)
            if(passedFirstStop):
                return
            else:
                motor_control.go_forward()
                passedFirstStop = True

        # if ((runCounter + 1) % stopSignCounter) == 0:
        #     if not passedFirstStop:
        #         isStop = detect_stopsign(frame)

        #         if isStop:
        #             print("Stopping")
        #             motor_control.stop()
        #             time.sleep(3)
        #             passedFirstStop = True

        #             secondStopWait = runCounter + 200

        #             stopSignCounter = 3

        #             motor_control.go_forward()
            
        #     elif passedFirstStop and runCounter > secondStopWait:
        #         isSecondStop = detect_stopsign(frame)
        #         if isSecondStop:
        #             print("Stopping second time")
        #             motor_control.stop()
        #             time.sleep(3)
        #             break

        now = time.time()
        dt = now - lastTime
        # print(f'Delta time: {dt}')
        deviation = steering_angle - 90

        # # Calculate the discrete integral
        # if len(queue) >= 20:
        #     queue.pop()
        # queue.insert(0, deviation)
        # deviation_int = np.mean(queue) # rolling average

        # deviation_dt = (deviation - last_deviation) / dt


        
        # print(f'Steering Angle: {steering_angle} | Deviation: {deviation} | Steering Angle {steering_angle}')
        error = max(24 - time_diff, 0.01)

        if deviation < DEAD_ZONE and deviation > -DEAD_ZONE:
            deviation = 0
            motor_control.steer_neutral()
        
        elif deviation > 5:
            motor_control.steer_right(0.3)

        elif deviation < -5:
            motor_control.steer_left(0.3)
        
        last_deviation = deviation

        derivative = kd * (error - lastError) / dt 
        proportional = kp * error
        pid = derivative + proportional
        print("pid", pid)
        motor_control.go_forward(pid)

        lastError = error
        lastTime = time.time()

        error_vals.append(error)
        p_vals.append(proportional)
        d_vals.append(derivative)
        speed_vals.append(motor_control.get_speed_value())
        steer_vals.append(motor_control.get_steer_value())
        
        key = cv2.waitKey(1)
        if key == 27:
            motor_control.stop()
            motor_control.steer_neutral()
            breaks
    
    
    write_list_to_file(error_vals, "error")
    write_list_to_file(p_vals, "p")
    write_list_to_file(d_vals, "d")
    write_list_to_file(speed_vals, "speed")
    write_list_to_file(steer_vals, "steer")
    video.release()
    cv2.destroyAllWindows()
    motor_control.steer_neutral()
    motor_control.stop()


if __name__ == "__main__":
    # test_image_checkpoints()
    run()