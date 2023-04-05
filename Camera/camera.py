import image, sensor, time
from math import atan2, sqrt, degrees
from pyb import UART

ROI = (77, 0, 180, 159)

CENTRE_X = 91
CENTRE_Y = 66

MAX_VALID_RADIUS = 87

YELLOW_THRESHOLD = (60, 89, -36, 6, 49, 94)
BLUE_THRESHOLD = (35, 65, -7, 26, -68, -48)

DEBUG_WHITEBALANCE = False
DEBUG_PRINTING = True

DRAW_CROSSES = True
DRAW_RECTANGLES = True
DRAW_CIRCLES = False

NO_GOAL_ANGLE = 400

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
#sensor.set_windowing(ROI)
sensor.skip_frames(time=100)

if DEBUG_WHITEBALANCE:
    sensor.set_auto_whitebal(True)
    print("Current white balance values: {}".format(sensor.get_rgb_gain_db()))
else:
    sensor.set_auto_whitebal(False, rgb_gain_db=(-4.87865, -6.02073, -3.65883))

if not DEBUG_WHITEBALANCE:
    sensor.set_brightness(0)
    sensor.set_contrast(3)
    sensor.set_saturation(2)
    sensor.set_auto_exposure(False, exposure_us=10000)
    sensor.set_auto_gain(False, gain_db=15)
    sensor.skip_frames(time=500)

uart = UART(3, 115200, timeout_char=10)

def send(data):
    sendData = [0x80]

    for num in data:
        num = round(num)
        sendData.append((num >> 7) & 0x7F)
        sendData.append(num & 0x7F)

    for num in sendData:
        try:
            uart.writechar(num)
        except:
            pass

def calculateAngleDistance(object, img):
    dx = object.cx() - CENTRE_X
    dy = object.cy() - CENTRE_Y

    angle = (450 - degrees(atan2(dy, dx))) % 360
    distance = (sqrt(dx**2 + dy**2))
    return (angle, distance)

def sortBlobs(blobs, img):
    if len(blobs) > 0:
        for blob in sorted(blobs, key=lambda x: x.pixels(), reverse = True):
            angle, distance = calculateAngleDistance(blob, img)
            if distance < MAX_VALID_RADIUS:
                if DRAW_CROSSES:
                    img.draw_cross(blob.cx(), blob.cy())
                if DRAW_RECTANGLES:
                    img.draw_rectangle(blob.rect())
                return (angle, distance)

    return (NO_GOAL_ANGLE, 0)

while True:
    img = sensor.snapshot()

    if DRAW_CIRCLES:
        img.draw_circle(CENTRE_X, CENTRE_Y, MAX_VALID_RADIUS)

    if DRAW_CROSSES:
        img.draw_cross(CENTRE_X, CENTRE_Y)

    yellowBlobs = img.find_blobs([YELLOW_THRESHOLD], x_stride=5, y_stride=5, area_threshold=200, pixel_threshold=200, merge=True, margin=23)
    blueBlobs = img.find_blobs([BLUE_THRESHOLD], x_stride=5, y_stride=5, area_threshold=200, pixel_threshold=200, merge=True, margin=23)

    yellowAngle, yellowDistance = sortBlobs(yellowBlobs, img)
    blueAngle, blueDistance = sortBlobs(blueBlobs, img)

    if DEBUG_PRINTING:
        print ([yellowAngle, yellowDistance, blueAngle, blueDistance])

    send([yellowAngle, yellowDistance, blueAngle, blueDistance])
