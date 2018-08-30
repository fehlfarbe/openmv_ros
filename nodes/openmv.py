#!/usr/bin/env python
import rospy
import pyopenmv
from time import sleep
from threading import Thread
from sensor_msgs.msg import Image

script = """
# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor, image, time

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

while(True):
    img = sensor.snapshot()         # Take a picture and return the image.
    sensor.flush()
"""


class OpenMVCam:

    def __init__(self, port="/dev/openmvcam"):
        self.port = port
        self.connected = False

        pyopenmv.disconnect()
        rospy.loginfo("Connecting...")
        for i in range(10):
            try:
                # opens CDC port.
                # Set small timeout when connecting
                pyopenmv.init(self.port, baudrate=921600, timeout=0.050)
                self.connected = True
                break
            except Exception as e:
                self.connected = False
                sleep(0.100)

        if not self.connected:
            rospy.logerr("Failed to connect to OpenMV's serial port.\n"
                        "Please install OpenMV's udev rules first:\n"
                        "sudo cp openmv/udev/50-openmv.rules /etc/udev/rules.d/\n"
                        "sudo udevadm control --reload-rules\n\n")
            return
        rospy.loginfo("Connected!")
        # Set higher timeout after connecting for lengthy transfers.
        pyopenmv.set_timeout(1*2)  # SD Cards can cause big hicups.
        pyopenmv.stop_script()
        pyopenmv.enable_fb(True)
        pyopenmv.exec_script(script)

        # init pub
        self.pub_image = rospy.Publisher("/image", Image, queue_size=10)

        # init thread
        self.running = False
        self.thread = Thread(target=self.run)
        self.thread.start()

        rospy.on_shutdown(self.shutdown)

    def run(self):
        self.running = True
        while self.running:
            # read framebuffer
            # rospy.loginfo("try to read fb...")
            fb = pyopenmv.fb_dump()

            if fb is None:
                continue

            w, h, data = fb
            # create Image message
            img = Image()
            img.header.stamp = rospy.Time.now()
            img.header.frame_id = "openmv_cam"
            img.width = w
            img.height = h
            img.data = list(data.flat[0:])
            img.step = w * 3
            img.encoding = "rgb8"
            # publish it
            self.pub_image.publish(img)

    def shutdown(self):
        rospy.loginfo("Shutting down...")
        self.running = False
        if self.thread is not None:
            self.thread.join()


if __name__ == '__main__':
    rospy.init_node("openmv_cam")
    cam = OpenMVCam()
    rospy.spin()
