import rospy
from dvs_msgs.msg import EventArray
from sensor_msgs.msg import Image
from queue import Queue
import numpy as np
import time

class E2VID_ROS:
    def __init__(self):
        rospy.init_node("e2vid")
        self.event_sub = rospy.Subscriber("/dvs/events", EventArray, self.event_callback, queue_size=1)
        self.frame_pub = rospy.Publisher("/e2vid/image", Image, queue_size=1)
        event_window_size = 30000
        self.event_queue = Queue(event_window_size)
        self.start_time = time.time()
        self.rate = rospy.Rate(30)

    def loop(self):
        start_index = 0
        while (not rospy.is_shutdown()):
            event_window = list(self.event_queue.queue)
            if (event_window == []):
                continue
            event_window = np.asarray(event_window)


            self.rate.sleep()

    def event_callback(self, msg):
        print(time.time() -self.start_time)
        self.start_time = time.time()
        # if (len(self.event_window)>self.event_window_size):
        # print("dvs")
        # for event in msg.events:
        #     if self.event_queue.full():
        #         self.event_queue.get()
        #         self.event_queue.put([event.ts, event.x, event.y, event.polarity])
        #     else:
        #         self.event_queue.put([event.ts, event.x, event.y, event.polarity])
        
if __name__ == "__main__":
    e2vid = E2VID_ROS()
    e2vid.loop()



