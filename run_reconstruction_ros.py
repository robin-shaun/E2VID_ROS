from utils.loading_utils import load_model, get_device
import numpy as np
import argparse
from utils.inference_utils import events_to_voxel_grid_pytorch
from utils.timers import Timer
from image_reconstructor import ImageReconstructor
from options.inference_options import set_inference_options
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from event_array_pb2 import Event_Array
import zmq

class E2VID_ROS:
    def __init__(self):
        rospy.init_node("e2vid")
        self.frame_pub = rospy.Publisher("/e2vid/image", Image, queue_size=1)
        self.width = 346
        self.height = 260
        self.frequency = 20
        self.event_window_size = 30000
        self.event_window = np.ones((4, self.event_window_size))
        # self.event_queue = Queue(self.event_window_size)
        # self.self.event_window = self.event_window.tolist()
        self.event_array = Event_Array()
        print('Sensor size: {} x {}'.format(self.width, self.height))
        parser = argparse.ArgumentParser(description='Evaluating a trained network')
        set_inference_options(parser)
        self.args = parser.parse_args()

        # address = ('127.0.0.1', 50009)
        # self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # try:
        #     self.socket.connect(address)
        # except Exception:
        #     print('[!] Server not found ot not open')
        #     sys.exit()

        context = zmq.Context()
        self.socket =  context.socket(zmq.REQ)
        self.socket.connect('tcp://localhost:10001')

        self.bridge = CvBridge()

        # Load model
        self.model = load_model("pretrained/E2VID.pth.tar")
        self.device = get_device(self.args.use_gpu)
        model = self.model.to(self.device)
        model.eval()

        self.reconstructor = ImageReconstructor(model, self.height, self.width, self.model.num_bins, self.args)

        self.rate = rospy.Rate(self.frequency)
        
    def loop(self):
        start_index = 0
        start_time = time.time()
        while (not rospy.is_shutdown()):
            # self.socket.send(b'Hello')
            # data = self.socket.recv(600000)
            # if (data == "Not ready".encode()):
            #     print("Not ready")
            #     continue
            print(time.time()-start_time)
            start_time = time.time()
            self.socket.send_string('Hello')
            data = self.socket.recv()
            if (data == "Not ready".encode()):
                print("Not ready")
                continue
            self.event_array.ParseFromString(data)
            self.event_window[0,:] = np.asarray(self.event_array.timestamp)
            self.event_window[1,:] = np.asarray(self.event_array.x)
            self.event_window[2,:] = np.asarray(self.event_array.y)
            self.event_window[3,:] = np.asarray(self.event_array.polarity)
            last_timestamp = self.event_window[0, -1]
            event_tensor = events_to_voxel_grid_pytorch(self.event_window.transpose(),
                                                            num_bins=self.model.num_bins,
                                                            width=self.width,
                                                            height=self.height,
                                                            device=self.device)

            num_events_in_window = self.event_window.shape[0]

            
            out = self.reconstructor.update_reconstruction(event_tensor, start_index + num_events_in_window, last_timestamp)
            
            reconstructed_image = self.bridge.cv2_to_imgmsg(out, encoding="passthrough")
            self.frame_pub.publish(reconstructed_image)
            start_index += num_events_in_window
            
            self.rate.sleep()


if __name__ == "__main__":
    e2vid = E2VID_ROS()
    e2vid.loop()



