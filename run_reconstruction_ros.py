from utils.loading_utils import load_model, get_device
import numpy as np
import argparse
from utils.inference_utils import events_to_voxel_grid_pytorch
from image_reconstructor import ImageReconstructor
from options.inference_options import set_inference_options
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from event_array_pb2 import Event_Array
import zmq

class E2VID_ROS:
    def __init__(self):
        rospy.init_node("e2vid")
        self.frame_pub = rospy.Publisher("/e2vid/image", Image, queue_size=1)
        self.width = 346
        self.height = 260
        self.event_window_size = 30000
        self.camera_link = "camera_link"
        self.event_window = np.ones((4, self.event_window_size))
        self.event_array = Event_Array()
        print('Sensor size: {} x {}'.format(self.width, self.height))
        parser = argparse.ArgumentParser(description='Evaluating a trained network')
        set_inference_options(parser)
        self.args = parser.parse_args()

        context = zmq.Context()
        self.socket =  context.socket(zmq.REQ)
        self.socket.connect('tcp://127.0.0.1:10001')

        self.bridge = CvBridge()

        # Load model
        self.model = load_model("pretrained/E2VID.pth.tar")
        self.device = get_device(self.args.use_gpu)
        model = self.model.to(self.device)
        model.eval()

        self.reconstructor = ImageReconstructor(model, self.height, self.width, self.model.num_bins, self.args)
        
    def loop(self):
        start_index = 0
        while (not rospy.is_shutdown()):
            self.socket.send_string('Hello')
            data = self.socket.recv()
            if (data == "Not ready".encode()):
                continue
            self.event_array.ParseFromString(data)
            self.event_window[0,:] = np.array(list(self.event_array.timestamp))
            self.event_window[1,:] = np.array(list(self.event_array.x))
            self.event_window[2,:] = np.array(list(self.event_array.y))
            self.event_window[3,:] = np.array(list(self.event_array.polarity))
            last_timestamp = self.event_window[0, -1]
            
            event_tensor = events_to_voxel_grid_pytorch(self.event_window.transpose(),
                                                            num_bins=self.model.num_bins,
                                                            width=self.width,
                                                            height=self.height,
                                                            device=self.device)

            num_events_in_window = self.event_window.shape[0]

            
            out = self.reconstructor.update_reconstruction(event_tensor, start_index + num_events_in_window, last_timestamp)
            reconstructed_image = self.bridge.cv2_to_imgmsg(out, encoding="passthrough")
            reconstructed_image.header.stamp = rospy.Time(last_timestamp)
            reconstructed_image.header.frame_id = self.camera_link
            self.frame_pub.publish(reconstructed_image)
            start_index += num_events_in_window


if __name__ == "__main__":
    e2vid = E2VID_ROS()
    e2vid.loop()



