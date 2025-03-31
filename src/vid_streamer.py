import rclpy
import gi
import cv2
import numpy as np
import rclpy.callback_groups
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# import required library like Gstreamer and GstreamerRtspServer
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib

latest_frame = np.zeros((1280, 720, 3))

class VidStreamer(Node):
    def __init__(self):
        super().__init__('vid_streamer')
        stream_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self.subscription = self.create_subscription(
            Image,
            'image_annotated',  # Topic name must match the publisher's topic
            self.listener_callback,
            2)
        self.bridge = CvBridge()
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Vid Streamer Node Started')
        
        Gst.init(None)
        self.glib_server = GstServer()
        self.glib_loop = GLib.MainLoop()
        # self.glib_context = self.glib_loop.get_context()
        self.timer = self.create_timer(1, self.glib_loop.run, callback_group=stream_callback_group, autostart=False)
        self.timer.reset()
        

    def listener_callback(self, msg):
        # update current frame with new frame
        global latest_frame
        latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                    


# Sensor Factory class which inherits the GstRtspServer base class and add
# properties to it.
class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, **properties):
        super(SensorFactory, self).__init__(**properties)
        self.number_frames = 0
        self.fps = 15
        self.duration = 1 / self.fps * Gst.SECOND  # duration of a frame in nanoseconds
        # self.launch_string = 'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME ' \
        #                      'caps=video/x-raw,format=BGR,width={},height={},framerate={}/1 ' \
        #                      '! videoconvert ! video/x-raw,format=I420 ' \
        #                      '! x264enc speed-preset=ultrafast tune=zerolatency ' \
        #                      '! rtph264pay config-interval=1 name=pay0 pt=96' \
        #                      .format(1920, 1080, self.fps)
                                #  '! vp9enc ' \
                                #  '! rtpvp9pay' \
        self.launch_string = 'appsrc name=source is-live=true format=GST_FORMAT_TIME ' \
                             'caps=video/x-raw,format=BGR,width={},height={} ' \
                             '! videoconvert ' \
                             '! openh264enc ' \
                             '! rtph264pay config-interval=1 name=pay0 pt=96' \
                            .format(1280, 720, self.fps)
    # method to capture the video feed from the camera and push it to the
    # streaming buffer.
    def on_need_data(self, src, length):
        global latest_frame
          
            
        # It is better to change the resolution of the camera 
        # instead of changing the image shape as it affects the image quality.
        # frame = cv2.resize(frame, (1920, 1080), \
        #     interpolation = cv2.INTER_LINEAR)
        data = latest_frame.tostring()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        buf.duration = self.duration
        timestamp = self.number_frames * self.duration
        buf.pts = buf.dts = int(timestamp)
        buf.offset = timestamp
        self.number_frames += 1
        retval = src.emit('push-buffer', buf)

        if retval != Gst.FlowReturn.OK:
            print(retval)

    # attach the launch string to the override method
    def do_create_element(self, url):
        return Gst.parse_launch(self.launch_string)
    
    # attaching the source element to the rtsp media
    def do_configure(self, rtsp_media):
        self.number_frames = 0
        appsrc = rtsp_media.get_element().get_child_by_name('source')
        appsrc.connect('need-data', self.on_need_data)

# Rtsp server implementation where we attach the factory sensor with the stream uri
class GstServer(GstRtspServer.RTSPServer):
    def __init__(self, **properties):
        super(GstServer, self).__init__(**properties)
        self.factory = SensorFactory()
        self.factory.set_shared(True)
        self.set_service('8554')
        self.get_mount_points().add_factory('/stream', self.factory)
        self.attach(None)

# initializing the threads and running the stream on loop.

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    # Gst.init(None)
    # server = GstServer()
    # loop = GLib.MainLoop()
    # stream_serv = threading.Thread(loop.run)
    # stream_serv.start()


    node = VidStreamer()
    executor.add_node(node)
    executor.spin()
    
    node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
