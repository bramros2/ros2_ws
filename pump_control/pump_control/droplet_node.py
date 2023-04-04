import rclpy
import sys
import time

from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from .flow_detector import *


class DropletDetector(Node):
    """
        This node subscribes to the "/image_raw" topic and processes the images by detecting droplets in them.
        It publishes the droplet size on the "droplet_size" topic, the mask image on the "image/mask" topic, and the flow
        image on the "image/flow" topic.
        """

    def __init__(self, thr_min: int, thr_max: int, blur: int = 15,
                 blob_params=None, detection_window=None):
        """
        Initializes the DropletDetector node.

        Parameters:
        thr_min: The minimum threshold for blob detection.
        thr_max: The maximum threshold for blob detection.
        blur: The blurring kernel size. Defaults to 15.
        blob_params: The parameters for blob detection. Defaults to None.
        detection_window: The detection window in the image (in [x_min, y_min, x_max, y_max]). Defaults to None.
        """

        super().__init__('droplet_detector')
        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window

        self._t0 = time.time()

        print(">> Publishing image to topic image/flow")
        print(">> Publishing mask to topic image/mask")
        self.image_pub = self.create_publisher(Image, "image/flow", 1)
        self.mask_pub = self.create_publisher(Image, "image/mask", 1)
        print(">> Publishing droplet size to topic droplet_size")
        self.droplet_pub = self.create_publisher(Float64, "droplet_size", 1)

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, "/image_raw", self.callback, 1)
        print("<< Subscribed to topic /image_raw")


    def set_threshold(self, thr_min, thr_max):
        """
        Sets the threshold for blob detection.

        Parameters:
        thr_min: The minimum threshold for blob detection.
        thr_max: The maximum threshold for blob detection.
        """
        self._threshold = [thr_min, thr_max]

    def set_blur(self, blur):
        """
        Sets the threshold for blob detection.

        Parameters:
        blur: Sets value for Gaussian blur
        """
        self._blur = blur

    def set_blob_params(self, blob_params):
        """
        Set the blob detection parameters.

        Args:
        blob_params (dict): Blob detection parameters.
        """
        self._blob_params = blob_params

    def callback(self, data):
        """
        Callback function for image subscription.

        Args:
        data (Image): Incoming image data.
        """

        # Log that an image is being received
        self.get_logger().info("Received image from topic /image_raw")

        try:
            # Convert ROS image message to OpenCV image. Encode image in bgr8
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            # Log error and exit callback if conversion fails
            print(e)
            return  # Exit callback on error

        # Check if the image is large enough for processing (320x240)
        (rows, cols, channels) = cv_image.shape
        if cols > 60 and rows > 60:
            # Call blob_detect function to detect droplets in the image
            # using specified parameters such as thresholds and blur
            keypoints, mask = blob_detect(cv_image, self._threshold[0], self._threshold[1], self._blur,
                                          blob_params=self._blob_params, search_window=self.detection_window)
            # Blur the area outside the detection window to create a focus
            cv_image = blur_outside(cv_image, 10, self.detection_window)
            # Draw the detection window as a rectangle
            cv_image = draw_window(cv_image, self.detection_window, line=1)
            # Draw a frame around the image
            cv_image = draw_frame(cv_image)
            # Draw circles around the detected droplets
            cv_image = draw_keypoints(cv_image, keypoints)

            try:
                # Show the image with the search window and droplets overlaid on top
                cv2.imshow("Frame overlay", cv_image)
                # Show image with the threshold mask applied to the original Image
                cv2.imshow("Mask", mask)
                cv2.waitKey(1)
            except CvBridgeError as e:
                print(e)

        # Calculate the average size of the detected droplets
        result = cv_image.copy()
        sum = 0
        count = 0
        # Iterate through all keypoints and draw circles
        for kp in keypoints:
            x, y = kp.pt
            r = kp.size / 2
            x, y, r = int(x), int(y), int(r)
            cv2.circle(result, (x, y), r, (0, 255, 0), thickness=2)
            sum += kp.size
            count += 1

        
        if count != 0:
            # Calculate average
            avg = sum / count

            # Log average diameter
            msgdata = 'Average diameter size (pixels) =  %s' % (avg)
            self.get_logger().info(" %s " % (msgdata))

            # Publish average diameter to topic "droplet_size"
            droplet = Float64()
            droplet.data = avg
            self.droplet_pub.publish(droplet)

            # Show the image with circles drawn around the detected droplets
            cv2.imshow("result", result)
            cv2.waitKey(1)
            
        fps = 1.0/(time.time()-self._t0)
        self._t0 = time.time()

def main(args=None):
    '''
    Main function that gets called when script is executed.
    Initialises parameters and creates instance of the DropletDetector class.

    Currently reads params from hardcoded config file; TODO: Change to some other form

    '''

    if args is None:
        args = sys.argv
    
    #with open("/home/bram/ros2_ws2/pump_control/pump_control/config.txt", 'r') as file:
    #    config_dict = {}
    #    for line in file:
    #        if line[0] != '#':  #ignores comments
    #            line = line.strip('\n')
    #            line = line.split()
    #            for elem in line:
    #                if elem.isdigit():
    #                    value = int(elem)                   #MAKE SURE NO FLOATS ARE NEEDED FOR VALUES IN SETTINGS
    #                elif elem != '=':    #ignores the = sign
    #                    identifier = elem
    #            config_dict[identifier] = value
    #hsv_min = (config_dict['hmin'],config_dict['smin'],config_dict['vmin'])
    #hsv_max = (config_dict['hmax'],config_dict['smax'],config_dict['vmax'])

    hmin = 160
    smin = 43
    vmin = 113
    hmax = 172
    smax = 103
    vmax = 159

    hsv_min = (hmin,smin,vmin)
    hsv_max = (hmax,smax,vmax)

    blur =  5# config_dict['blur']
    min_size = 10 #config_dict['min_size']
    max_size = 40 #config_dict['max_size']

    #--- detection window respect to camera frame in [x_min, y_min, x_max, y_max] adimensional (0 to 1)
    x_min   = 0.0
    x_max   = 1.0
    y_min   = 0.0
    y_max   = 1.0

    detection_window = [x_min, y_min, x_max, y_max]

    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 100;
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 20
    params.maxArea = 20000
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
     
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.2
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.7   

    rclpy.init(args=args)

    drop_detector = DropletDetector(hsv_min, hsv_max, blur, params, detection_window)
    
    rclpy.spin(drop_detector)

    drop_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



