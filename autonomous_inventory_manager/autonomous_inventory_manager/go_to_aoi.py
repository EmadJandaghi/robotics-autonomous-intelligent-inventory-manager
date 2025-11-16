import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time
from typing import List, Tuple
from autonomous_inventory_manager.trajectory_sender import TrajectorySender


class AOIFollower(Node):

    def __init__(self) -> None:
        super().__init__('aoi_follower')

        self.trajectory_sender = TrajectorySender(self)
        self.bridge = CvBridge()

        self.img_sub = self.create_subscription(
            Image,
            '/deepmind_robot1/deepmind_robot1_camera/image_raw',
            self.camera_callback,
            10)

        self.inventory_done_sub = self.create_subscription(
            Bool,
            '/inventory_done',
            self.inventory_done_callback,
            10)

        self.arrived_pub = self.create_publisher(Bool, '/aoi_arrived', 10)
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.aoi_choice_pub = self.create_publisher(String, '/aoi_choice', 10)

        # Internal variables used to track state
        self.aoi_reached  = False
        self.inv_done = False
        self.aoi_choice = None
        self.aoi_detected = False
        self.last_detection_time = None 
        self.head_initialized = False  

        # Mission list
        self.aoi_sequence = ["yellow", "green", "red", "blue"]
        self.current_aoi_index = 0

        self.publish_next_aoi()

        self.timer = self.create_timer(0.1, self.controller_loop)

    def controller_loop(self) -> None:
        if not self.inv_done and not self.aoi_reached:
            self.proc_img_go_aoi()
            self.get_logger().info("processing image and going to aoi.")

        elif not self.inv_done and self.aoi_reached:
            self.arrived_pub.publish(Bool(data=True))
            # self.get_logger().info("Arrived at the area of interest! Publishing arrival and waiting for the inventory checking done.")
        
        elif self.inv_done and self.aoi_reached:
            self.reposition()
            self.inv_done = False
            self.aoi_reached = False
            self.current_aoi_index += 1
            self.publish_next_aoi()
            self.head_initialized = False
            self.arrived_pub.publish(Bool(data=False))

    def publish_next_aoi(self):
        """Publish the next AOI color in the sequence."""
        if self.current_aoi_index < len(self.aoi_sequence):
            self.aoi_choice = self.aoi_sequence[self.current_aoi_index]
            msg = String()
            msg.data = self.aoi_choice
            self.aoi_choice_pub.publish(msg)
            self.get_logger().info(f"New AOI target: {self.aoi_choice}")
        else:
            self.get_logger().info("All AOIs completed — shutting down.")
            self.stop_robot()
            rclpy.shutdown()

    def inventory_done_callback(self, msg: Bool):
            self.inv_done = msg.data
            self.get_logger().info("Inventory DONE")


    def tilt_head_down(self):
        if not self.head_initialized:
            self.get_logger().info("Tilting head down …")
            self.trajectory_sender.send_trajectory(
                ['deepmind_robot1_head_base_joint', 'deepmind_robot1_head_joint'],
                [0.0, 0.9], 1)
            time.sleep(2)
            self.head_initialized = True


    def aoi_callback(self, msg: String) -> None:
        self.aoi_choice = msg.data


    def camera_callback(self, msg: Image) -> None:
        try:

            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:

            self.get_logger().info(f"Error converting image: {e}")
            return

    def proc_img_go_aoi(self):
        if self.cv_image is None:
            return
        cv_image = self.cv_image.copy()
        self.tilt_head_down()

        purple_bounds = ([120, 40, 40], [160, 255, 255])
        blue_bounds   = ([100, 150, 0], [140, 255, 255])
        red_bounds = ([170, 150, 50], [180, 255, 255])
        yellow_bounds = ([20, 100, 100], [35, 255, 255])
        green_bounds  = ([35, 50, 50], [90, 255, 255])

        # Get image dimensions
        height, width, _ = cv_image.shape

        # Crop two parts of the image: bottom (for line) and top (for AOI).
        crop_img, crop_top_img = self.crop_image(cv_image, height, width)

        # Convert both cropped regions to HSV.
        # Bottom HSV → used for purple path detection
        # Top HSV → used for AOI color detection
        hsv, hsv_detect_aoi = self.convert_to_hsv(crop_img, crop_top_img)

        # ----------------------------------------------------------
        # Detect the purple line in the bottom cropped image.
        # Returns the masked visualization (res) and the contours.
        # ----------------------------------------------------------
        res, purple_contours = self.detect_contours(hsv, purple_bounds[0], purple_bounds[1], crop_img)

        # ----------------------------------------------------------
        # If user chose an AOI color, search for corresponding contours
        # in the TOP cropped region of the image.
        # ----------------------------------------------------------
        aoi_choice_lower = (self.aoi_choice or "").strip().lower()

        if aoi_choice_lower == 'blue':
            res2, blue_contours = self.detect_contours(hsv_detect_aoi, blue_bounds[0], blue_bounds[1], crop_top_img)
            self.aoi_detected = self.search_aoi('blue', blue_contours, res2)

        elif aoi_choice_lower == 'red':
            res2, red_contours = self.detect_contours(hsv_detect_aoi, red_bounds[0], red_bounds[1], crop_top_img)
            self.aoi_detected = self.search_aoi('red', red_contours, res2)

        elif aoi_choice_lower == 'yellow':
            res2, yellow_contours = self.detect_contours(hsv_detect_aoi, yellow_bounds[0], yellow_bounds[1], crop_top_img)
            self.aoi_detected = self.search_aoi('yellow', yellow_contours, res2)

        elif aoi_choice_lower == 'green':
            res2, green_contours = self.detect_contours(hsv_detect_aoi, green_bounds[0], green_bounds[1], crop_top_img)
            self.aoi_detected = self.search_aoi('green', green_contours, res2)
        # --------------------------------------------------------------------
        # Compute the centroid (center point) of each purple contour detected.
        # This provides x,y coordinates for path-following.
        # --------------------------------------------------------------------
        centres = []
        for i in range(len(purple_contours)):
            moments = cv2.moments(purple_contours[i])
            try:
                centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
                cv2.circle(res, centres[-1], 10, (0, 255, 0), -1)
            except ZeroDivisionError:
                pass

        if len(centres) < 1 and self.aoi_detected:
            self.go_to_aoi(hsv_detect_aoi, crop_top_img, width)
            return

        # ensure centres is not empty before fining the max, min centroids
        if centres:
            most_right_centroid = max(centres, key=lambda x: x[0])
            most_left_centroid = min(centres, key=lambda x: x[0])
        else: # no centroid -> exit
            self.get_logger().error("No purple contours detected. Cannot find centroids.")
            return

        # Verify if the color choice is valid ("blue", "red", "yellow", "green").
        self.is_valid_aoi_choice()


        if (self.is_valid_aoi_choice() and self.aoi_detected):
            cx, cy = self.get_centroid_coordinates(most_left_centroid, height, width)
        else:
            cx, cy = self.get_centroid_coordinates(most_right_centroid, height, width)

        # Draw a red circle to indicate the currently selected centroid (the one used for steering).
        cv2.circle(res, (int(cx), int(cy)), 5, (0, 0, 255), -1)

        cv2.imshow("RES", res)
        cv2.waitKey(1)

        # Compute steering error.
        # The error is how far (in pixels) the selected centroid (cx) is from the center of the image.
        # Positive error means the line is to the right; the robot must turn right.
        # Negative error means the line is to the left; the robot must turn left.
        error_x = cx - width / 2

        # Publish motion commands based on computed error.
        self.pub_velocities(error_x)


    def crop_image(self, cv_image: np.ndarray, height: int, width: int) -> Tuple[np.ndarray, np.ndarray]:
        """ Crop image, we create two cropped image
            One for the line follower and one to access the AOI """
        rows_to_watch = 20

        crop_img_start_row = height * 3 // 4
        crop_img_end_row = min(height, crop_img_start_row + rows_to_watch)
        crop_img = cv_image[crop_img_start_row:crop_img_end_row, 0:width]

        crop_aoi_img_start_row = height // 3
        crop_aoi_img_end_row = min(height, height * 2 // 3)
        crop_aoi_img_start_col = width // 8
        crop_aoi_img_end_col = min(width, width // 2)
        crop_aoi_img = cv_image[crop_aoi_img_start_row:crop_aoi_img_end_row, crop_aoi_img_start_col:crop_aoi_img_end_col]

        return crop_img, crop_aoi_img


    def convert_to_hsv(self, crop_img: np.ndarray, crop_top_img: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Converts both cropped BGR images to HSV color space.
        HSV (Hue, Saturation, Value) is better for color segmentation
        because hue represents color type directly and is less affected by light.
        """
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        hsv_detect_aoi = cv2.cvtColor(crop_top_img, cv2.COLOR_BGR2HSV)
        return hsv, hsv_detect_aoi


    def detect_contours(self, hsv: np.ndarray, lower_bound: List[int], upper_bound: List[int], image: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:
        """
        Applies a color filter in HSV space and finds contours of detected regions.
        Used for both line and AOI detection.
        Returns:
            res  → image masked by color (for visualization)
            contours → list of contour boundaries (arrays of (x, y) points)
        """
        # Create binary mask for specified color range.
        mask = cv2.inRange(hsv, np.array(lower_bound), np.array(upper_bound))
        # Visualize masked areas.
        res = cv2.bitwise_and(image, image, mask=mask)
        # Find contour outlines of white (255) regions in the mask.
        # RETR_CCOMP retrieves both outer and inner boundaries; TC89_L1 simplifies the contour points.
        contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        return res, contours


    def is_recently_detected(self) -> bool:
        """Determines if aoi was recently detected """
        if self.last_detection_time is not None:
            current_time = time.time()
            elapsed_time = current_time - self.last_detection_time
            if elapsed_time < 10:
                self.get_logger().warning(f'is_recently_detected < 10')
                return True
        return False

    def search_aoi(self, aoi_color: str, contours: List[np.ndarray], res: np.ndarray) -> bool:
        """
        Determines whether the chosen AOI color (blue/red/yellow/green) is currently visible.
        """
        if contours:
            # AOI color contours found → AOI detected
            self.get_logger().warning(f'{aoi_color.capitalize()} AOI detected')
            self.aoi_detected = True
            self.last_detection_time = time.time()  # record detection time
            # Show visualization for debugging
            cv2.imshow("RES2", res)
            cv2.waitKey(1)
        else:
            # If no AOI contours, check if it was seen recently
            if self.is_recently_detected():
                self.aoi_detected = True
            else:
                self.get_logger().warning(f'Still searching for the {aoi_color} AOI')
                self.aoi_detected = False
        return self.aoi_detected

    def is_valid_aoi_choice(self) -> bool:
        """
        Validates that the user-specified AOI color is acceptable.
        If not, logs a warning message.
        """
        valid_aoi_colors = ['blue', 'red', 'yellow', 'green']
        if self.aoi_choice in valid_aoi_colors:
            return True
        else:
            self.get_logger().warning("Please choose between 'blue', 'red', 'yellow', or 'green' for the AOI color.")
            return False


    def get_centroid_coordinates(self, centroid: Tuple[int, int], height: int, width: int) -> Tuple[int, int]:
        """
        Safely extracts the x, y coordinates of a centroid tuple.
        If extraction fails (invalid centroid), defaults to image center.
        """
        try:
            cx = centroid[0]
            cy = centroid[1]
        except:
            cy, cx = height/2, width/2
        return cx, cy

    def stop_robot(self) -> None:
        twist_object = Twist()
        twist_object.linear.x = 0.0
        twist_object.angular.z = 0.0
        self.pub_vel.publish(twist_object)

    def go_to_aoi(self, hsv_detect_aoi: np.ndarray, cv_image: np.ndarray, width: int) -> None:
        """ Ensure that the robot positions itself accurately to one of the four key locations of interest"""
        # Find the blob of interest
        blob_bounds = None
        if self.aoi_choice == 'yellow':
            blob_bounds = ([20, 100, 50], [35, 255, 255])
        elif self.aoi_choice == 'green':
            blob_bounds = ([45, 100, 50], [85, 255, 255])
        elif self.aoi_choice == 'blue':
            blob_bounds = ([90, 150, 50], [125, 255, 255])
        elif self.aoi_choice == 'red':
            blob_bounds = ([170, 150, 50], [180, 255, 255])

        if blob_bounds:
            _, contours = self.detect_contours(hsv_detect_aoi, blob_bounds[0], blob_bounds[1], cv_image)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                moments = cv2.moments(largest_contour)
                try:
                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])

                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)  # Red dot at the centroid
                    self.pub_aoi_velocities(cx, width)

                    cv2.imshow("Centroid Visualization", cv_image)
                    cv2.waitKey(1)
                except ZeroDivisionError:
                    self.get_logger().error("Zero division error while calculating centroid.")
                    self.stop_robot()
            else:
                self.get_logger().info("Arrived at the area of interest!")
                self.stop_robot()
                self.aoi_reached = True

        else:
            self.get_logger().error("Invalid aoi choice. Cannot determine blob bounds.")
            self.stop_robot()

    def pub_aoi_velocities(self, cx: int, width: int) -> None:
        """ Publish aoi velocities based on the centroid and AOI color choice """
        linear_vel = 0.2
        angular_vel = 0.07
        error_x = cx - width / 2
        twist_object = Twist()
        twist_object.linear.x = linear_vel
        angular_velocity = -error_x / 100

        # Ensure angular velocity is within a certain range
        if angular_velocity < -0.3:
            angular_velocity = -angular_vel
        elif angular_velocity > 0.3:
            angular_velocity = angular_vel

        twist_object.angular.z = angular_velocity
        self.get_logger().info(f"Navigating to {self.aoi_choice} aoi: Adjusting angular velocity ---> {twist_object.angular.z:.2f}")
        self.pub_vel.publish(twist_object)


    def pub_velocities(self, error: float) -> None:
        twist_object = Twist()
        twist_object.linear.x = 0.2

        # Proportional controller for steering
        angular_velocity = -error / 100

        # Clamp the angular velocity to safe limits.
        if angular_velocity < -0.35:
            angular_velocity = -0.35
        elif angular_velocity >= 0.35:
            angular_velocity = 0.35

        twist_object.angular.z = angular_velocity

        self.get_logger().info(f"Adjusting the angular velocity ---> {twist_object.angular.z:.2f}")
        # Publish Twist message to command the robot.
        self.pub_vel.publish(twist_object)

    def reposition(self):
        """Reacquire the purple line by rotating and tilting head."""
        self.get_logger().info("Repositioning: rotating and tilting head.")
        twist = Twist()
        twist.angular.z = -0.8
        self.pub_vel.publish(twist)
        time.sleep(3)
        self.stop_robot()
        self.get_logger().info("Reposition complete.")


def main(args=None) -> None:
    rclpy.init(args=args)
    go_to_aoi = AOIFollower()
    try:
        rclpy.spin(go_to_aoi)
    except KeyboardInterrupt:
        pass
    go_to_aoi.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()