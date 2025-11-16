import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class LineFollower(Node):
    """
    This class defines an optimized ROS2 node for a color-based line follower robot.

    Purpose:a to detect a specific colored line (here, purple)
      - The robot uses its onboard camer
        on the ground and follows it autonomously.
      - It continuously processes camera frames to find the line position and
        adjusts its movement commands in real-time to stay centered on the line.

   
      1. Detects multiple purple regions (contours) rather than treating all as one blob.
      2. Calculates centroids for each region and intelligently selects the "most relevant" one.
      3. Uses optimized contour approximation (`CHAIN_APPROX_TC89_L1`) for faster and smoother shape analysis.
      4. Provides clearer visualization (green = all centroids, red = selected centroid).
      5. More robust against partial line loss, noise, or intersections.

    In short:
        - The camera looks down at the floor.
        - The image is cropped to the bottom section (where the line is expected).
        - The purple line is isolated using color filtering (HSV).
        - The contours (boundaries) of all purple blobs are found.
        - Each blob’s centroid is computed.
        - The algorithm picks the rightmost centroid (most likely the line to follow).
        - The robot turns toward that centroid to stay aligned with the line.
    """

    def __init__(self) -> None:
        super().__init__('line_follower')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/deepmind_robot1/deepmind_robot1_camera/image_raw',
            self.camera_callback,
            10)

        # geometry_msgs/Twist
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)


    def camera_callback(self, msg: Image) -> None:
        """
        This callback is triggered every time a new image is received from the camera.

        Step-by-step overview:
          1. Convert ROS image message → OpenCV image
          2. Crop the bottom region of the frame (where the line appears)
          3. Convert color space from BGR → HSV (better for color segmentation)
          4. Apply color threshold to detect purple regions
          5. Use contour detection to find separate purple blobs
          6. Compute the centroid (center) of each contour
          7. Select one centroid (the rightmost one)
          8. Compute steering error (difference between centroid and image center)
          9. Publish movement commands to keep the robot centered
        """

        try:
            # "bgr8" → 8-bit OpenCV standard.
            # conversion ROS images (sensor_msgs/Image) OpenCV images (numpy arrays).
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            # fails for bad frame or wrong encoding
            self.get_logger().info(f"Error converting image: {e}")
            return

        # - -> number of color channels (3 for BGR)
        height, width, _ = cv_image.shape

        # Only process the lower part of the image.
        rows_to_watch = 20  # number of vertical rows to include in the cropped section

        # Crop the bottom horizontal strip from the image.
        crop_img = cv_image[height*3//4:height*3//4 + rows_to_watch][1:width]

        # Convert cropped image from BGR to HSV color space.
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the lower and upper color thresholds for detecting purple.
        lower_purple = np.array([120, 40, 40])  # dull/dark purple lower bound
        upper_purple = np.array([160, 255, 255])  # bright purple upper bound

        # Create a binary mask
        mask = cv2.inRange(hsv, lower_purple, upper_purple)

        # Apply the binary mask to the cropped image for visualization.
        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        # ----------------------------------------------------------------------
        #                 NEW FEATURE: CONTOUR DETECTION
        # ----------------------------------------------------------------------
        # Find contours around connected white areas in the mask.
        #
        # cv2.RETR_CCOMP: retrieves all contours and groups them into two levels
        #                 (external and internal boundaries). Useful if the line
        #                 has holes or inner shapes.
        #
        # cv2.CHAIN_APPROX_TC89_L1: an advanced contour approximation method
        #                            (Teh-Chin chain algorithm). It reduces
        #                            the number of stored points per contour,
        #                            improving performance and smoothing edges.
        contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)

        # Log the number of contours (blobs) found.
        # helps debug whether the camera sees multiple purple segments.
        self.get_logger().info("Number of centroids==>" + str(len(contours)))

        # Prepare a list to store centroid coordinates of each detected contour.
        centres = []

        # ----------------------------------------------------------------------
        #           COMPUTE THE CENTROID OF EACH DETECTED purple BLOB
        # ----------------------------------------------------------------------
        for i in range(len(contours)):
            # Compute image moments for each contour.
            moments = cv2.moments(contours[i])
            try:
                # Calculate centroid coordinates of the contour.
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                
                centres.append((cx, cy))

                # Draw a green circle on the visualization image for each centroid.
                cv2.circle(res, centres[-1], 10, (0, 255, 0), -1)

            except ZeroDivisionError:
                pass

        # ----------------------------------------------------------------------
        #           SELECT THE "MOST RELEVANT" CENTROID TO FOLLOW
        # ----------------------------------------------------------------------
        # Here we choose the rightmost centroid (largest x-coordinate).
        # In cases where multiple line segments appear (like intersections or curves),
        # choosing the rightmost one helps the robot stay aligned with the primary path.
        # alternatives (largest contour, leftmost, etc.)
        most_right_centroid = max(centres, key=lambda x: x[0])

        try:
            # Extract the x and y of the selected centroid.
            cx = most_right_centroid[0]
            cy = most_right_centroid[1]
        except:
            # If no centroid is detected (centres is empty), default to image center.
            # This prevents crashes and keeps robot stable.
            cy, cx = height / 2, width / 2

        # Draw a small red circle on the selected (active) centroid.
        # - Green = all centroids detected
        # - Red = the one the robot actually uses for steering
        cv2.circle(res, (int(cx), int(cy)), 5, (0, 0, 255), -1)

        # ----------------------------------------------------------------------
        #                 DISPLAY THE VISUALIZATION WINDOW
        # ----------------------------------------------------------------------
        # The window "RES" shows:
        #   - Cropped bottom image
        #   - Detected purple regions (in color)
        #   - Green circles for all centroids
        #   - Red circle for the chosen one
        cv2.imshow("RES", res)
        cv2.waitKey(1)  # small delay to refresh display each frame

        # ----------------------------------------------------------------------
        #                   CALCULATE THE STEERING ERROR
        # ----------------------------------------------------------------------
        # The robot tries to keep the red centroid at the center of the image.
        # Compute how far the selected centroid is from the image center (width/2).
        # Positive error → centroid to the right → turn right.
        # Negative error → centroid to the left → turn left.
        error_x = cx - width / 2

        # Use the computed horizontal error to control robot velocity.
        self.pub_velocities(error_x)


    def pub_velocities(self, error: float) -> None:
        """
        Publishes linear and angular velocities to keep the robot aligned with the line.

        The controller here is a simple proportional controller (P-controller):
            angular velocity = -error / 100

        The scaling factor (1/100) determines how sharply the robot turns.
        """
        twist_object = Twist()

        twist_object.linear.x = 0.2

        # Set angular velocity proportional to error.
        # The negative sign ensures the robot turns in the correct direction.
        # (depends on how your camera and robot axes are oriented).
        twist_object.angular.z = -error / 100

        self.get_logger().info(f"Adjusting the angular velocity ---> {twist_object.angular.z:.2f}")

        self.pub_vel.publish(twist_object)


def main(args=None) -> None:

    # Initialize
    rclpy.init(args=args)
    line_follower = LineFollower()

    try:
        # Keep the node running
        rclpy.spin(line_follower)
    except KeyboardInterrupt:
        # Graceful shutdown when the user presses Ctrl+C.
        pass
    line_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
