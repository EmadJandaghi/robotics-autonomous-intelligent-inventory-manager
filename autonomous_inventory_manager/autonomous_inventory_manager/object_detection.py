from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.msg import InferenceResult, Yolov8Inference, ObjectCount
from std_srvs.srv import Trigger

bridge = CvBridge()

class YoloObjectDetection(Node):
    def __init__(self) -> None:
        super().__init__('object_detection')

        # Load a pre-trained YOLOv8 object detection model
        self.model = YOLO('~/ros2_ws/src/autonomous_inventory_manager/data/yolov8n.pt')
        self.yolov8_inference = Yolov8Inference()

        self.img_sub = self.create_subscription(
            Image,
            '/deepmind_robot1/deepmind_robot1_camera/image_raw',
            self.camera_callback,
            10)

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        self.object_count_pub = self.create_publisher(ObjectCount, "/detected_object_count", 1)

        # handle start/stop commands using a service
        self.start_service = self.create_service(Trigger, 'start_detection', self.start_detection)
        self.stop_service = self.create_service(Trigger, 'stop_detection', self.stop_detection)
        self.active = False


    def start_detection(self, request, response):
        self.active = True
        response.success = True
        response.message = "Object detection started"
        return response

    def stop_detection(self, request, response):
        self.active = False
        response.success = True
        response.message = "Object detection stopped"
        return response

    def camera_callback(self, msg: Image) -> None:
        """ Performs object detection using the loaded YOLO model
            and processes the detection results. The image with 
            annotated detections is also published for visualization """
        if not self.active:
            return

        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img) 

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        object_counts = {}
        # This is used because the Yolo object detection recognizes unwanted objects
        excluded_objects = ['dining table', 'keyboard', 'umbrella', 'remote', 'chair']

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inf_result = InferenceResult()
                # get box coordinates in (top, left, bottom, right) format
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  
                c = box.cls
                class_name = self.model.names[int(c)]
                self.inf_result.class_name = class_name
                self.inf_result.left = int(b[0])
                self.inf_result.top = int(b[1])
                self.inf_result.right = int(b[2])
                self.inf_result.bottom = int(b[3])
                self.inf_result.box_width = (self.inf_result.right - self.inf_result.left) 
                self.inf_result.box_height = (self.inf_result.bottom - self.inf_result.top)
                self.inf_result.x = self.inf_result.left + (self.inf_result.box_width/2.0)
                self.inf_result.y = self.inf_result.top + (self.inf_result.box_height/2.0)
                self.yolov8_inference.yolov8_inference.append(self.inf_result)

                # Exclude specific unwanted objects and filter out cell phones with a height less than 100 pixels, as these are often misinterpreted
                if class_name not in excluded_objects and not (class_name == "cell phone" and self.inf_result.box_height < 100):
                    if class_name in object_counts:
                        object_counts[class_name] += 1 
                    else:
                        object_counts[class_name] = 1

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  
        self.img_pub.publish(img_msg)

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

        # Print the object counts
        print("Detected objects:")
        print(f"{object_counts}")
        for class_name, count in object_counts.items():
            print(f"{class_name}: {count}")

        # Publish the object counts
        object_count_msg = ObjectCount()
        object_count_msg.header.stamp = self.get_clock().now().to_msg()
        object_count_msg.header.frame_id = "object_count"
        for class_name, count in object_counts.items():
            object_count_msg.classes.append(class_name)
            object_count_msg.counts.append(count)
        
        self.object_count_pub.publish(object_count_msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    object_detection = YoloObjectDetection()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()