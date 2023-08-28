#!/usr/bin/env python

import time
import threading
import rclpy
from rclpy.node import Node
from derived_object_msgs.msg import ObjectArray, Object
from shape_msgs.msg import SolidPrimitive
from autoware_auto_perception_msgs.msg import DetectedObjects, DetectedObject, ObjectClassification, DetectedObjectKinematics, Shape



class DelayGenerator(Node):
    def __init__(self, role_name, delay_time):
        super().__init__('recognition_delay')
        self.carla_objects_sub = self.create_subscription(
            ObjectArray, f"/carla/{role_name}/objects", self._objects_updated, 10)
        self.autoware_objects_pub = self.create_publisher(
            DetectedObjects, "/lidar_center_point/output/objects", 
            rclpy.qos.QoSProfile(depth=1)
            )
        self._role_name = role_name
        self.delay_time = delay_time    # [seconds]
    
    def _objects_updated(self, object_array):
        t = threading.Thread(target=self.publish_delay_objects, args=(object_array,))
        t.start()
    
    def publish_delay_objects(self, object_array):
        time.sleep(self.delay_time)
        
        output_msg = self._convert_object_array_to_detected_objects(object_array)

        # for object in object_array.objects:
        #     self.get_logger().info(str(object.LASSIFICATION_UNKNOWN))  # ログ出力に変更

        self.autoware_objects_pub.publish(output_msg)
    
    def _decide_autoware_existence_probability_from_object(self, object: Object) -> float:
        # object側のdetection_levelが検知(DETECTED)なら確率を低めに、追跡(TRACKED)なら確率を高めにする
        # range (min=0.0, max=1.0)
        autoware_existence_probability = float()
        if object.detection_level == Object.OBJECT_DETECTED:
            autoware_existence_probability = 0.5
        elif object.detection_level == Object.OBJECT_TRACKED:
            autoware_existence_probability = 1.0
        
        return autoware_existence_probability
      
    def _decide_autoware_object_classification_from_object(self, object: Object) -> ObjectClassification:
        autoware_classification = ObjectClassification()
        if object.object_classified:
            if object.classification in [
                Object.CLASSIFICATION_UNKNOWN,
                Object.CLASSIFICATION_UNKNOWN_SMALL,
                Object.CLASSIFICATION_UNKNOWN_MEDIUM,
                Object.CLASSIFICATION_UNKNOWN_BIG
                ]:
                autoware_classification.label ==  ObjectClassification.UNKNOWN
            elif object.classification == Object.CLASSIFICATION_PEDESTRIAN:
                autoware_classification.label = ObjectClassification.PEDESTRIAN
            elif object.classification == Object.CLASSIFICATION_BIKE:
                autoware_classification.label = ObjectClassification.BICYCLE
            elif object.classification == Object.CLASSIFICATION_CAR:
                autoware_classification.label = ObjectClassification.CAR
            elif object.classification == Object.CLASSIFICATION_TRUCK:
                autoware_classification.label = ObjectClassification.TRUCK
            elif object.classification == Object.CLASSIFICATION_MOTORCYCLE:
                autoware_classification.label = ObjectClassification.MOTORCYCLE
            elif object.classification == Object.CLASSIFICATION_OTHER_VEHICLE:
                autoware_classification.label = ObjectClassification.BUS
            # BARRIERとSIGNに対応するautoware側のmsgはないのでUNKNOWを割り当てる
            # TODO : SIGNについてはautowareの別のtopicにpublishするべきかもしれない
            elif object.classification in [
                Object.CLASSIFICATION_BARRIER,
                Object.CLASSIFICATION_SIGN
                ]:
                autoware_classification.label = ObjectClassification.UNKOWN
            
            autoware_classification.probability = object.classification_certainty / 255

        return autoware_classification

    def _decide_autoware_detected_object_kinematics_from_object(self, object: Object) -> DetectedObjectKinematics:
        autoware_kinematics = DetectedObjectKinematics()
        
        return autoware_kinematics
    
    def _decide_autoware_shape_from_object(self, object: Object) -> Shape:
        autoware_shape = Shape()
        
        # type(uint8)
        if object.shape.type == SolidPrimitive.BOX:
            autoware_shape.type = Shape.BOUNDING_BOX
        elif object.shape.type == SolidPrimitive.CYLINDER:
            autoware_shape.type = Shape.CYLINDER
        # SPHERE, CONE に対応するautoware側のmsgはないのでPOLYGON(多角形)を割り当てる
        elif object.shape.type in [
            SolidPrimitive.SPHERE,
            SolidPrimitive.CONE
            ] :
            autoware_shape.type = Shape.POLYGON
        
        # footprint(geometry_msgs::msg::Polygon)
        autoware_shape.footprint = object.polygon
        
        # dimensions (geometry_msgs::msg::Vector3)
        autoware_shape.dimensions.x = object.shape.dimensions[0]
        autoware_shape.dimensions.y = object.shape.dimensions[1]
        autoware_shape.dimensions.z = object.shape.dimensions[2]
        
        return autoware_shape
    
    def _convert_object_array_to_detected_objects(self, object_array: ObjectArray) -> DetectedObjects:
        detected_objects = DetectedObjects()
        detected_objects.header = object_array.header
        
        for object in object_array.objects:
            detected_object = DetectedObject()
            
            existence_probability = self._decide_autoware_existence_probability_from_object(object)
            detected_object.existence_probability = existence_probability

            classification = self._decide_autoware_object_classification_from_object(object)
            detected_object.classification.append(classification)

            kinematics = self._decide_autoware_detected_object_kinematics_from_object(object)
            detected_object.kinematics = kinematics

            shape = self._decide_autoware_shape_from_object(object)
            detected_object.shape = shape
            
            detected_objects.objects.append(detected_object)
        
        return detected_objects
    
    def set_delay_time(self, delay_time):
        self.delay_time = delay_time


def main(args=None):
    rclpy.init(args=args)

    role_name = 'ego_vehicle'
    delay_time = 0.1
    delay_generator = DelayGenerator(role_name, delay_time)

    try:
        rclpy.spin(delay_generator)
    except KeyboardInterrupt:
        pass
    
    delay_generator.carla_objects_sub.destroy()
    delay_generator.autoware_objects_pub.destroy()
    delay_generator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
