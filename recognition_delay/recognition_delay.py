#!/usr/bin/env python

import time
import threading
import math
import numpy as np
import copy
import rclpy
from rclpy.node import Node
from derived_object_msgs.msg import ObjectArray, Object
from shape_msgs.msg import SolidPrimitive
from autoware_auto_perception_msgs.msg import DetectedObjects, DetectedObject, ObjectClassification, DetectedObjectKinematics, Shape



class DelayGenerator(Node):
    def __init__(self, role_name:str, delay_time:float, rand_pos_mean:float, rand_pos_std:float):
        super().__init__('recognition_delay')
        # self.declare_parameter('t_delay', 1)
        # self.declare_parameter('role_name')
        self.carla_objects_sub = self.create_subscription(
            ObjectArray, f"/carla/{role_name}/objects", self._objects_updated, 10)
        self.dummy_objects_sub = self.create_subscription(
            DetectedObjects, "/perception/object_recognition/detection/objects/ontime", self._autoware_objects_updated, 10)

        self.autoware_objects_pub = self.create_publisher(
            DetectedObjects, "/perception/object_recognition/detection/objects", 
            rclpy.qos.QoSProfile(depth=1)
            )
        # self.autoware_objects_pub = self.create_publisher(
        #     DetectedObjects, "/lidar_center_point/output/objects", 
        #     rclpy.qos.QoSProfile(depth=1)
        #     )
        self._role_name = role_name
        self.delay_time = delay_time    # [seconds]
        self.rand_pos_mean = rand_pos_mean
        self.rand_pos_std = rand_pos_std
        self._delay_objects = DetectedObjects()


    def _autoware_objects_updated(self, object_array):
        object_array.objects.extend(self._delay_objects.objects)
        if object_array.header.frame_id != "map":
            object_array.header.frame_id = "map"
        self.autoware_objects_pub.publish(object_array)
    
    def _objects_updated(self, object_array):
        t = threading.Thread(target=self.publish_delay_objects, args=(object_array,))
        t.start()
    
    def publish_delay_objects(self, object_array):
        time.sleep(self.delay_time)
        converted_msg = self._convert_object_array_to_detected_objects(object_array)
        self._delay_objects = converted_msg
    
    def _decide_autoware_existence_probability_from_object(self, object: Object) -> float:
        # range (min=0.0, max=1.0)
        autoware_existence_probability = 0.0
        
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
            
            autoware_classification.probability = 1.0

        return autoware_classification

    def _decide_autoware_detected_object_kinematics_from_object(self, object: Object) -> DetectedObjectKinematics:
        autoware_kinematics = DetectedObjectKinematics()
        
        # covarianceはもともとのobjectに含まれていないのでFalse
        autoware_kinematics.has_position_covariance = False
        autoware_kinematics.has_twist_covariance = False
        
        # twist情報の有無
        autoware_kinematics.has_twist = False

        autoware_kinematics.orientation_availability = DetectedObjectKinematics.UNAVAILABLE
        
        # pose (covarianceないので省略)、正規分布に基づいてrandom性を加える
        current_position = [object.pose.position.x, object.pose.position.y, object.pose.position.z]
        rand_position = self.add_normal_randomness_to_position(current_position, mean=self.rand_pos_mean, std=self.rand_pos_std)
        object.pose.position.x = rand_position[0]
        object.pose.position.y = rand_position[1]
        object.pose.position.z = rand_position[2]
        autoware_kinematics.pose_with_covariance.pose = object.pose
        # twist (covarianceないので省略)
        
        radian = math.atan2(object.twist.linear.y, object.twist.linear.x)
        
        magnitude = math.sqrt(object.twist.linear.y**2 + object.twist.linear.x**2)
        # ここで角度合わせ
        convert_radian = radian/360
        convert_x = magnitude * math.cos(convert_radian)
        convert_y = magnitude * math.sin(convert_radian)
        convert_twist = copy.copy(object.twist)
        convert_twist.linear.x = convert_x
        convert_twist.linear.y = convert_y

        # convert_twist.linear.x = object.twist.linear.x
        # convert_twist.linear.y = object.twist.linear.y

        autoware_kinematics.twist_with_covariance.twist = convert_twist

        ##dummy carと合わせてみる
        # autoware_kinematics.twist_with_covariance.twist.linear.x = 3.0
        # autoware_kinematics.twist_with_covariance.twist.linear.y = 0.0
        # autoware_kinematics.twist_with_covariance.twist.linear.z = 0.0

        # autoware_kinematics.twist_with_covariance.twist.angular.x = 0.0
        # autoware_kinematics.twist_with_covariance.twist.angular.y = 0.0
        # autoware_kinematics.twist_with_covariance.twist.angular.z = 0.0

        
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
        
        detected_objects.header.stamp = self.get_clock().now().to_msg()

        return detected_objects

    def add_normal_randomness_to_position(self, position:list, mean:float=0, std:float=1) -> list:
        # 正規分布に従う乱数を生成し、元の位置情報に加える
        random_offsets = np.random.normal(mean, std, size=len(position))
        new_position = [p + offset for p, offset in zip(position, random_offsets)]
        return new_position
    
    def set_delay_time(self, delay_time):
        self.delay_time = delay_time


def main(args=None):
    rclpy.init(args=args)

    role_name = 'ego_vehicle'
    delay_time = 0.0
    rand_pos_mean = 0.0
    rand_pos_std = 0.0
    delay_generator = DelayGenerator(role_name, delay_time, rand_pos_mean, rand_pos_std)

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
