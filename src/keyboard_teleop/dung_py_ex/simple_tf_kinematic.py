import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster,TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from my_robot_msgs.srv import GetTranform
from tf_transformations import quaternion_from_euler, quaternion_multiply ,quaternion_inverse


class SimpleTfkinematic(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematic")

        self.static_tf_kinematic_ = StaticTransformBroadcaster(self)
        self.static_stamped_ = TransformStamped()

        self.dynamic_tf_kinematic_ =TransformBroadcaster(self)
        self.dynamic_tf_stamped_ = TransformStamped()
        self.x_increment_ = 0.05
        self.last_x_ = 0.0

        self.rotation_counter_ = 0 
        self.last_orientation_ = quaternion_from_euler(0 , 0 , 0)
        self.orientation_increment_ = quaternion_from_euler(0 , 0 , 0.05)


        self.tf_buffer_ =Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_,self)

        self.static_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_stamped_.header.frame_id = "base_footprint"
        self.static_stamped_.child_frame_id = "base_child"

        self.static_stamped_.transform.translation.x = 0.0
        self.static_stamped_.transform.translation.y =0.0
        self.static_stamped_.transform.translation.z = 0.2
        self.static_stamped_.transform.rotation.x = 0.0
        self.static_stamped_.transform.rotation.y = 0.0
        self.static_stamped_.transform.rotation.z = 0.0
        self.static_stamped_.transform.rotation.w = 1.0
        self.static_tf_kinematic_.sendTransform(self.static_stamped_)

        self.timer_ = self.create_timer(0.1, self.timerCallback)

        self.get_tranform_srv_=self.create_service(GetTranform ,"get_tranform" , self.SrvCallback)


    def timerCallback(self):
        self.dynamic_tf_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_tf_stamped_.header.frame_id = "odom"
        self.dynamic_tf_stamped_.child_frame_id = "base_footfrint"

        self.dynamic_tf_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_tf_stamped_.transform.translation.y =0.0
        self.dynamic_tf_stamped_.transform.translation.z = 0.2
        q = quaternion_multiply(self.last_orientation_ , self.orientation_increment_)
        self.dynamic_tf_stamped_.transform.rotation.x = q[0]
        self.dynamic_tf_stamped_.transform.rotation.y = q[1]
        self.dynamic_tf_stamped_.transform.rotation.z = q[2]
        self.dynamic_tf_stamped_.transform.rotation.w = q[3]
        self.dynamic_tf_kinematic_.sendTransform(self.dynamic_tf_stamped_)

        self.rotation_counter_ +=1
        self.last_orientation_ = q
        self.last_x_ = self.dynamic_tf_stamped_.transform.translation.x 
        if self.rotation_counter_ >=50:
            self.orientation_increment_ = quaternion_inverse(self.orientation_increment_)
            self.rotation_counter_=0
            self.x_increment_ = -self.x_increment_

    def SrvCallback(self, req, res):
        request_tranform_ = TransformStamped()
        try:
            request_tranform_= self.tf_buffer_.lookup_transform(req.frame_id,req.child_frame_id , rclpy.time.Time())
        except TransformException as e:
            res.success = False
            return res
    
        res.transform = request_tranform_
        res.success= True
        return res




def main():
    rclpy.init()
    simple_tf_kinematic = SimpleTfkinematic()
    rclpy.spin(simple_tf_kinematic)
    simple_tf_kinematic.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()























