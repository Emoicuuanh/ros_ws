import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class simpleparameter(Node):
    def __init__(self):
        super().__init__("simpleparameter")
        self.declare_parameter("simpleint" ,28)
        self.declare_parameter("simpestring","dung")

        self.add_on_set_parameters_callback(self.paracallback)

    def paracallback(self,params):
        result = SetParametersResult()

        for param in params:
            if(param.name == "simpleint" and param.type == Parameter.Type.INTEGER):
                self.get_logger().info("New value is %d" % param.value)
                result.successful =True
            if(param.name == "simpestring" and param.type == Parameter.Type.STRING):
                self.get_logger().info("New value is %s" % param.value)
                result.successful =True
        return result
def main():
    rclpy.init()
    simple_parameter =simpleparameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()
    







