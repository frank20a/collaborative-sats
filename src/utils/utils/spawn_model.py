import rclpy
from gazebo_msgs.srv import SpawnEntity
from rclpy.node import Node
import xacro, sys, os

class ModelSpawner(Node):

    def __init__(self):
        super().__init__('spawn_model')

        self.declare_parameter('name', '')
        self.declare_parameter('suffix', '')
        self.declare_parameter('folder', '')

        self.cli = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()

    def send_request(self):

        # Get the robot description XML
        name = self.get_parameter('name').get_parameter_value().string_value
        folder = self.get_parameter('folder').get_parameter_value().string_value
        if folder == '':
            path = os.path.join('/home/frank20a/dev-ws/models', name, name + '.urdf.xacro')
        else:
            path = os.path.join('/home/frank20a/dev-ws/models', folder, name + '.urdf.xacro')
        self.req.xml = xacro.process_file(path).toxml()

        # Set the robot name in Gazebo
        suffix = self.get_parameter('suffix').get_parameter_value().string_value
        if suffix != '': 
            self.req.name = name + '_' + suffix
        else:
            self.req.name = name

        if suffix != '':
            self.req.robot_namespace = self.req.name


        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    spawner = ModelSpawner()
    spawner.send_request()

    while rclpy.ok():
        rclpy.spin_once(spawner)
        if spawner.future.done():
            try:
                response = spawner.future.result()
            except Exception as e:
                spawner.get_logger().info('Service call failed %r' % (e,))
            else:
                spawner.get_logger().info('Spawned succesfully')
            break

    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()