import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


class SpawnManager(Node):
    def __init__(self):
        super().__init__('spawn_manager')
        # Define publishers and         
        print("Spawn manager initialized")
        # Define callback timer
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.spawn_manager_callback)

        self.number_of_shuttles = 0
        self.number_of_manipulators = 0
        self.number_of_segments = 0
        self.segment_positions = []
        self.manipulator_positions = []


        self.client = self.create_client(GetParameters,
                                '/global_parameter_server/get_parameters')
        

        # global_parameters = ['num_of_shuttles', 'num_of_manipulators', 'num_of_tables', 'table_positions', 'robot_positions']
        # self.requests = [GetParameters.Request() for i in range(len(global_parameters))]
        # for idx, request in enumerate(self.requests):
        #     request.names = [global_parameters[idx]]
            
        # self.result = {key: 0 for key in global_parameters}

        self.request = GetParameters.Request()
        self.request.names = ['num_of_shuttles', 'num_of_manipulators', 'num_of_tables', 'table_positions', 'robot_positions'] #['num_of_shuttles']
        # #self.result = 0

        
    def spawn_manager_callback(self):
        # for idx, request in enumerate(self.requests):
        #     future = self.client.call_async(request=request)
        #     future.add_done_callback(self.callback_global_param)

        future = self.client.call_async(request=self.request)
        future.add_done_callback(self.callback_global_param)
        self.spawn_shuttle()
        self.spawn_manipulator()
        self.spawn_segments()
        try:
            #print(f'Number of shuttles: {self.result.values[0].integer_value}')
            print(f'number of shuttles {self.result.values[0].integer_value}')
            print(f'number of manipulators {self.result.values[1].integer_value}')
        except:
            pass

    def spawn_shuttle(self):
        pass

    def spawn_manipulator(self):
        pass

    def spawn_segments(self):
        pass

    def callback_global_param(self, future):
        try:
            #print(future)
            self.result = future.result()

        except Exception as e:
            self.get_logger().warn("Service call failed inside shuttle %r" % (e,))


def main(args=None):
    """Initializes the Shuttle node and spins it until it is destroyed."""
    rclpy.init(args=args)

    node = SpawnManager()

    rclpy.spin(node)
    #print("NU")
    #save_yaml(shuttle_node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
