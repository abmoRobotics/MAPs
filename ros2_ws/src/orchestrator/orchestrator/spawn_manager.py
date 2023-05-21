import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from tutorial_interfaces.srv import PoseSrv
from utils import ValueIDManager, ClientAsync
from geometry_msgs.msg import Pose
class SpawnManager(Node):
    def __init__(self):
        super().__init__('spawn_manager')
        # Define publishers and         
        print("Spawn manager initialized")
        # Define callback timer
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.spawn_manager_callback)


        self.number_of_segments = 0
        self.segment_positions = []


        self.client = self.create_client(GetParameters,
                                '/global_parameter_server/get_parameters')
        

        self.request = GetParameters.Request()
        self.request.names = ['num_of_tables', 'table_position'] 
        
        self.states = {key: 0 for key in self.request.names}

        self.service_client = ClientAsync()
        self.value_id_manager = ValueIDManager()
        self.old_number_of_segments = 0


    def spawn_manager_callback(self):

        future = self.client.call_async(request=self.request)
        future.add_done_callback(self.callback_global_param)
        removed_array_index, removed_ids = self.value_id_manager.sync_array(self.segment_positions)



        # if new_states['num_of_shuttles'] != self.states['num_of_shuttles']:
        #     if new_states['num_of_shuttles'] > self.states['num_of_shuttles']:
        #         for i in range(new_states['num_of_shuttles'] - self.states['num_of_shuttles']):
        #             self.spawn_shuttle()
        #     else:
        #         for i in range(self.states['num_of_shuttles'] - new_states['num_of_shuttles']):
        #             self.remove_shuttle()
        
        # if new_states['num_of_manipulators'] != self.states['num_of_manipulators']:
        #     if new_states['num_of_manipulators'] > self.states['num_of_manipulators']:
        #         for i in range(new_states['num_of_manipulators'] - self.states['num_of_manipulators']):
        #             self.spawn_manipulator()
        
        if self.number_of_segments < self.old_number_of_segments:
            removed_array_index.sort(reverse=True)
            for array_idx, idx in zip(removed_array_index, removed_ids):
                self.service_client.remove_segment(idx)

        elif self.number_of_segments > self.old_number_of_segments:
            if len(list(self.value_id_manager.value_to_id.keys())) == 0:
                segment_id = 0
            else:
                segment_id = list(self.value_id_manager.id_to_value.keys())[-1]
            
            segment_position = self.value_id_manager.id_to_value[segment_id]
            segment_pose = self.transform_to_pose(segment_position)
            self.service_client.spawn_segment(segment_id, segment_pose)

        self.old_number_of_segments = self.number_of_segments



        # self.spawn_shuttle()
        # self.spawn_manipulator()
        # self.spawn_segments()
        # try:
        #     #print(f'Number of shuttles: {self.result.values[0].integer_value}')
        #     print(f'number of shuttles {self.result.values[0].integer_value}')
        #     print(f'number of manipulators {self.result.values[1].integer_value}')
        #     print(f'number of segments {self.result.values[2].integer_value}')
        #     print(f'positions of segments {self.result.values[3].double_array_value}')
        #     print(f'positions of manipulators {self.result.values[4].double_array_value}')
        # except Exception as e:
        #     print(e)

    def spawn_shuttle(self):
        pass

    def spawn_manipulator(self, robot_type, robot_id, pose):
        pass

    def spawn_segments(self):
        pass

    def remove_shuttle(self):
        pass

    def remove_manipulator(self, robot_type, robot_id):
        pass

    def remove_segments(self):
        pass

    def callback_global_param(self, future):
        try:
            result = future.result()
            self.number_of_segments = result.values[0].integer_value
            self.segment_positions = result.values[1].double_array_value
            self.number_of_segments = len(self.segment_positions)

        except Exception as e:
            self.get_logger().warn("Service call failed inside shuttle %r" % (e,))



    def transform_to_pose(self, pos):
        pos = str(pos)
        x, y = pos.split('.')
        pose = Pose()
        pose.position.x = -float(y)*0.24-0.2
        pose.position.y = float(x)*0.24+0.2
        pose.position.z = float(0.9)
        pose.orientation.x = float(0)
        pose.orientation.y = float(0)
        pose.orientation.z = float(0)
        pose.orientation.w = float(1)
        return pose

    # def remove_extras(self, arr):
    #     for value in list(self.value_to_id.keys()): # create a copy of keys to avoid runtime error
    #         if value not in arr:
    #             self.remove_by_value(value)
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


