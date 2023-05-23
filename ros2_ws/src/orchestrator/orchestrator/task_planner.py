import rclpy
from rclpy.node import Node
import numpy as np
from math import pi

from robot_actions.action import Man
from robot_actions.action import Shu

from utils import create_action_client_array
from utils import KR4R600, KR3R540, ValueIDManager, TaskPlannerUtils, Station, Shuttle
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
class TaskPlanner(Node):

    def __init__(self):
        super().__init__('task_planner')
        # Define publishers and messages
        
        number_of_kuka540 = 2
        number_of_kuka600 = 2

        self.number_of_shuttles = 5 * ["Shuttle"]
        self.number_of_shuttles = 0
        self.number_of_manipulators = 0
        self.number_of_segments = 0
        self.robot_position = []
        self.segment_position = []
        self.old_number_of_segments = 0
        self.old_number_of_manipulators = 0
        self.old_number_of_shuttles = 0

        name540 = number_of_kuka540 * [KR3R540()._name]
        name600 = number_of_kuka600 * [KR4R600()._name]

        self.name_of_manipulators = np.concatenate((name600, name540))
        self.shuttle_joint_state_subscribers = []
        

        goal = [0, -pi/2, pi/2, 0, 0, 0]

        # for idx, name in enumerate(self.name_of_manipulators):
        #     self.ManiActionClient = create_action_client_array(self, 
        #                                                        Man, 
        #                                                        topic_prefix=name,
        #                                                        n_robots=1) 
            
        # for idx, name in enumerate(self.number_of_shuttles):
        #     self.ShuActionClient = create_action_client_array(self,
        #                                                       Shu,
        #                                                       topic_prefix=name,
        #                                                       n_robots=1
        #                                                       )

        self.action_clients_manipulator = []
        self.action_clients_shuttle = []
        self.client = self.create_client(GetParameters,
                                '/global_parameter_server/get_parameters')
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.main_callback)
        print("Task planner initialized")
        self.value_id_manager_shuttle = ValueIDManager()
        self.value_id_manager_manipulator = ValueIDManager()
        self.value_id_manager_segment = ValueIDManager()
        print("Task planner initialized")
        self.task_planner = TaskPlannerUtils()


        self.shuttles_id_to_storage_station = []
        self.shuttles_waiting_for_storage = []

    def main_callback(self):
        self.update_parameters_from_server()
        #self.print_parameters()
        #self.get_logger().info(f'task_planner {self.number_of_manipulators}')
        self.update_states()

    def update_parameters_from_server(self):
        request = GetParameters.Request()
        request.names = ['num_of_manipulators', 'robot_position', 'num_of_tables', 'table_position', 'num_of_shuttles']
        future = self.client.call_async(request=request)
        future.add_done_callback(self.update_parameters_from_server_callback)


        #1. Run task planner
        self.task_planner.main()

        #2. Get active tasks
        active_tasks = self.task_planner.get_active_tasks()
        if active_tasks:
            for idx, active_task in enumerate(active_tasks):
                self.get_logger().info(f'active task idx{idx}:  {active_task.get_assigned_shuttle_id()}')
        #3. Move the shuttles to the assigned stations and wait for 3 seconds per add command 
        if active_tasks:
            for idx, active_task in enumerate(active_tasks):
                if not active_task.get_in_progress():
                    shuttle_id = active_task.get_assigned_shuttle_id()
                    station = active_task.get_assigned_station()
                    station_position = station.get_position()

                    # call action client
                    goal = [station_position[0], station_position[1], 0.0, 0.0, 0.0, 0.0]
                    self.send_goal_for_shuttles(goal, shuttle_id)


                    active_task.set_in_progress(True)
                    self.get_logger().info(f'active task idx {idx}:  {active_task.get_assigned_shuttle_id()}')




        #4. Move the shuttles to a random segment that is not a station and set shuttle to free
        
        if self.shuttles_id_to_storage_station:
            for shuttle_id in self.shuttles_id_to_storage_station:
                goal = [0.1, 0.1, 0.0, 0.0, 0.0, 0.0]
                self.send_goal_for_shuttles(goal, shuttle_id)
                self.shuttles_id_to_storage_station.pop(0)
                self.shuttles_waiting_for_storage.append(shuttle_id)

    
    def update_states(self):
        # self.get_logger().info(f'old number of manipulators {self.old_number_of_manipulators}')
        # self.get_logger().info(f'number of manipulators {self.number_of_manipulators}')
        #removed_array_index_shuttle, removed_ids_shuttle = self.value_id_manager_shuttle.sync_array(self.number_of_shuttles)
        removed_array_index_manipulator, removed_ids_manipulator = self.value_id_manager_manipulator.sync_array(self.robot_position)
        removed_array_index_segment, removed_ids_segment = self.value_id_manager_segment.sync_array(self.segment_position)
        #self.get_logger().info(f'segment position {self.segment_position}')

        if self.number_of_manipulators < self.old_number_of_manipulators:
            removed_array_index_manipulator.sort(reverse=True)
            for array_idx, idx in zip(removed_array_index_manipulator, removed_ids_manipulator):
                self.action_clients_manipulator[array_idx].destroy()
                self.action_clients_manipulator.pop(array_idx)
                

                ## Remove station from task planner class
                #station = self.find_closet_segment_to_manipulator(self.robot_position[array_idx])
                station_to_remove = self.task_planner.stations.get_station_by_index(array_idx)
                self.task_planner.stations.remove_station(station_to_remove)
            
            #self.get_logger().info(f'stations {self.task_planner.stations.get_stations()}')
            self.old_number_of_manipulators = self.number_of_manipulators

        if self.number_of_manipulators > self.old_number_of_manipulators:
            if len(list(self.value_id_manager_manipulator.id_to_value.keys())) == 0:
                manipulator_id = 0
            else:
                manipulator_id = list(self.value_id_manager_manipulator.id_to_value.keys())[-1]
            
            self.action_clients_manipulator.append(ActionClient(self,Man, KR4R600()._name + str(f'{manipulator_id:02}')))

            ## Add station to task planner class
            station_position = self.find_closet_segment_to_manipulator(self.robot_position[manipulator_id])
            station = Station()
            station.set_position(self.convert_to_xy(station_position)*0.24)
            # print(station)
            # print(self.task_planner.stations)
            

            self.task_planner.stations.add_station(station)
            # self.get_logger().info(f'stations {station}')
            # self.get_logger().info(f'stations {self.task_planner.stations}')
            # self.get_logger().info(f'stations {self.task_planner.stations.get_stations()}')

            self.old_number_of_manipulators = self.number_of_manipulators

        if self.number_of_shuttles < self.old_number_of_shuttles:
            shuttles = self.task_planner.shuttles.get_shuttles()
            shuttles_to_remove = shuttles[self.number_of_shuttles:]
            for shuttle in shuttles_to_remove:
                self.task_planner.shuttles.remove_shuttle(shuttle)


            self.shuttle_joint_state_subscribers[self.number_of_shuttles-1].destroy()
            self.shuttle_joint_state_subscribers.pop(self.number_of_shuttles-1)
            self.action_clients_shuttle[self.number_of_shuttles-1].destroy()
            self.action_clients_shuttle.pop(self.number_of_shuttles-1)


            self.old_number_of_shuttles = self.number_of_shuttles

        elif self.number_of_shuttles > self.old_number_of_shuttles:
            shuttle = Shuttle()
            diff = self.number_of_shuttles - self.old_number_of_shuttles
            for i in range(diff):
                self.shuttle_joint_state_subscribers.append(self.create_subscription(JointState, f'/shuttle{(self.number_of_shuttles-1+i):02}/joint_states', self.shuttle_joint_state_callback, 10))
                self.task_planner.shuttles.add_shuttle(shuttle)
                
                self.action_clients_shuttle.append(ActionClient(self,Shu, f'/shuttle/shuttle{(self.number_of_shuttles-1+i):02}'))
            # self.shuttle_joint_state_subscribers.append(self.create_subscription(JointState, f'/shuttle{(self.number_of_shuttles-1):02}/joint_states', self.shuttle_joint_state_callback, 10))
            # self.task_planner.shuttles.add_shuttle(shuttle)
            # self.get_logger().info(f'shuttles {self.task_planner.shuttles.get_shuttles()}')
            self.old_number_of_shuttles = self.number_of_shuttles

    def update_parameters_from_server_callback(self, future):
        try:
            result = future.result()
            #self.number_of_manipulators = result.values[0].integer_value
            self.robot_position = result.values[1].double_array_value
           # self.number_of_segments = result.values[2].integer_value
            self.segment_position = result.values[3].double_array_value
            self.number_of_shuttles = result.values[4].integer_value
            self.number_of_manipulators = len(self.robot_position)
            self.number_of_segments = len(self.segment_position)
        except Exception as e:
            self.get_logger().warn("Service call failed inside shuttle %r" % (e,))

    def print_parameters(self):
        print(f'number of shuttles {self.number_of_shuttles}')
        print(f'number of manipulators {self.number_of_manipulators}')
        print(f'number of segments {self.number_of_segments}')
        print(f'positions of segments {self.segment_position}')
        print(f'positions of manipulators {self.robot_position}')

    def manipulator_send_goal(self, goal):
        # Function that send the goal for multiple manipulators
        goal_msg = Man.Goal()
        goal_msg.goal = goal

        for idx, Client in enumerate(self.ManiActionClient):
            self.ManiActionClient[idx].wait_for_server()
            future = self.ManiActionClient[idx].send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            future.add_done_callback(self.goal_response_callback) 

    def shuttle_send_goal(self, goal):
        # Function that send the goal for multiple shuttles
        goal_msg = Shu.Goal()
        goal_msg.goal = goal

        for idx, Client in enumerate(self.ShuActionClient):
            self.ShuActionClient[idx].wait_for_server()
            future = self.ShuActionClient[idx].send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            future.add_done_callback(self.goal_response_callback) 
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.goalresult))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.goalfeedback))

    def shuttle_joint_state_callback(self, msg: JointState):
        try:
            #self.get_logger().info(f'shuttle joint state {msg}')
            #self.get_logger().info(f'shuttle joint state {msg.name}')
            idx = int(msg.name[0])
        # self.get_logger().info(f'shuttle joint state idx {idx}')
            x, y = msg.position[0], msg.position[1]
            position = np.array([x, y])
            #self.get_logger().info(f'all shuttles {self.task_planner.shuttles.get_shuttles()}')
            self.task_planner.shuttles.get_shuttle_by_index(idx).set_position(position)
        # self.get_logger().info(f'shuttle joint state {msg}')
        except:
            pass

    def find_closet_segment_to_manipulator(self, position):
        segment_positions = self.segment_position
        min_distance = np.inf
        position = self.convert_to_xy(position)
        closest_segment = None
        for segment_position in segment_positions:
            segment_position_converted = self.convert_to_xy(segment_position)
            self.get_logger().info(f'position {position}')
            self.get_logger().info(f'segment_position {segment_position_converted}')
            distance = np.linalg.norm(position - segment_position_converted)
            if distance < min_distance:
                min_distance = distance
                closest_segment = segment_position
        
        return closest_segment

    def convert_to_xy(self, position):
        position = str(position)
        x, y = [float(xy) for xy in position.split('.')]
        return np.array([x, y])

    def send_goal_for_shuttles(self, goal, id):
        # Function that send the goal for multiple shuttles
        self.get_logger().info(f'inside send goal for shuttles {id}')
        goal_msg = Shu.Goal()
        goal_msg.id = id
        goal_msg.goal = goal
        self.get_logger().info(f'debug #1')
        self.action_clients_shuttle[id].wait_for_server()
        self.get_logger().info(f'debug #2')
        future = self.action_clients_shuttle[id].send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info(f'debug #3')
        future.add_done_callback(self.shuttle_goal_response_callback)
        self.get_logger().info(f'debug #4')
        self.get_logger().info(f'end of send goal for shuttles {id}')

    def shuttle_feedback_callback(self, feedback_msg):
        pass
        # feedback = feedback_msg.feedback
        # self.get_logger().info('Received feedback: {0}'.format(feedback.goalfeedback))

    def shuttle_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted for shuttle')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.shuttle_get_result_callback)

    def shuttle_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.id))
        #status = 'Goal succeeded!' if result.succes else 'Goal failed!'

        # 1. 
        active_tasks = self.task_planner.get_active_tasks()
        if active_tasks:
            for idx, active_task in enumerate(active_tasks):
                shuttle_id = active_task.get_assigned_shuttle_id()
                if result.id == shuttle_id:
                    active_tasks.pop(idx)
                    self.shuttles_id_to_storage_station.append(shuttle_id)
                    self.get_logger().info(f'active task idx {idx}:  {active_task.get_in_progress()}')
                    self.task_planner.free_station(active_task.get_assigned_station())
                    self.get_logger().info(f'active task idx {idx}:  {active_task.get_in_progress()}')

        if self.shuttles_waiting_for_storage:
            for idx, shuttle_id in enumerate(self.shuttles_waiting_for_storage):
                if result.id == shuttle_id:
                    self.shuttles_waiting_for_storage.pop(idx)
                    self.task_planner.free_shuttle(self.task_planner.shuttles.get_shuttle_by_index(shuttle_id))
                    self.get_logger().info(f'shuttle {shuttle_id} is free')

def main(args=None):
    """Initializes the Manipulator node and spins it until it is destroyed."""
    rclpy.init(args=args)

    task_node = TaskPlanner()

    # goal = [0.0, -pi/2, pi/2, 0.0, 0.0, 0.0]

    # task_node.Man_send_goal(goal)
    rclpy.spin(task_node)





if __name__ == '__main__':
    main()
