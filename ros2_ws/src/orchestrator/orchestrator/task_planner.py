import rclpy
from rclpy.node import Node
import numpy as np
from math import pi

from robot_actions.action import Man
from robot_actions.action import Shu

from utils import create_action_client_array
from utils import KR4R600, KR3R540




class TaskPlanner(Node):

    def __init__(self):
        super().__init__('task_planner')
        # Define publishers and messages

        number_of_kuka540 = 2
        number_of_kuka600 = 2

        self.number_of_shuttels = 5 * ["Shuttle"]

        name540 = number_of_kuka540 * [KR3R540()._name]
        name600 = number_of_kuka600 * [KR4R600()._name]

        self.name_of_manipulators = np.concatenate((name600, name540))


        goal = [0, -pi/2, pi/2, 0, 0, 0]

        for idx, name in enumerate(self.name_of_manipulators):
            self.ManiActionClient = create_action_client_array(self, 
                                                               Man, 
                                                               topic_prefix=name,
                                                               n_robots=1) 
            
        for idx, name in enumerate(self.number_of_shuttels):
            self.ShuActionClient = create_action_client_array(self,
                                                              Shu,
                                                              topic_prefix=name,
                                                              n_robots=1
                                                              )

    def Man_send_goal(self, goal):
        # Function that send the goal for multiple manipulators
        goal_msg = Man.Goal()
        goal_msg.goal = goal

        for idx, Client in enumerate(self.ManiActionClient):
            self.ManiActionClient[idx].wait_for_server()
            future = self.ManiActionClient[idx].send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            future.add_done_callback(self.goal_response_callback) 


    def Shu_send_goal(self, goal):
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



def main(args=None):
    """Initializes the Manipulator node and spins it until it is destroyed."""
    rclpy.init(args=args)

    task_node = TaskPlanner()

    goal = [0.0, -pi/2, pi/2, 0.0, 0.0, 0.0]

    task_node.Man_send_goal(goal)
    rclpy.spin(task_node)

if __name__ == '__main__':
    main()
