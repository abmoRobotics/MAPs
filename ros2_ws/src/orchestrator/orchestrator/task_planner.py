import rclpy
from rclpy.node import Node
import numpy as np
from robot_actions.action import Man

from utils import create_action_client_array
from utils import KR4R600, KR3R540




class TaskPlanner(Node):

    def __init__(self):
        super().__init__('task_planner')
        # Define publishers and messages

        number_of_kuka540 = 0
        number_of_kuka600 = 8

        name540 = number_of_kuka540 * [KR3R540()._name]
        name600 = number_of_kuka600 * [KR4R600()._name]

        self.name_of_manipulators = np.concatenate((name600, name540))
        for idx, name in enumerate(self.name_of_manipulators):
            self.ManiActionClient = create_action_client_array(self, 
                                                               Man, 
                                                               topic_prefix=name,
                                                               n_robots=1) 

    def goal_callback(self):
        pass
        




def main(args=None):
    """Initializes the Manipulator node and spins it until it is destroyed."""
    rclpy.init(args=args)

    task_node = TaskPlanner()

    rclpy.spin(task_node)

    task_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
