# import rclpy

# from rclpy.node import Node

import time

import heapq

from typing import List

import numpy as np

import copy

class Station():
    """Class for a single station"""
    def __init__(self) -> None:
        self.available = True

    def get_position(self):
        return self.position

    def set_position(self, position: np.ndarray):
        if position.size != 2:
            raise ValueError(f"position should be a 2D vector, but got {position.size}D vector, {position}")
            
        self.position = position
        
    def get_availibility(self):
        return self.available
    
    def set_availibility(self, available: bool):
        self.available = available


class Shuttle():
    """Class for a single shuttle"""
    def __init__(self) -> None:
        self.available = True
        self.position = np.array([0,0])
        self.index = None

    def get_position(self):
        return self.position

    def set_position(self, position: np.ndarray):
        if position.size != 2:
            raise ValueError(f"position should be a 2D vector, but got {position.size}D vector, {position}")
            
        self.position = position

    def get_availibility(self):
        return self.available
    
    def set_availibility(self, available: bool):
        self.available = available

    def get_index(self):
        return self.index
    
    def set_index(self, index: int):
        self.index = index
class Task():
    """Class for a single task"""
    def __init__(self, task) -> None:
        self.creation_time = time.time()
        self.task = task
        self.assigned_shuttle_id = None
        self.in_progress = False
        self.assigned_station = None

    def get_creation_time(self):
        return self.creation_time

    def get_task(self):
        return self.task
    
    def set_assigned_shuttle_id(self, shuttle_id: int):
        self.assigned_shuttle_id = shuttle_id

    def get_assigned_shuttle_id(self):
        return self.assigned_shuttle_id
    
    def set_in_progress(self, status: bool):
        self.in_progress = status

    def get_in_progress(self):
        return self.in_progress
    
    def set_assigned_station(self, station: Station):
        self.assigned_station = station

    def get_assigned_station(self):
        return self.assigned_station
    
class StationManager():
    """Class for keeping track of the stations"""
    def __init__(self) -> None:
        self.stations = []

    def add_station(self, station: Station):
        self.stations.append(station)

    def remove_station(self, station: Station):
        self.stations.remove(station)

    def get_station_by_index(self, station_id: int) -> Station:
        return self.stations[station_id]

    def get_stations(self) -> List[Station]:
        return self.stations
    
    def set_status(self, station: Station, status: bool):
        station.set_availibility(status)

class TaskManager():
    """Class for keeping track of the tasks"""
    def __init__(self) -> None:
        self.tasks =  []

    def add_task(self, task: Task):
        assert isinstance(task, Task), "task is not of type Task"
        self.tasks.append(task)
    
    def remove_task(self, task: Task):
        self.tasks.remove(task)

    def get_tasks(self):
        return self.tasks

class ShuttleManager():
    """Class for keeping track of the shuttles"""
    def __init__(self) -> None:
        self.shuttles: List[Shuttle] = []

    def add_shuttle(self, shuttle: Shuttle):
        index = len(self.shuttles)
        self.shuttles.append(shuttle)
        self.shuttles[index].set_index(index)

    def remove_shuttle(self, shuttle: Shuttle):
        self.shuttles.remove(shuttle)

    def get_shuttles(self):
        return self.shuttles

    def get_shuttle_by_index(self, shuttle_id: int) -> Shuttle:
        return self.shuttles[shuttle_id]
    
    def get_available_shuttles(self):
        pass

    def set_status(self, shuttle: Shuttle, status: bool):
        shuttle.set_availibility(status)

class TaskPlannerUtils():
    """ Node for planning and orchestrating tasks for the manipulators and shuttles"""
    def __init__(self) -> None:


    

        self.stations = StationManager()
        self.tasks = TaskManager()
        self.shuttles = ShuttleManager()
        self.active_tasks: List[Task] = []
        self.heuristic_spatial_weight = 0.5
        self.heuristic_temporal_weight = 0.5

        tasks = create_fake_tasks()

        for task in tasks:
            self.tasks.add_task(Task(task))


    def main(self):
        # 1. Assign the next task to the available shuttle and manipulator
        self.next_task()
        # 2. Check if task is done

        # 3. Check if sub-task is done

    def timer_callback(self):
        self.main()

    def priority_scores(self, task: Task) -> float:
        """Function to calculate the priority score of a task"""

        # 1. Get the requirements of the task

        # 2. Get the available stations and shuttles that can fulfill the requirements
        stations = self.get_available_stations()
        shuttles = self.get_available_shuttles()


 

        # 3. Calculate the priority score for each station and shuttle and store them in a heapq

        priority_scores_heap = []
        for station in stations:
            for shuttle in shuttles:
                score = self.heuristic(task, station, shuttle)
                # Use heapq to store the scores
                heapq.heappush(priority_scores_heap, (score, station, shuttle, shuttle.get_index()))
               

        # 4. Return the lowest priority score

        score, station, shuttle, shuttle_idx = heapq.heappop(priority_scores_heap)
        return score, station, shuttle, shuttle_idx

    def heuristic(self, task: Task, station: Station, shuttle: Shuttle, materialShuttle: Shuttle = None) -> float:
        """Function to calculate the heuristic of a task"""
        spatial_priority_score = 0
        temporal_priority_score = 0
        ws = self.heuristic_spatial_weight
        wt = self.heuristic_temporal_weight
        

        if materialShuttle is not None:
            station_position = station.get_position()
            spatial_priority_score = (station_position - shuttle.get_position()) + (station_position - materialShuttle.get_position())
        else:
            spatial_priority_score = station.get_position() - shuttle.get_position()
        

        # 2. Calculate temporal priority score
        creation_time = task.get_creation_time()
        temporal_priority_score = time.time() - creation_time
        spatial_priority_score = np.linalg.norm(spatial_priority_score)        
        score = ws * spatial_priority_score + wt * temporal_priority_score
        return score

    def next_task(self):
        """Function to assign find the next task to be executed"""

        # 1. Check if there are any tasks
        tasks = self.get_tasks()
        if not tasks:
            return None
        # 2. Check if there are any stations available
        stations = self.get_available_stations()
        if not stations:
            return None

        # 3. Check if there are any shuttles available
        shuttles = self.get_available_shuttles()
        if not shuttles:
            return None

        # 4. Check if there are any manipulators available -> Not required for now

        # 5. Calculate the priority score of the tasks
        min_score = float('inf')
        min_station = None
        min_shuttle = None
        min_task = None
        idx_shuttle_min_task = None
        for task in tasks:
            score, station, shuttle, shuttle_idx = self.priority_scores(task)
            if score < min_score:
                min_score = score
                min_station = station
                min_shuttle = shuttle
                min_task = task
                idx_shuttle_min_task = shuttle_idx

        
        # 5.1 set the assigned shuttle to the task
        min_task.set_assigned_shuttle_id(idx_shuttle_min_task)
        min_task.set_assigned_station(min_station)

        # 6. Assign the task with the lowest priority score to the available shuttle and manipulator
        # task = 
        # 6.1. Update the station list
        self.stations.set_status(min_station, False)
        # 6.2. Update the shuttle list
        self.shuttles.set_status(min_shuttle, False)
        # 6.3

        # 6.3. Update the manipulator list

        # 7. Update the task list
        self.active_tasks.append(min_task)
        self.tasks.remove_task(min_task)
        print(self.active_tasks)

        
    def assign_task(self, task: Task):
        """Function to assign a task to a shuttle and manipulator"""

    def get_tasks(self) -> List[Task]:
        """Function to get the tasks from the task manager"""
        if self.tasks.get_tasks():
            return self.tasks.get_tasks()
        else:
            return None
        
    def get_active_tasks(self):
        """Function to get the active tasks from the task manager"""
        if self.active_tasks:
            return self.active_tasks
        else:
            return None
    
    def get_available_stations(self):
        """Function to get the stations from the station manager"""
        if self.stations.get_stations():
            stations = []
            for station in self.stations.get_stations():
                if station.get_availibility():
                    stations.append(station)
            return stations
        else:
            return None
        
    def get_available_shuttles(self):
        """Function to get the shuttles from the shuttle manager"""
        if self.shuttles.get_shuttles():
            shuttles = []
            for shuttle in self.shuttles.get_shuttles():
                if shuttle.get_availibility():
                    shuttles.append(shuttle)
            return shuttles
        else:
            None

    def free_station(self, station: Station):
        """Function to free a station"""
        self.stations.set_status(station, True)

    def free_shuttle(self, shuttle: Shuttle):
        """Function to free a shuttle"""
        self.shuttles.set_status(shuttle, True)
    
import random
def create_fake_tasks():
    """Function to create fake tasks"""
    fake1_actions = [{"name": 'add',
                      "material": {'value': '4-bromoaniline',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'add',
                      "material": {'value': 'dichloromethane',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'add',
                      "material": {'value': 'sodium hydroxide',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'stir',
                      "content": {'duration': {'value': 15, 'unit': 'seconds'}}},
                      {"name": 'store', 
                       "content": {'sample_name': {'value': 15, 'unit': 'seconds'}}}]
    fake2_actions = [{"name": 'add',
                      "material": {'value': '4-bromoaniline',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'add',
                      "material": {'value': 'dichloromethane',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'add',
                      "material": {'value': 'sodium hydroxide',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'stir',
                      "content": {'duration': {'value': 15, 'unit': 'seconds'}}},
                      {"name": 'store', 
                       "content": {'sample_name': {'value': 15, 'unit': 'seconds'}}}]
    fake3_actions = [{"name": 'add',
                      "material": {'value': '4-bromoaniline',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'add',
                      "material": {'value': 'dichloromethane',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'add',
                      "material": {'value': 'sodium hydroxide',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'stir',
                      "content": {'duration': {'value': 15, 'unit': 'seconds'}}},
                      {"name": 'store', 
                       "content": {'sample_name': {'value': 15, 'unit': 'seconds'}}}]
    fake4_actions = [{"name": 'add',
                      "material": {'value': '4-bromoaniline',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'add',
                      "material": {'value': 'dichloromethane',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'add',
                      "material": {'value': 'sodium hydroxide',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'stir',
                      "content": {'duration': {'value': 15, 'unit': 'seconds'}}},
                      {"name": 'store', 
                       "content": {'sample_name': {'value': 15, 'unit': 'seconds'}}}]
    fake5_actions = [{"name": 'add',
                      "material": {'value': '4-bromoaniline',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'add',
                      "material": {'value': 'dichloromethane',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'add',
                      "material": {'value': 'sodium hydroxide',
                                   "quantity": {'value': 15, 'unit': 'ml'}},},
                     {"name": 'stir',
                      "content": {'duration': {'value': 15, 'unit': 'seconds'}}},
                      {"name": 'store', 
                       "content": {'sample_name': {'value': 15, 'unit': 'seconds'}}}]


    fake_tasks = [fake1_actions, fake2_actions, fake3_actions, fake4_actions, fake5_actions]
    
    tasks = [task for task in fake_tasks for _ in range(10)]
    random.shuffle(tasks)

    return tasks


if __name__ == '__main__':
    tasks = create_fake_tasks()
    task_planner = TaskPlannerUtils()

    for task in tasks:

        task_planner.tasks.add_task(Task(task))

    Station1 = Station()
    Station2 = Station()
    Station3 = Station()

    Station1.set_position(np.array([0,0]))
    Station2.set_position(np.array([1,0]))
    Station3.set_position(np.array([2,0]))

    task_planner.stations.add_station(Station1)
    task_planner.stations.add_station(Station2)
    task_planner.stations.add_station(Station3)

    Shuttle1 = Shuttle()
    Shuttle2 = Shuttle()
    Shuttle3 = Shuttle()

    Shuttle1.set_position(np.array([2,2]))
    Shuttle2.set_position(np.array([3,3]))
    Shuttle3.set_position(np.array([4,4]))  

    task_planner.shuttles.add_shuttle(Shuttle1)
    task_planner.shuttles.add_shuttle(Shuttle2)
    task_planner.shuttles.add_shuttle(Shuttle3)
    
    #task_planner.main()
    for i in range(4):
        task_planner.main()

    active_tasks = task_planner.get_active_tasks()
    for active_task in active_tasks:
        #print(f'Active task: {active_task.get_task()}')
        print(f'Assigned shuttle id: {active_task.get_assigned_shuttle_id()}')
        print(f'Active task: {active_task.get_task()}')
    

    #print(f'Get available shuttles: {task_planner.get_available_shuttles()}')
    #print(f'Get available stations: {task_planner.get_available_stations()}')

    

    # 1. Start executing the task
 