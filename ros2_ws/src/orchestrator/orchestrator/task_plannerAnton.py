import rclpy

from rclpy.node import Node

import time

import heapq

from typing import List



class Station():
    """Class for a single station"""
    def __init__(self) -> None:
        self.available = True

    def get_position(self):
        pass

    def set_position(self):
        pass

    def get_availibility(self):
        return self.available
    
    def set_availibility(self, available: bool):
        self.available = available

class Task():
    """Class for a single task"""
    def __init__(self) -> None:
        creation_time = time.time()

    def get_creation_time(self):
        return self.creation_time

class Shuttle():
    """Class for a single shuttle"""
    def __init__(self) -> None:
        self.available = True

    def get_position(self):
        pass

    def set_position(self):
        pass

    def get_availibility(self):
        return self.available
    
    def set_availibility(self, available: bool):
        self.available = available

class StationManager():
    """Class for keeping track of the stations"""
    def __init__(self) -> None:
        self.stations = []

    def add_station(self, station: Station):
        self.stations.append(station)

    def remove_station(self, station: Station):
        self.stations.remove(station)

    def get_stations(self):
        return self.stations
    
    def set_status(self, station: Station, status: bool):
        station.set_availibility(status)

class TaskManager():
    """Class for keeping track of the tasks"""
    def __init__(self) -> None:
        self.tasks =  []

    def add_task(self, task: Task):
        self.tasks.append(task)
    
    def remove_task(self, task: Task):
        self.tasks.remove(task)

    def get_tasks(self):
        return self.tasks

class ShuttleManager():
    """Class for keeping track of the shuttles"""
    def __init__(self) -> None:
        self.shuttles = []

    def add_shuttle(self, shuttle: Shuttle):
        self.shuttles.append(shuttle)

    def remove_shuttle(self, shuttle: Shuttle):
        self.shuttles.remove(shuttle)

    def get_shuttles(self):
        return self.shuttles
    
    def get_available_shuttles(self):
        pass

    def set_status(self, shuttle: Shuttle, status: bool):
        shuttle.set_availibility(status)

class TaskPlanner():
    """ Node for planning and orchestrating tasks for the manipulators and shuttles"""
    def __init__(self) -> None:
        self.stations = StationManager()
        self.tasks = TaskManager()
        self.shuttles = ShuttleManager()
        self.active_tasks: List[Task] = []
        self.heuristic_spatial_weight = 0.5
        self.heuristic_temporal_weight = 0.5

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
                heapq.heappush(priority_scores_heap, (score, station, shuttle))
               

        # 4. Return the lowest priority score

        score, station, shuttle = heapq.heappop(priority_scores_heap)
        return score, station, shuttle

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
        for task in tasks:
            score, station, shuttle = self.priority_scores(task)
            if score < min_score:
                min_score = score
                min_station = station
                min_shuttle = shuttle
                min_task = task
        


        # 6. Assign the task with the lowest priority score to the available shuttle and manipulator
        # task = 
        # 6.1. Update the station list
        self.stations.set_status(min_station, False)
        # 6.2. Update the shuttle list
        self.shuttles.set_status(min_shuttle, False)

        # 6.3. Update the manipulator list

        # 7. Update the task list
        self.active_tasks.append(min_task)
        self.tasks.remove_task(min_task)

    def assign_task(self, task: Task):
        """Function to assign a task to a shuttle and manipulator"""

    def get_tasks(self):
        """Function to get the tasks from the task manager"""
        if self.tasks.get_tasks():
            return self.tasks.get_tasks()
        else:
            return None
    
    def get_available_stations(self):
        """Function to get the stations from the station manager"""
        if self.stations.get_stations():
            return self.stations.get_stations()
        else:
            return None
        
    def get_available_shuttles(self):
        """Function to get the shuttles from the shuttle manager"""
        if self.shuttles.get_shuttles():
            return self.shuttles.get_shuttles()
        else:
            None

    