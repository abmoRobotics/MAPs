
import math
from sensor_msgs.msg import JointState


class ShuttleMode():
    """
    Holds a number of different task that the ACOPOS 6D can do.
    : move      Move the shuttel to different positions (X,Y)
    : rotate    Rotate the shuttle in a specific RPM (Joint Z)
    : shake     Lift and sink the shuttle in Z to make i shake
    : tumble    Tumble the shuttle around the Z joint, moving the X and Y joint.
    
    """

    def __init__(self):

        self.msg = JointState()
        
    def move(self, position: float, velocity: float):
        """
        Move the shuttel to different positions (X,Y,Z)
        """
        self.msg.position = position

        self.msg.velocity = velocity
  
        return self.msg
        
        

    def rotate(self, RPM: int, time: float):
        """
        Rotate the shuttle in a specific RPM (Joint Z)
        """
        
        self.msg.position = [0,0,0,0,0,1]

        self.msg.velocity = [0,0,0,0,0,(RPM/60)*360]

        return self.msg





    def shake(self, RPM: int):
        """
        Lift and sink the shuttle in Z to make i shake
        """

        self.msg.position = [0, 0, 1, 0, 0, 0]

        self.msg.velocity = [0, 0, 2, 0, 0, 0]

        return self.msg


    def tumbler(self):
        
        return self.msg


    def dispensing(self):
        
        return self.msg