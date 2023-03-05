
from rclpy.node import Node



class ShuttleMode(Node):
    """
    Holds a number of different task that the ACOPOS 6D can do.
    : move      Move the shuttel to different positions (X,Y)
    : rotate    Rotate the shuttle in a specific RPM (Joint Z)
    : shake     Lift and sink the shuttle in Z to make i shake
    : tumble    Tumble the shuttle around the Z joint, moving the X and Y joint.
    
    """
    def move(position: float):
        """
        Move the shuttel to different positions (X,Y)
        """
        pass

    def rotate( rpm: int):
        """
        Rotate the shuttle in a specific RPM (Joint Z)
        """
        pass




    def shake( Hz: int):
        """
        Lift and sink the shuttle in Z to make i shake
        """
        pass



    def tumbler():
        pass


    def dispensing():
        pass