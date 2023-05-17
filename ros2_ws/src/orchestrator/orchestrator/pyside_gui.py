import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import rclpy
from rclpy.node import Node

from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterDescriptor, ParameterValue, Parameter
from rclpy.parameter import ParameterType


class Color(QWidget):

    def __init__(self, color):
        super(Color, self).__init__()
        self.setAutoFillBackground(True)

        palette = self.palette()
        palette.setColor(QPalette.Window, QColor(color))
        self.setPalette(palette)

class Pos(QWidget):

    shuttle = pyqtSignal(int)
    robot = pyqtSignal(int)
    default_robot = pyqtSignal(int)
    default_shuttle = pyqtSignal(int)

    robot_position = []
    table_position = []

    


    def __init__(self, x, y, *args, **kwargs):
        super(Pos, self).__init__(*args, **kwargs)

        self.setFixedSize(QSize(20, 20))

        self.x = x
        self.y = y

        self.draw_robot_img = False
        self.draw_shuttle_img = False
        self.draw_default_robot = False
        self.draw_default_shuttle = False

        

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = event.rect()
        p.drawRect(r)

        # Draw robot
        if self.draw_robot_img and not self.draw_shuttle_img: 
            #p.begin()
            #Node("gui").get_logger().info("Position robot: " + str(self.x))
            p.fillRect(r,QColor(Qt.gray))
            self.robot_position.append(float('%d.%d' % (self.x, self.y)))
            print("Robot  position: " + str(self.robot_position))
            # Parameter('robot_position',Parameter.Type.INTEGER_ARRAY,self.robot_position)
            self.draw_default_shuttle = False
            self.draw_default_robot = False
            self.draw_shuttle_img = False
            #self.update()
            #p.end()

        # Draw shuttle
        if self.draw_shuttle_img and not self.draw_robot_img:
            #p.begin()
            p.fillRect(r,QColor(Qt.black))
    
            self.table_position.append(float('%d.%d' % (self.x, self.y)))
            print("Position table: " + str(self.table_position))
            #Parameter('table_position',Parameter.Type.INTEGER_ARRAY,self.table_position)
            self.draw_default_robot = False
            self.draw_default_shuttle = False
            self.draw_robot_img = False
            #self.update()


        # Reset robot or shuttle
        if self.draw_default_robot:
            #p.begin()
            p.eraseRect(r)
            self.robot_position.remove(float('%d.%d' % (self.x, self.y)))
            print("Robot  position: " + str(self.robot_position))
            p.drawRect(r)
            self.draw_robot_img = False
            #p.end()

        if self.draw_default_shuttle:
            #p.begin()
            p.eraseRect(r)
            self.table_position.remove(float('%d.%d' % (self.x, self.y)))
            print("Position table: " + str(self.table_position))
            p.drawRect(r)
            self.draw_shuttle_img = False
        
        


    def mouseReleaseEvent(self, e):

        if (e.button() == Qt.RightButton) and self.draw_robot_img:
            self.draw_robot_img = False
            self.draw_default_robot = True
            self.default_robot.emit(1)
            self.update()

        elif (e.button() == Qt.LeftButton) and self.draw_shuttle_img:
            self.draw_shuttle_img = False
            self.draw_default_shuttle = True
            self.default_shuttle.emit(1)
            self.update()

        elif (e.button() == Qt.RightButton):
            self.draw_robot_img = True
            self.draw_default_robot = False
            self.robot.emit(1)
            self.update()

        elif (e.button() == Qt.LeftButton):
            self.draw_shuttle_img = True
            self.draw_default_shuttle = False
            self.shuttle.emit(1)
            self.update()

        

class Input(QLineEdit):
    def __init__(self, placeholder):
        super(Input, self).__init__()

        self.lenght = 10
        self.width = 10

        self.setPlaceholderText(placeholder)
        self.setValidator(QIntValidator(0,1000,self))
        self.textChanged.connect(MainWindow.input)

class Grid(QGridLayout):
    def __init__(self, input_width, input_lenght):
        super(Grid, self).__init__()
        self.setSpacing(0)
        self.width = input_width
        self.lenght = input_lenght
        rclpy.init()
        self.gui = Node("gui")

        self.num_robot = 0
        self.num_tabels = 0

        self.client = self.gui.create_client(GetParameters,
                                        '/global_parameter_server/get_parameters')
        request = GetParameters.Request()
        # self.cli = self.gui.create_client(SetParameters, '/global_parameter_server/set_parameters')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.gui.get_logger().warn('111service not available, waiting again...')

        #self.req = SetParameters.Request()
        #input = MainWindow()
        #self.send_request()
        

        request.names = ['num_of_shuttles']
        # while not self.client.wait_for_service(timeout_sec=3):
        #      self.gui.get_logger().warn('222service not available, waiting again...')
        future = self.client.call_async(request=request)

        future.add_done_callback(self.callback_global_param)
        self.result = future.result()


        # self.num_robot =  self.gui.get_parameter('num_of_manipulators').get_parameter_value().integer_value
        # self.num_tabels = self.gui.get_parameter('num_of_tabels').get_parameter_value().integer_value
        #self.num_shuttels = self.gui.get_parameter('num_of_shuttles').get_parameter_value().integer_value
    
        # self.robot_position = []
        # self.table_position = []
        
        # self.gui.declare_parameter('robot_position', self.robot_position)
        # self.gui.declare_parameter('table_position', self.table_position)

        self.setSizeConstraint(QLayout.SetFixedSize)
        self.shuttle = False
        self.but = None
        self.create_grid(input_width, input_lenght)

    def callback_global_param(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.gui.get_logger().warn("service call failed %r" % (e,))
        else:
            self.param = result.values[0].integer_value
            self.gui.get_logger().error("Got global param: %s" % (self.param,))

        
    
    # def send_request(self, value: int, name: str):
    #     new_param_value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    #     self.req.parameters = [Parameter(name='num_of_shuttles', value=new_param_value)]
    #     self.future = self.cli.call_async(self.req)
            
        

    def update_grid(self, input_width, input_lenght):
        # Delete the old grid, and create a new 
        for x in range(0, self.width):
            for y in range(0, self.lenght):
                w = Pos(x, y)
                self.itemAtPosition(y, x).widget().deleteLater()

        self.width = input_width
        self.lenght = input_lenght

        self.create_grid(input_width, input_lenght)

    def create_grid(self, input_width, input_lenght):
        #Create the map grid
        for x in range(0,self.width):
            for y in range(0, self.lenght):
                self.w = Pos(x, y)
                self.w.robot.connect(self.draw_robot)
                self.w.shuttle.connect(self.draw_shuttle)
                self.w.default_robot.connect(self.draw_default_robot_img)
                self.w.default_shuttle.connect(self.draw_default_shuttle_img)
                self.addWidget(self.w,y,x)
    
    def draw_robot(self):
        self.num_robot += 1
        # Initial setup for parameters
        cli = self.gui.create_client(SetParameters, '/global_parameter_server/set_parameters')
        req = SetParameters.Request()

        # Parameters for number of manipulators
        new_param_value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, 
                                         integer_value=int(self.num_robot))
        req.parameters = [Parameter(name='num_of_manipulators', 
                                         value=new_param_value)]
        self.future = cli.call_async(req)

        # Parameters for manipulator position
        robot_pos_param = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, 
                                         double_array_value=self.w.robot_position)
        req.parameters = [Parameter(name='robot_position', 
                                         value=robot_pos_param)]
        self.future = cli.call_async(req)

        self.gui.get_logger().warn("Robot positions: " + str(self.w.robot_position))
        print("Number of robot:", self.num_robot)
        self.draw_robot_img = True

    def draw_shuttle(self):
        self.num_tabels += 1
        # Initial setup for parameters
        cli = self.gui.create_client(SetParameters, '/global_parameter_server/set_parameters')
        req = SetParameters.Request()

        # Parameters for number of tabels
        new_param_value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, 
                                         integer_value=int(self.num_tabels))
        req.parameters = [Parameter(name='num_of_tabels', 
                                         value=new_param_value)]
        self.future = cli.call_async(req)
        
        # Parameters for table position
        table_pos_param = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, 
                                         double_array_value=self.w.table_position)
        req.parameters = [Parameter(name='table_position', 
                                         value=table_pos_param)]
        self.future = cli.call_async(req)

        self.gui.get_logger().warn("Table positions: " + str(self.w.table_position))
        print("Number of tables:", self.num_tabels)
        self.draw_shuttle_img = True

    def draw_default_robot_img(self):
        self.num_robot -= 1
        # Initial setup for parameters
        cli = self.gui.create_client(SetParameters, '/global_parameter_server/set_parameters')
        req = SetParameters.Request()

        # Parameters for number of manipulators
        new_param_value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, 
                                         integer_value=int(self.num_robot))
        req.parameters = [Parameter(name='num_of_manipulators', 
                                         value=new_param_value)]
        self.future = cli.call_async(req)

        # Parameters for manipulator position
        robot_pos_param = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, 
                                         double_array_value=self.w.robot_position)
        req.parameters = [Parameter(name='robot_position', 
                                         value=robot_pos_param)]
        self.future = cli.call_async(req)

        self.gui.get_logger().warn("Robot positions: " + str(self.w.robot_position))
        print("Number of robot:", self.num_robot)
        self.draw_default_robot = True

    def draw_default_shuttle_img(self):
        self.num_tabels -= 1
        # Initial setup for parameters
        cli = self.gui.create_client(SetParameters, '/global_parameter_server/set_parameters')
        req = SetParameters.Request()

        # Parameters for number of tabels
        new_param_value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, 
                                         integer_value=int(self.num_tabels))
        req.parameters = [Parameter(name='num_of_tabels', 
                                         value=new_param_value)]
        self.future = cli.call_async(req)

        # Parameters for table position
        table_pos_param = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, 
                                         double_array_value=self.w.table_position)
        req.parameters = [Parameter(name='table_position', 
                                         value=table_pos_param)]
        self.future = cli.call_async(req)

        self.gui.get_logger().warn("Table positions: " + str(self.w.table_position))
        print("Number of tables:", self.num_tabels)
        self.draw_default_shuttle = True


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        
        
        self.setWindowTitle("Interface for shuttles and robots")

        # Setting the line edit up for 
        self.line_w = QLineEdit()
        self.line_w.setPlaceholderText("Width")
        self.line_w.setValidator(QIntValidator(0,99,self))
        self.line_w.textChanged.connect(self.input)

        self.line_l = QLineEdit()
        self.line_l.setPlaceholderText("Lenght")
        self.line_l.setValidator(QIntValidator(0,99,self))
        self.line_l.textChanged.connect(self.input)

        self.num_shu = QLineEdit()
        self.num_shu.setPlaceholderText("Number of shuttels")
        self.num_shu.setValidator(QIntValidator(0,80,self))
        self.num_shu.textChanged.connect(self.num_shuttle)

        lenght = 3
        width = 3

        layout1 = QHBoxLayout()
        self.width2 = 0
        self.lenght2 = 0

        but_create = QPushButton("Create grid")
        but_stop = QPushButton("Stop")

        robot = QPushButton("Robot")
        shuttle = QPushButton("Shuttle")
        layout1.addWidget(robot)
        layout1.addWidget(shuttle)

        layout1.addWidget(but_create)
        layout1.addWidget(but_stop)
        layout1.addWidget(self.num_shu, stretch=0.2)
        layout1.addWidget(self.line_w, stretch=0.2)
        layout1.addWidget(self.line_l, stretch=0.2)


        self.grid = Grid(width, lenght)
        layout1.addLayout(self.grid)
        but_create.clicked.connect(self.update1)

        layout1.setContentsMargins(0,0,0,0)
        layout1.setSpacing(10)

        widget = QWidget()
        widget.frameSize()
        widget.setLayout(layout1)
        self.setCentralWidget(widget)
        
    def update1(self):
        # Update the grid
        self.grid.update_grid(int(self.line_w.text()), int(self.line_l.text()))

    def num_shuttle(self):
        # Update the number of shuttels
        # Send the number of shuttles to param server
        self.cli = self.grid.gui.create_client(SetParameters, '/global_parameter_server/set_parameters')
        self.req = SetParameters.Request()
        new_param_value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, 
                                         integer_value=int(self.num_shu.text()))
        self.req.parameters = [Parameter(name='num_of_shuttles', 
                                         value=new_param_value)]
        self.future = self.cli.call_async(self.req)

    def input(self):
        # Take the input from the line edit and return it
        return self.width2, self.lenght2, 


def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    executor = MultiThreadedExecutor()

    executor.add_node(window.grid.gui)
    thread = Thread(target=executor.spin)
    thread.start()

    try:
        window.show()
        sys.exit(app.exec())

    finally:
        window.grid.gui.get_logger().info("Shutting down ROS2 Node . . .")
        # window.grid.gui.destroy_node()
        # executor.shutdown()
if __name__ == '__main__':
    main()