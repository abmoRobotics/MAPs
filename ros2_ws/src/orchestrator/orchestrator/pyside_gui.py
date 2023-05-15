import sys
from PySide6.QtWidgets import *
from PySide6.QtGui import *
from PySide6.QtCore import *


class Color(QWidget):

    def __init__(self, color):
        super(Color, self).__init__()
        self.setAutoFillBackground(True)

        palette = self.palette()
        palette.setColor(QPalette.Window, QColor(color))
        self.setPalette(palette)

class Pos(QWidget):

    shuttle = Signal(int)
    robot = Signal(int)
    default_robot = Signal(int)
    default_shuttle = Signal(int)


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
            p.fillRect(r,QBrush(Qt.gray))
            print("Position robot: ", self.x, self.y)
            self.draw_default_shuttle = False
            self.draw_default_robot = False
            self.draw_shuttle_img = False

        # Draw shuttle
        if self.draw_shuttle_img and not self.draw_robot_img:
            p.fillRect(r,QBrush(Qt.black))
            print("Position shuttle: ", self.x, self.y)
            self.draw_default_robot = False
            self.draw_default_shuttle = False
            self.draw_robot_img = False

        # Reset robot or shuttle
        if self.draw_default_robot:
            p.eraseRect(r)
            p.drawRect(r)
            self.draw_robot_img = False
            self.update()

        if self.draw_default_shuttle:
            p.eraseRect(r)
            p.drawRect(r)
            self.draw_shuttle_img = False
            self.update()


    def mouseReleaseEvent(self, e):

        if (e.button() == Qt.RightButton) and self.draw_robot_img:
            self.draw_robot_img = False
            self.draw_default_robot = True
            self.default_robot.emit(1)

        elif (e.button() == Qt.LeftButton) and self.draw_shuttle_img:
            self.draw_shuttle_img = False
            self.draw_default_shuttle = True
            self.default_shuttle.emit(1)

        elif (e.button() == Qt.RightButton):
            self.draw_robot_img = True
            self.draw_default_robot = False
            self.robot.emit(1)

        elif (e.button() == Qt.LeftButton):
            self.draw_shuttle_img = True
            self.draw_default_shuttle = False
            self.shuttle.emit(1)

        

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

        self.num_robot = 0
        self.num_shuttle = 0

        self.setSizeConstraint(QLayout.SetFixedSize)
        self.shuttle = False
        self.but = None
        self.create_grid(input_width, input_lenght)

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
        print("Number of robot:", self.num_robot)
        self.draw_robot_img = True

    def draw_shuttle(self):
        self.num_shuttle += 1
        print("Number of tables:", self.num_shuttle)
        self.draw_shuttle_img = True

    def draw_default_robot_img(self):
        self.num_robot -= 1
        print("Number of robot:", self.num_robot)
        print("I am default robot now")
        self.draw_default_robot = True

    def draw_default_shuttle_img(self):
        self.num_shuttle -= 1
        print("Number of tables:", self.num_shuttle)
        print("I am default shuttle now")
        self.draw_default_shuttle = True


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        
        
        self.setWindowTitle("Interface for shuttles and robots")

        # Setting the line edit up for 
        self.line_w = QLineEdit()
        self.line_w.setPlaceholderText("Width")
        self.line_w.setValidator(QIntValidator(0,1000,self))
        self.line_w.textChanged.connect(self.input)


        self.line_l = QLineEdit()
        self.line_l.setPlaceholderText("Lenght")
        self.line_l.setValidator(QIntValidator(0,1000,self))
        self.line_l.textChanged.connect(self.input)

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
        self.show()

    def update1(self):
        # Update the grid
        self.grid.update_grid(int(self.line_w.text()), int(self.line_l.text()))


    def input(self):
        # Take the input from the line edit and return it
        return self.width2, self.lenght2, 
        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    app.exec()