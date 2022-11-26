import sys, os
import backend
from time import sleep
from gtts import gTTS
from pygame import mixer
from PyQt5.QtWidgets import QApplication, QWidget, QComboBox, QHBoxLayout, QVBoxLayout, QLabel, QPushButton
from PyQt5.QtGui import *
from PyQt5.QtCore import QSize, Qt
from PyQt5 import QtTest

class App(QWidget):
    def __init__(self):
        super().__init__()
        self.audio('Welcome to Beamish Munro Hall. My name is George. I will guide you to your destination within the building. Would you like to go to floor 1, 2, or 3?')
        self.rooms = ['Select']
        self.room_num = None;
        self.setup()
        self.select_floor()
        self.button.clicked.connect(self.directions)
        self.show()

    def setup(self):
        #self.setStyleSheet("QWidget {background-color: rgb(255, 255, 255);}")
        self.setWindowTitle("Autonomous Building Guide")
        #self.setMinimumWidth(600)

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.label = QLabel('Welcome to Beamish Munro Hall.')

        self.pic = QLabel()
        pixmap = QPixmap(os.getcwd() + '/Queens_Logo.jpeg')
        pixmap = pixmap.scaled(128, 128, Qt.KeepAspectRatio)
        self.pic.setPixmap(pixmap)

        self.layout.addWidget(self.label)
        #self.layout.addWidget(self.pic)


    def audio(self,message):
        language = 'en'
        myobj = gTTS(text=message, lang=language, slow=False)
        myobj.save("message.mp3")
        mixer.init()
        mixer.music.load("message.mp3")
        mixer.music.play()

    def select_floor(self):
        self.l1 = QLabel('Select Floor: ')
        self.combobox1 = QComboBox()
        self.combobox1.addItems(['Select', '1', '2', '3'])
        self.combobox1.currentTextChanged.connect(self.listRooms)
        self.layoutH1 = QHBoxLayout()
        self.layoutH1.addWidget(self.l1)
        self.layoutH1.addWidget(self.combobox1)

        self.l2 = QLabel('Select Room: ')
        self.combobox2 = QComboBox()
        self.combobox2.addItems(self.rooms)
        self.layoutH2 = QHBoxLayout()
        self.combobox2.currentTextChanged.connect(self.destination)
        self.layoutH2.addWidget(self.l2)
        self.layoutH2.addWidget(self.combobox2)

        self.layout.addLayout(self.layoutH1)
        self.layout.addLayout(self.layoutH2)

        self.button = QPushButton("Continue")
        self.layout.addWidget(self.button)
        self.button.setEnabled(False)

    def listRooms(self,value):
        self.rooms = ["Select"]
        rooms = backend.listByFloor(int(value))
        for i in rooms:
            self.rooms.append(i)
        self.combobox2.clear()
        self.combobox2.addItems(self.rooms)
        self.audio("Room options are ")
        QtTest.QTest.qWait(2000)

        for i in self.rooms:
            if i != "Select":
                self.audio(i)
                QtTest.QTest.qWait(1000)

    # Confirm destination
    def destination(self,value):
        if (value != "Select"):
            text = "You have selected room " + value + ". Press continue if that is correct."
            self.audio(text)
            self.room_num = value
            self.button.setEnabled(True)

    # List Obstacles in Path
    def directions(self):
        obstacles = backend.getObstackeList();

        # Clear Screen
        self.clearWidget(self.combobox1)
        self.clearWidget(self.combobox2)
        self.clearWidget(self.l1)
        self.clearWidget(self.l2)
        self.clearWidget(self.button)

        self.label.setText("Obstacles in the path are:")
        self.audio("Obstacles in the path are.")
        QtTest.QTest.qWait(2000)
        self.temp = QLabel()
        self.layout.addWidget(self.temp)

        if obstacles:
            for i in obstacles:
                self.temp.setText(i)
                self.audio(i)
                QtTest.QTest.qWait(2000)

        else:
            self.audio("None.")

        self.audio("Do you wish to continue?")

        self.button2 = QPushButton("Continue")
        self.layout.addWidget(self.button2)
        self.button2.clicked.connect(self.navigating)

    # Screen when navigating to destination
    def navigating(self):
        self.label.setText("Going to room " + self.room_num)

        # Clear screen
        self.clearWidget(self.temp)

        # Robot should begin moving here
        self.audio("Going to room " + self.room_num)
        QtTest.QTest.qWait(3000)

        # Play elevator music
        mixer.init()
        mixer.music.load("Georgeofthejungle.mp3")
        mixer.music.play()


    # Clear Widget
    def clearWidget(self, item):
        self.layout.removeWidget(item)
        item.deleteLater()
        item = None

if __name__ == '__main__':
    app = QApplication(sys.argv)

    window = App()
    window.show()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        print('Closing Window...')
