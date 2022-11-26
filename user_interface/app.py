import sys, os
import backend
from time import sleep
from gtts import gTTS
from pygame import mixer
from PyQt5.QtWidgets import QApplication, QWidget, QComboBox, QHBoxLayout, QVBoxLayout, QLabel
from PyQt5.QtGui import *
from PyQt5.QtCore import QSize, Qt

class App(QWidget):
    def __init__(self):
        super().__init__()
        self.audio('Welcome to Beamish Munro Hall. My name is George. I will guide you to your destination within the building. Would you like to go to floor 1, 2, or 3?')
        self.rooms = ['Select']
        self.setup()
        self.show()

    def setup(self):
        #self.setStyleSheet("QWidget {background-color: rgb(255, 255, 255);}")
        self.setWindowTitle("Autonomous Building Guide")
        self.setMinimumWidth(600)

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.label = QLabel('Welcome to Beamish Munro Hall.')

        self.pic = QLabel()
        pixmap = QPixmap(os.getcwd() + '/Queens_Logo.jpeg')
        pixmap = pixmap.scaled(128, 128, Qt.KeepAspectRatio)
        self.pic.setPixmap(pixmap)

        self.layout.addWidget(self.label)
        #self.layout.addWidget(self.pic)

        self.select_floor()

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

    def listRooms(self,value):
        self.rooms = ["Select"]
        rooms = backend.listByFloor(int(value))
        for i in rooms:
            self.rooms.append(i)
        self.combobox2.clear()
        self.combobox2.addItems(self.rooms)
        self.audio("Room options are ")
        sleep(2)

        for i in self.rooms:
            if i != "Select":
                self.audio(i)
                sleep(1)

    def destination(self,value):
        if (value != "Select"):
            text = "You have selected room " + value
            self.audio(text)


if __name__ == '__main__':
    app = QApplication(sys.argv)

    window = App()
    window.show()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        print('Closing Window...')
