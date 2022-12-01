import sys, os
#import backend
import real_backend as backend
from time import sleep
from gtts import gTTS
# import pyttsx3
from pygame import mixer
from PyQt5.QtWidgets import QApplication, QWidget, QComboBox, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QToolBar, QToolButton, QMenuBar, QMenu, QAction, QMainWindow
from PyQt5.QtGui import *
from PyQt5.QtCore import QSize, Qt, QRect, QFile, QTextStream
from PyQt5 import QtTest

class App(QWidget):
    def __init__(self):
        super().__init__()

        self.setup()
        self.sound = True;
        self.audio("Welcome",'Welcome to Beamish Munro Hall. My name is George. I will guide you to your destination within the building. Press H if you would like to hear the help menu. Would you like to go to floor 1, 2, or 3?')
        self.rooms = ['Select']
        self.room_num = None;
        self.dark_mode = True

        self.select_floor()
        self.show()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_H:
            self.playHelpMenu()
        if event.key() == Qt.Key_B:
            self.changeMode()
        if event.key() == Qt.Key_M:
            self.muteSound()
        event.accept()

    def setup(self):
        self.setWindowTitle("Autonomous Building Guide")
        self.setFixedWidth(500)

        with open("dark/stylesheet.qss", "r") as f:
            content = f.readlines()
        text = ""
        for i in content:
            text += i
        self.setStyleSheet(text)

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.label = QLabel()

        self.pic = QLabel()
        pixmap = QPixmap(os.getcwd() + '/roomba.png')
        pixmap = pixmap.scaled(128, 128, Qt.KeepAspectRatio)
        self.pic.setPixmap(pixmap)

        self.backward = QAction()
        self.backward.setIcon(QIcon('icons/back_white.png'))
        self.backward.setEnabled(False)
        self.help = QAction("Help")
        self.help.triggered.connect(self.playHelpMenu)
        self.mute = QAction()
        self.mute.setIcon(QIcon('icons/unmute_white.png'))
        self.mute.triggered.connect(self.muteSound)
        self.mode = QAction()
        self.mode.setIcon(QIcon('icons/moon.png'))
        self.mode.triggered.connect(self.changeMode)

        self.menuBar = QMenuBar(self)
        self.menuBar.setNativeMenuBar(False)
        self.menuBar.addAction(self.backward)
        self.menuBar.addAction(self.mute)
        self.menuBar.addAction(self.mode)
        self.menuBar.addAction(self.help)

        self.layout.setMenuBar(self.menuBar)
        self.layout.addWidget(self.label)

    def audio(self,filename, message):
        if (self.sound):
            files = os.listdir('sound')
            filename += ".mp3"
            if (filename in files):
               mixer.init()
               mixer.music.load("sound/" + filename)
               mixer.music.play()
            else:
                language = 'en'
                myobj = gTTS(text=message, lang=language, slow=False)
                myobj.save("sound/" + filename)
                mixer.init()
                mixer.music.load("sound/" + filename)
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
        self.button.clicked.connect(self.directions)

    def listRooms(self,value):
        self.rooms = ["Select"]
        rooms = backend.listByFloor(value)
        for i in rooms:
            self.rooms.append(i)
        self.combobox2.clear()
        self.combobox2.addItems(self.rooms)
        self.audio("RoomOptions","Room options are ")
        QtTest.QTest.qWait(2000)

        for i in self.rooms:
            if i != "Select":
                self.audio(i,i)
                QtTest.QTest.qWait(1000)

    # Confirm destination
    def destination(self,value):
        if (value != "Select" and value != " "):
            text = "You have selected room " + value + ". Press continue if that is correct."
            self.audio("Select" + value, text)
            self.room_num = value
            self.button.setEnabled(True)
        if (value == "Select"):
            self.button.setEnabled(False)

    # List Obstacles in Path
    def directions(self):
        obstacles = backend.getObstackeList();

        # Clear Screen
        self.backward.setEnabled(True)
        self.clearWidget(self.combobox1)
        self.clearWidget(self.combobox2)
        self.clearWidget(self.l1)
        self.clearWidget(self.l2)
        self.clearWidget(self.button)
        self.clearWidget(self.pic)

        self.label.setText("Features in the path are:")
        self.audio("Features","Features in the path are.")
        QtTest.QTest.qWait(2000)
        self.temp = QLabel()
        self.layout.addWidget(self.temp)

        if obstacles:
            for i in obstacles:
                self.temp.setText(i)
                self.audio(i,i)
                QtTest.QTest.qWait(2000)

        else:
            self.audio("None","None.")

        self.audio("Continue","Do you wish to continue?")

        self.button2 = QPushButton("Continue")
        self.layout.addWidget(self.button2)
        self.button2.clicked.connect(self.navigating)

    # Screen when navigating to destination
    def navigating(self):
        self.label.setText("Going to room " + self.room_num)
        backend.sendRoom(self.room_num)

        # Clear screen
        self.clearWidget(self.temp)
        self.clearWidget(self.button2)

        # Robot should begin moving here
        self.audio("Going" + self.room_num, "Going to room " + self.room_num)
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

    def muteSound(self):
        if (self.sound):
            if (self.dark_mode): self.mute.setIcon(QIcon('icons/mute_white.png'))
            else: self.mute.setIcon(QIcon('icons/mute_black.png'))
            self.audio("Soundoff","Sound off")
            QtTest.QTest.qWait(1000)
            self.sound = False
        else:
            if (self.dark_mode): self.mute.setIcon(QIcon('icons/unmute_white.png'))
            else: self.mute.setIcon(QIcon('icons/unmute_black.png'))
            self.sound = True
            self.audio("Soundon","Sound on")
            QtTest.QTest.qWait(1000)

    def goBack(self):
        self.clearWidget(self.backward)
        self.clearWidget(self.mute)
        self.clearWidget(self.help)
        #self.clearLayout(self.layout)
        #self.layout.deleteLater()
        #self.layout = None
        self.setup()

    def changeMode(self):
        if (self.dark_mode):
            self.dark_mode = False
            self.mode.setIcon(QIcon('icons/sun.png'))
            self.backward.setIcon(QIcon('icons/back_black.png'))
            if (self.sound): self.mute.setIcon(QIcon('icons/unmute_black.png'))
            else: self.mute.setIcon(QIcon('icons/mute_black.png'))
            with open("light/stylesheet.qss", "r") as f:
                content = f.readlines()
            text = ""
            for i in content:
                text += i
            self.setStyleSheet(text)

        else:
            self.dark_mode = True
            self.mode.setIcon(QIcon('icons/moon.png'))
            self.backward.setIcon(QIcon('icons/back_white.png'))
            if (self.sound): self.mute.setIcon(QIcon('icons/unmute_white.png'))
            else: self.mute.setIcon(QIcon('icons/mute_white.png'))
            with open("dark/stylesheet.qss", "r") as f:
                content = f.readlines()
            text = ""
            for i in content:
                text += i
            self.setStyleSheet(text)

    def playHelpMenu(self):
        text = "Help Menu. M to mute/unmute. B to change to light/dark mode."
        self.audio("Help",text)

if __name__ == '__main__':
    app = QApplication(sys.argv)

    window = App()
    window.show()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        print('Closing Window...')
