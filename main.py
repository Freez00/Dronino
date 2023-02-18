import kivy                         # imports regarding the GUI (kivy.*)
from kivy.app import App
from kivy.lang import Builder
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.uix.textinput import TextInput    
from kivy.uix.button import Button
from kivy.uix.widget import Widget
from kivy.properties import ObjectProperty
from kivy.uix.slider import Slider
from kivy.uix.floatlayout import FloatLayout
from joystick.joystick import Joystick
from kivy.clock import Clock
from kivy.uix.screenmanager import ScreenManager, Screen
import kivy.config
from kivy.config import Config
import socket                       # socket import to establish connecction and send udp packages

UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)      # start the udp client socket

try:                                # attempt to find the local network (LAN) IP by using a mask
    UDPClientSocket.connect(('192.255.255.255', 1))
    droneIP = UDPClientSocket.getsockname()[0]      # set the IP variable
except:
    droneIP = '127.0.0.1'                    # if we can't find the IP of the network => set it to the most common IP

droneIP = droneIP[:-1] + '1'            # replace the last digit of the IP with 1 since its a constant for the arduino  

serverAddressPort = (droneIP, 2390)         # set the address that we are going to send the Udp packages to

bufferSize = 1024               # set variables for the network
Ssid = ""
Password = "" 

class MainMenu(Screen):             # create the MainMenu class for our app (MAIN CONTROL SCREEN)

    rotateLeftID = ObjectProperty(None)                 # Get the required elements from the screen by their id (buttons(2), slider, switch, joystick {accordingly})
    rotateRightID = ObjectProperty(None)                # button for rotating the drone to the right
    sliderID = ObjectProperty(None)                     # power controlling slider element
    switchID = ObjectProperty(None)                     # ON/OFF switch
    directionsJoystickID = ObjectProperty(None)         # joystick for controlling the mmovement of the drone

    def __init__(self, **kwargs):                           # function runs when the class is created
        super(MainMenu, self).__init__(**kwargs)
        Clock.schedule_interval(self.send_packaged_data, 0.001)                     # activate a clock to run send_packaged_data() every 0.0001 seconds
        self.initial_loop = Clock.schedule_interval(self.initial_disable, 0.1)      # call a function to attempt disabling the buttons
    
    def initial_disable(self, *args):                                      # a function to run when entering the screen to disable all the buttons and other elements
        try:
            self.directionsJoystickID.disabled = True               # disable all the elements
            self.rotateLeftID.disabled = True
            self.rotateRightID.disabled = True
            self.sliderID.disabled = True
            self.initial_loop.cancel()                             # cancel the clock schedule
        except: pass
    
    def switch_input(self, widget):                  # a function that enables or disables the buttons and other elements depending on the status of the ON/OFF switch
        if(widget.active == False):
            self.directionsJoystickID.disabled = True
            self.rotateLeftID.disabled = True
            self.rotateRightID.disabled = True
            self.sliderID.disabled = True
            self.sliderID.value = 1000;
        else:
            encoded = str.encode("S")
            UDPClientSocket.sendto(encoded, serverAddressPort) 
            self.directionsJoystickID.disabled = False
            self.rotateLeftID.disabled = False
            self.rotateRightID.disabled = False
            self.sliderID.disabled = False
        

    def send_packaged_data(self, *args):                    # function that sends encoded packages to the arduino with the movement information
        if self.switchID.active == False: return      # don't send anything if the switch is OFF

        package = str.encode("X")                 # create the encoded message with the appropriate values
        package += int.to_bytes(int(self.directionsJoystickID.pad[0]*1000/2 + 1500), 2, 'little')
        package += str.encode("Y")
        package += int.to_bytes(int(self.directionsJoystickID.pad[1]*-1000/2 + 1500), 2, 'little')
        if self.rotateLeftID.state == "normal":
            package+= "LF".encode()
        elif self.rotateLeftID.state == "down":
            package+= "LT".encode()
        if self.rotateRightID.state == "normal":
            package+= "RF".encode()
        elif self.rotateRightID.state == "down":
            package+= "RT".encode()
        package+= "P".encode()
        package += int.to_bytes(int(self.sliderID.value), 2, 'little')
        UDPClientSocket.sendto(package, serverAddressPort)          # send the message as an Udp package to the arduino 

class WifiOptionsMenu(Screen):              # class WifiOptionsMenu which is the first screen when the app is opened (Allows the user to change the configuration of the arduino's hotspot)

    def change_configuration(self, ssid, password, app):        # function that is called when the 'Change' button is pressed (Sends encoded  
                                                                # data to the arduino containing the new configuration settings for the hotspot)

        if len(ssid) >= 8 and len(password) >= 8:                # require the ssid(network name) and password to have at least 8 characters as it is required for a hotspot
            app.root.current = "control"                        # change the screen to the main screen
            Ssid = ssid
            Password = password
            encoded_package = str.encode("N")                       # indificator for the new config
            encoded_package += int.to_bytes(len(Ssid), 1, 'little')     # create the encoded message
            encoded_package += Ssid.encode()
            encoded_package += int.to_bytes(len(Password), 1, 'little')
            encoded_package += Password.encode()
            UDPClientSocket.sendto(encoded_package, serverAddressPort)      # send the encoded message for the new config of the hotspot

class ScreenManagement(ScreenManager):              # create a screen manager class to take care of different screens and transitions between them
    pass

kv = Builder.load_string("""ScreenManagement:
    canvas.before:
        Color:
            rgba: 27/255, 28/255, 40/255, 1
        Rectangle:
            size: self.size
            pos: self.pos
    id: screen_manager
    WifiOptionsMenu:
    MainMenu:

<RoundedButton@Button>:
    background_color: (0,0,0,0)
    background_normal: ''
    canvas.before:
        Color:
            rgba: (63/255, 127/255, 191/255, 1) if self.state == "normal" and not self.disabled else (49/255, 101/255, 140/255, 1)
        RoundedRectangle:
            size: self.size
            pos: self.pos
            radius: [29]

<Purpl@Button>:
    background_color: (0,0,0,0)
    background_normal: ''
    canvas.before:
        Color:
            rgba: (126/255, 98/255, 179/255, 1) if self.state == "normal" and not self.disabled else (94/255, 70/255, 140/255, 1)
        RoundedRectangle:
            size: self.size
            pos: self.pos
            radius: [29]



<Joystick>:
    sticky: False
    outer_size: 1
    inner_size: 0.75
    pad_size:   0.25
    outer_line_width: 0.01
    inner_line_width: 0.01
    pad_line_width:   0.01
    outer_background_color: (158/255, 158/255, 203/255, 0.4)
    outer_line_color:       (1, 1, 1, 1)
    inner_background_color: (158/255, 158/255, 203/255, 0.4)
    inner_line_color:       (0,  0,  0,  0)
    pad_background_color:   (191/255, 77/255, 128/255,  1)
    pad_line_color:         (218/255, 87/255, 146/255, 1)
    size_hint: 1, 1
    pos_hint: {"center_x":0.6, "center_y":0.25}


<WifiOptionsMenu>:
    name: "wificonfig"
    canvas.before:
        Color:
            rgba: 27/255, 28/255, 40/255, 1
        Rectangle:
            pos: self.pos
            size: self.size

    FloatLayout:
        size: root.width, root.height

        Label:
            text: "SSID:"
            text_size: self.size
            halign: 'right'
            valign: 'middle'
            size_hint: 0.3, 0.3
            font_size: 32
            pos_hint: {"center_x":0.17, "center_y":0.7}

        TextInput:
            id: inputSSID_ID
            size_hint: 0.4, 0.07
            pos_hint: {"center_x":0.6, "center_y":0.7}
            multiline: False
            font_size: 32
            background_color: (220/255, 220/255, 220/255, 1)

        Label:
            text: "Password:"
            text_size: self.size
            halign: 'right'
            valign: 'middle'
            size_hint: 0.3, 0.3
            font_size: 32
            pos_hint: {"center_x":0.17, "center_y":0.5}

        TextInput:
            id: inputPassword_ID
            size_hint: 0.4, 0.07
            pos_hint: {"center_x":0.6, "center_y":0.5}
            multiline: False
            font_size: 32
            background_color: (220/255, 220/255, 220/255, 1)

        RoundedButton:
            text:"Change"
            font_size: 30
            size_hint: 0.4, 0.15
            pos_hint: {"center_x":0.35, "center_y":0.2}
            on_release: root.change_configuration(inputSSID_ID.text, inputPassword_ID.text, app)

        RoundedButton:
            text:"Skip"
            background_normal: "normal.png"
            font_size: 30
            size_hint: 0.2, 0.15
            pos_hint: {"center_x":0.75, "center_y":0.2}
            on_release: app.root.current = "control"

<MainMenu>:
    name: "control"
    canvas.before:
        Color:
            rgba: 27/255, 28/255, 40/255, 1
        Rectangle:
            pos: self.pos
            size: self.size
    directionsJoystickID:directionsJoystickID
    rotateLeftID:rotateLeftID
    rotateRightID:rotateRightID
    sliderID:sliderID
    switchID:switchID

    FloatLayout:
        size: root.width, root.height
        BoxLayout:
            size_hint: 0.4, 0.4
            pos_hint: {"x":0.05, "center_y": 0.65}

            Joystick:
                id: directionsJoystickID

        BoxLayout:
            orientation: "horizontal"
            spacing: 40
            pos_hint: {"center_x": 0.5, "y":0.1}
            size_hint: 0.4, 0.1

            Purpl:
                id: rotateLeftID
                text:"Rotate Left"
                size_hint: 3, 1
                font_size: 20

            Purpl:
                id: rotateRightID
                text:"Rotate Right"
                size_hint: 3, 1
                font_size: 20
        
        BoxLayout:
            orientation: "vertical"
            spacing: 10
            pos_hint: {"right": 0.9, "center_y":0.5}
            size_hint: 0.1, 1
            Label:
                text: "Power"
                color: 1,1,1 , 1
                font_size: 44
                pos_hint: {"right": 1, "top": 1}
                size_hint: 1, 1

            Slider:
                id: sliderID
                min: 1000
                max: 2000
                color: 0.5, 0, 0.7, 1
                step: 1

                value_track: True   
                value_track_color: (191/255, 77/255, 128/255,  1)
                value_track_width: 10
                
                cursor_image: ""
                cursor_disabled_image: ""
                cursor_size: (30, 20)

                orientation: "vertical"
                pos_hint: {"right": 1, "center_y": 1}
                size_hint: 1, 2.5

            Switch:
                id: switchID
                pos_hint: {"right": 1, "top": 0.7}
                size_hint: 1, 0.7
                on_active: root.switch_input(self)""")

class DroneApp(App):        # Main class of the app
    def build(self):        
        try:
            self.icon = "icon.png"     # Set the icon of the app
        except:pass
        return kv
        #return super().build()  # run the build of the super(App) class

if __name__ == "__main__":
    DroneApp().run()            # run the app