import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from parse import parse
import serial.tools.list_ports

"""
python3 -m venv .
source bin/activate

Installer un paquet
python3 -m pip install <package>

Générer le fichier requirements.txt
pip3 freeze > requirements.txt

Lister les ports série
python3 -m serial.tools.list_ports
"""

##### Config #####
port = "/dev/cu.usbmodem1203"
baudrate = 1000000
timeout = 1
bytesize = 8
parity = "N"
stopbits = 1
##### Config #####

def lister_ports():
    print("=== Liste des ports disponible ===\n")
    ports = serial.tools.list_ports.comports()

    for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))
    print("\n==================================")

try:
    liaison_serie = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, bytesize = bytesize, parity=parity, stopbits=stopbits)

    liaison_serie.close() #Ferme le port s'il est déjà ouvert
    liaison_serie.open() #On ouvre le port

    lister_ports()
except:
    print("Impossible d'ouvrir le port : " + port)
    lister_ports()

if liaison_serie.isOpen():
    print(liaison_serie.name + " is open…")
    print(liaison_serie.get_settings()) #Grace a ces 3 lignes lorsque le Port est ouvert c’est indiqué dans le LOG 

def lidar_start_scan(my_serial: serial.Serial):
    if my_serial.isOpen() :
        my_serial.write("START_SCAN\n")

def lidar_stop(my_serial: serial.Serial):
    if my_serial.isOpen() :
        my_serial.write("STOP\n")

def lidar_reset(my_serial: serial.Serial):
    if my_serial.isOpen() :
        my_serial.write("RESET\n")

###################################################
#affichage des données acquises sur l'environnement
#pour les tests
###################################################

# Diastance en mm
tableau_lidar_mm = []

# Angle en °
teta = []

fig = plt.figure()

ax = plt.subplot(111, projection='polar')

# update function to update data and plot
def update(frame):
    angle = 0
    distance = 0

    chaine_recu = liaison_serie.readline().decode("ascii", errors="ignore")

    print("Chaine reçu : " + chaine_recu)

    if parse("({:d},{:d})\n", chaine_recu) != None :
        # Extraction (angle °, distance mm)
        angle, distance = parse("({:d},{:d})\n", chaine_recu)
    
        print("angle : {} distane : {}".format(angle, distance))

        tableau_lidar_mm.append(distance)
        teta.append(angle * np.pi / 180)
    else:
        print("Impossible de parser : " + chaine_recu)

    ax.clear()  # clearing the axes

    line = ax.scatter(teta, tableau_lidar_mm, s=2)

    line.set_array(tableau_lidar_mm)

    ax.set_rmax(8000)

    ax.grid(True)

anim = FuncAnimation(fig=fig, func=update, frames=1000, interval=1)

plt.show()