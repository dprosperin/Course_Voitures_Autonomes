import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from parse import parse
import serial.tools.list_ports
from sys import exit

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
## Via un fichier
filename = "./lidar.txt"
mode_ouverture = "r"
## Via la liaison série
port = "/dev/cu.usbmodem103"
baudrate = 1000000
timeout = 1
bytesize = 8
parity = "N"
stopbits = 1
##### Config #####

print("""
Programme : Plotter polaire pour tester le LIDAR via liaison série ou un fichier de sortie
Auteur : David PROSPÉRIN

1 : Pour se connecter à la liaison série avec les paramètres suivant :
port     = %s
baudrate = %d
timeout  = %d
bytesize = %d
parity   = %s
stopbits = %s

2 : Pour ouvrir le fichier nommé : 
filename = %s
Mode d'ouverture = %s
""" % (port, baudrate, timeout, bytesize, parity, stopbits, filename, mode_ouverture))

mode = int(input(">>> "))

while mode != 1 and mode != 2:
    print("%d n'est pas un mode valide" % mode)
    mode = int(input(">>> "))

def lister_ports():
    print("=== Liste des ports disponible ===\n")
    ports = serial.tools.list_ports.comports()

    for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))
    print("\n==================================")

if mode == 1:
    try:
        liaison_serie = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, bytesize = bytesize, parity=parity, stopbits=stopbits)

        liaison_serie.close() #Ferme le port s'il est déjà ouvert
        liaison_serie.open() #On ouvre le port

        lister_ports()

        if liaison_serie.isOpen():
            print(liaison_serie.name + " is open…")
            print(liaison_serie.get_settings()) #Grace a ces 3 lignes lorsque le Port est ouvert c’est indiqué dans le LOG 
    except:
        print("Impossible d'ouvrir le port : " + port)
        lister_ports()
        exit()
elif mode == 2:
    #Ouverture fichier
    print("Ouverture du fichier %s" % filename)
    try:
        output_file = open(filename, mode_ouverture)
        print("Fichier %s ouvert avec succès" % filename)
    
    except:
        print("Impossible d'ouvrir le fichier %s en mode %s" % (filename, mode_ouverture))
        exit()

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

    if parse("({:f},{:f})\n", chaine_recu) != None :
        # Extraction (angle °, distance mm)
        angle, distance = parse("({:f},{:f})\n", chaine_recu)
    
        print("angle : {} distance : {}".format(angle, distance))

        tableau_lidar_mm.append(distance)
        teta.append(angle * np.pi / 180)
    else:
        print("Impossible de parser : " + chaine_recu)

    ax.clear()  # clearing the axes

    line = ax.scatter(teta, tableau_lidar_mm, s=2)

    line.set_array(tableau_lidar_mm)

    ax.set_rmax(1000)

    ax.grid(True)

if mode == 1:
    anim = FuncAnimation(fig=fig, func=update, frames=1000, interval=1)
elif mode == 2:
    while True:
        ligne = output_file.readline()

        if parse("({:f},{:f})\n", ligne) != None :
            # Extraction (angle °, distance mm)
            angle, distance = parse("({:f},{:f})\n", ligne)
    
            #print("angle : {} distane : {}".format(angle, distance))

            tableau_lidar_mm.append(distance)
            teta.append(angle * np.pi / 180)
        else:
            print("Impossible de parser : " + ligne)

        if ligne == "":
            break

    line = ax.scatter(teta, tableau_lidar_mm, s=2)

    line.set_array(tableau_lidar_mm)

    ax.set_rmax(800)

    ax.grid(True)

plt.show()