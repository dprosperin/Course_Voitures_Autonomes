# Copyright 1996-2022 Cyberbotics Ltd.
#
# Controle de la voiture TT-02 simulateur CoVAPSy pour Webots 2023b
# Inspiré de vehicle_driver_altino controller
# Kévin Hoarau, Anthony Juton, Bastien Lhopitallier, Martin Raynaud
# août 2023

import matplotlib.pyplot as plt
from vehicle import Driver
from controller import Lidar
import numpy as np
import time
from pprint import pprint


driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# Données pour le graphique
times = []
values = []

# Paramètres du graphique
plt.ion()  # Mode interactif pour mettre à jour le graphique en temps réel
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
line, = ax1.plot(times, values, label="Donnée simulée")
ax1.set_title("Graphique en temps réel avec Webots")
ax1.set_xlabel("Temps (s)")
ax1.set_ylabel("Valeur")
ax1.grid(True)
ax1.legend()

ax2 = fig.add_subplot(122, projection='polar')
tableau_lidar_mm=[0]*360
theta = np.linspace(0, 2 * np.pi, 360)
ax2.plot(theta, tableau_lidar_mm, label="Lidar")
ax2.set_title("Projection Polaire")
ax2.grid(True)
ax2.legend()

plt.ylim(-1000, 1000)  # Min = 0, Max = 40

#Lidar
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud() 

#clavier
keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

# vitesse en km/h
speed = 0
maxSpeed = 28 #km/h

# angle de la direction
angle = 0
maxangle_degre = 16


# mise a zéro de la vitesse et de la direction
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)

tab_discontinuite = [[0, 0] for _ in range(100)]

def discontinuite(data_lidar_mm_main):
    seuil_discontinuite = 150  # Seuil pour considérer une discontinuité
    cpt = 0
    
   

    for i in range(1, 359):
        if 0 <= i <= 90 or 270 <= i <= 359:
            distance_courante = data_lidar_mm_main[i]
            distance_suivante = data_lidar_mm_main[i + 1]
            
            diff = abs(distance_suivante - distance_courante)
            
            if diff >= seuil_discontinuite:
                tab_discontinuite[cpt][0] = diff
                tab_discontinuite[cpt][1] = i
                #print(f"{tab_discontinuite[cpt][0]},{tab_discontinuite[cpt][1]}")
                cpt += 1
                

def set_vitesse_m_s(vitesse_m_s):
    speed = vitesse_m_s*3.6
    if speed > maxSpeed :
        speed = maxSpeed
    if speed < 0 :
        speed = 0
    driver.setCruisingSpeed(speed)
     
def set_direction_degre(angle_degre):
    if angle_degre > maxangle_degre:
        angle_degre = maxangle_degre
    elif angle_degre < -maxangle_degre:
        angle_degre = -maxangle_degre   
    angle = -angle_degre * 3.14/180
    driver.setSteeringAngle(angle)

def recule(): #sur la voiture réelle, il y a un stop puis un recul pendant 1s.
    driver.setCruisingSpeed(-1)  

# mode auto desactive
modeAuto = False
print("cliquer sur la vue 3D pour commencer")
print("a pour mode auto (pas de mode manuel sur TT02_jaune), n pour stop")

start_time = time.time()
while driver.step() != -1:
    # Récupérer le temps écoulé
    current_time = time.time() - start_time
    while True:
    #acquisition des donnees du lidar
         # recuperation de la touche clavier
        currentKey = keyboard.getKey()
 
        if currentKey == -1:
            break
       
        elif currentKey == ord('n') or currentKey == ord('N'):
            if modeAuto :
                modeAuto = False
                print("--------Modes Auto TT-02 jaune Désactivé-------")
        elif currentKey == ord('a') or currentKey == ord('A'):
            if not modeAuto : 
                modeAuto = True
                print("------------Mode Auto TT-02 jaune Activé-----------------")
    
    #acquisition des donnees du lidar
    donnees_lidar_brutes = lidar.getRangeImage()
    for i in range(360) :
        if (donnees_lidar_brutes[-i]>0) and (donnees_lidar_brutes[-i]<20) :
            tableau_lidar_mm[i] = 1000*donnees_lidar_brutes[-i]
        else :
            tableau_lidar_mm[i] = 0
   
    if not modeAuto:
        set_direction_degre(0)
        set_vitesse_m_s(0)
        
    if modeAuto:
    ########################################################
    # Programme etudiant avec
    #    - le tableau tableau_lidar_mm
    #    - la fonction set_direction_degre(...)
    #    - la fonction set_vitesse_m_s(...)
    #    - la fonction recule()
    #######################################################
   
        #un secteur par tranche de 20° donc 10 secteurs numérotés de 0 à 9    
        vitesse_m_s = 0
        set_vitesse_m_s(vitesse_m_s)
        value = tableau_lidar_mm[0]   # Par exemple, le temps courant

        
        discontinuite(tableau_lidar_mm)
        pprint(tab_discontinuite)
        tab_discontinuite = [[0, 0] for _ in range(100)]

        # Mettre à jour les données
        times.append(current_time)
        values.append(value)
        
         # Limiter le nombre de points affichés
        if len(times) > 50:  # Garder seulement 50 points
            times.pop(0)
            values.pop(0)
            ax2.clear()

        if len(times) > 3:  # Garder seulement 3 points
            ax2.clear()

        line2 = ax2.scatter(theta, tableau_lidar_mm, s=2)
        line2.set_array(tableau_lidar_mm)

        # Mettre à jour le graphique
        line.set_xdata(times)
        line.set_ydata(values)
        ax1.relim()  # Réajuster les limites
        ax1.autoscale_view()
        plt.pause(0.01)  # Pause pour afficher le graphique
 
    #########################################################

# Fermer proprement le graphique à la fin
plt.ioff()
plt.tight_layout()
plt.show()