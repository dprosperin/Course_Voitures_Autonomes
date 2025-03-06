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
#plt.ion()  # Mode interactif pour mettre à jour le graphique en temps réel
#fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
#line, = ax1.plot(times, values, label="Donnée simulée")
#ax1.set_title("Graphique en temps réel avec Webots")
#ax1.set_xlabel("Temps (s)")
#ax1.set_ylabel("Valeur")
#ax1.grid(True)
#ax1.legend()

#ax2 = fig.add_subplot(122, projection='polar')
tableau_lidar_mm=[0]*360
#theta = np.linspace(0, 2 * np.pi, 360)
#ax2.plot(theta, tableau_lidar_mm, label="Lidar")
#ax2.set_title("Projection Polaire")
#ax2.grid(True)
#ax2.legend()

#plt.ylim(-1000, 1000)  # Min = 0, Max = 40

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

vitesse_moyenne = 1
#kp_vitesse = 0.0002
kp_vitesse = 0
kp_rotation = 3.5

# angle de la direction
angle = 0
maxangle_degre = 16

tableau_min_locaux = [[0, 0] for _ in range(100)]
tableau_max_locaux = [[0, 0] for _ in range(100)]

etat = "INIT"
cpt_min_locaux = 0

# mise a zéro de la vitesse et de la direction
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)

tab_discontinuite = [[0, 0] for _ in range(100)]
nombre_discontinuite = 0

def get_angle_cap(angle_mapped):
    if angle_mapped >= 0 and angle_mapped <= 180:
        return map(angle_mapped, 0, 180, 16, -16)
    else:
        return 0

def map(x, in_min, in_max, out_min, out_max):
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def get_mapped_angle(angle_unmapped):
    if angle_unmapped >= 0 and angle_unmapped <= 90:
        return 90 - angle_unmapped
    elif angle_unmapped >= 270 and angle <= 359:
        return 450 - angle_unmapped
    else:
        return angle_unmapped

def get_unmapped_angle(angle_mapped):
    if angle_mapped >= 0 and angle_mapped <= 90:
        return 90 - angle_mapped
    elif angle_mapped >= 91 and angle_mapped <= 180:
        return 450 - angle_mapped
    else:
        return 0

def discontinuite(data_lidar_mm_main):
    seuil_discontinuite = 150  # Seuil pour considérer une discontinuité
    global nombre_discontinuite

    nombre_discontinuite = 0

    for i in range(1, 359):
        if 0 <= i <= 90 or 270 <= i <= 359:
            distance_courante = data_lidar_mm_main[i]
            distance_suivante = data_lidar_mm_main[i + 1]
            
            diff = abs(distance_suivante - distance_courante)
            
            if diff >= seuil_discontinuite:
                tab_discontinuite[nombre_discontinuite][0] = diff
                tab_discontinuite[nombre_discontinuite][1] = get_mapped_angle(i)
                #print(f"{tab_discontinuite[cpt][0]},{tab_discontinuite[cpt][1]}")
                nombre_discontinuite += 1
 
def recherches_locaux(data_lidar_mm_main):
    distance_actuelle = 0
    distance_apres = 0
    global cpt_min_locaux 
    cpt_max_locaux = 0
    global etat
    
    # Parcours des angles du lidar (de 1 à 358 pour éviter de sortir du tableau avec i+1)
    for i in range(1, 359):
        # Filtrage des angles souhaités (ici 0-90° et 270-359°)
        if (0 <= i <= 90) or (270 <= i <= 359):
            distance_actuelle = data_lidar_mm_main[i]
            distance_apres = data_lidar_mm_main[i + 1]
            
            if etat == "INIT":
                if distance_actuelle < distance_apres:
                    etat = "AUGMENTER"
                elif distance_actuelle > distance_apres:
                    etat = "DIMINUER"
            
            elif etat == "AUGMENTER":
                if distance_actuelle < distance_apres:
                    etat = "AUGMENTER"
                else:
                    etat = "MAX_LOCAL"
            
            elif etat == "DIMINUER":
                if distance_actuelle > distance_apres:
                    etat = "DIMINUER"
                else:
                    etat = "MIN_LOCAL"
            
            elif etat == "MIN_LOCAL":
                #print(f"Minimum local (distance = {distance_actuelle} mm à l'angle {i}°)")
                tableau_min_locaux[cpt_min_locaux][0] = distance_actuelle
                tableau_min_locaux[cpt_min_locaux][1] = i
                cpt_min_locaux += 1
                etat = "INIT"
            
            elif etat == "MAX_LOCAL":
                tableau_max_locaux[cpt_max_locaux][0] = distance_actuelle
                tableau_max_locaux[cpt_max_locaux][1] = i
                cpt_max_locaux += 1
                #print(f"Maximum local (distance = {distance_actuelle} mm à l'angle {i}°)")
                etat = "INIT"  # Réinitialiser pour le prochain cycle

def cap_navigation():
    global tab_discontinuite
    global nombre_discontinuite
    
    if nombre_discontinuite == 0:
        return 0
      
    max1 = 0
    max2 = 0
    max3 = 0 
    angle_max1 = 0
    angle_max2 = 0
    angle_max3 = 0
    min = 0 
    min2 = 0
    
    if nombre_discontinuite > 1  : 
        for i in range(nombre_discontinuite):  # Itérer sur le nombre de discontinuités
          diff = tab_discontinuite[i][0]  # La distance (diff)
          angle = tab_discontinuite[i][1]  # L'angle correspondant
        if diff > max1:
            max2 = max1
            angle_max2 = angle_max1
            max1 = diff
            angle_max1 = angle  
        elif diff > max2:
            max2 = diff
            angle_max2 = angle  
        return (angle_max1 + angle_max2) / 2  # Retourner la moyenne des angles
    
    elif nombre_discontinuite == 1 : 
        for i in range(nombre_discontinuite):  # Itérer sur le nombre de discontinuités
          difff = tab_discontinuite[i][0]  
          angles = tab_discontinuite[i][1]  
          if difff > max1:
            max3 = diff
            angle_max3 = angles
        else : 
            max3 = max3
            angle_max3=angle_max3
    for i in range(100):
        min = tableau_min_locaux[cpt_min_locaux][0] 
        if min > min2 :
            min = min 
        elif min2 > min : 
            min = min2
    return (angle_max3+min)/2

def clean_tab_discontinuite():
    global tab_discontinuite
    tab_discontinuite = [[0, 0] for _ in range(100)]

def clean_tableau_min_locaux():
    global tableau_min_locaux
    tableau_min_locaux = [[0, 0] for _ in range(100)]

def clean_tableau_max_locaux():
    global tableau_max_locaux
    tableau_max_locaux = [[0, 0] for _ in range(100)]

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
    
        vitesse_m_s = vitesse_moyenne + tableau_lidar_mm[i] * kp_vitesse
        set_vitesse_m_s(vitesse_m_s)
    
        discontinuite(tableau_lidar_mm)
        #pprint(tab_discontinuite)
        value = get_unmapped_angle(cap_navigation()) #Valeur à observer au cours du temps
        #value = cap_navigation()
        print("Angle navigation mapped : %d Angle navigation unmapped %d Angle cap reelle %d nb disc %d" %  (cap_navigation(), value, get_angle_cap(cap_navigation()),  nombre_discontinuite))

        set_direction_degre(kp_rotation * get_angle_cap(cap_navigation()))
        clean_tab_discontinuite()

        #recherches_locaux(tableau_lidar_mm)
        #pprint(tableau_min_locaux)
        #pprint(tableau_max_locaux)
        clean_tableau_min_locaux()
        clean_tableau_max_locaux()

        # Mettre à jour les données
        #times.append(current_time)
        #values.append(value)
        
         # Limiter le nombre de points affichés
        #if len(times) > 50:  # Garder seulement 50 points
         #   times.pop(0)
         #   values.pop(0)
        #    ax2.clear()

        #if len(times) > 3:  # Garder seulement 3 points
        #    ax2.clear()

        #line2 = ax2.scatter(theta, tableau_lidar_mm, s=2)
        #line2.set_array(tableau_lidar_mm)

        # Mettre à jour le graphique
        #line.set_xdata(times)
        #line.set_ydata(values)
        #ax1.relim()  # Réajuster les limites
        #ax1.autoscale_view()
        #plt.pause(0.01)  # Pause pour afficher le graphique
 
    #########################################################

# Fermer proprement le graphique à la fin
plt.ioff()
plt.tight_layout()
plt.show()