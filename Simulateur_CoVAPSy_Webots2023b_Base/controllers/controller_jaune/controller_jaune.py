# Copyright 1996-2022 Cyberbotics Ltd.
#
# Controle de la voiture TT-02 simulateur CoVAPSy pour Webots 2023b
# Inspiré de vehicle_driver_altino controller
# Kévin Hoarau, Anthony Juton, Bastien Lhopitallier, Martin Raynaud
# août 2023

from vehicle import Driver
from controller import Lidar
import numpy as np
import time

#######################

# Paramètres de configuration
target_distance = 1.0  # Distance cible en mètres du mur
kp = 0.5               # Coefficient proportionnel
ki = 0.1               # Coefficient intégral
kd = 0.05              # Coefficient dérivé

# Variables du PID
previous_error = 0.0
integral = 0.0

def get_distance_to_wall(lidar_data, angle_range=(-45, 45)):
    """
    Mesure la distance moyenne entre la voiture et le mur dans une plage angulaire donnée.
    
    :param lidar_data: liste des distances mesurées par le lidar en fonction des angles.
    :param angle_range: tuple (min_angle, max_angle) définissant la plage d'angle en degrés.
    :return: distance moyenne au mur dans la plage spécifiée.
    """
    # Conversion des angles en indices (supposons 360 points lidar pour 360°)
    min_angle, max_angle = angle_range
    min_index = int((min_angle + 180) % 360)
    max_index = int((max_angle + 180) % 360)
    
    # Extraire les distances dans la plage d'angles
    relevant_distances = lidar_data[min_index:max_index+1]
    average_distance = np.mean(relevant_distances)
    
    return average_distance

def pid_control(error, dt):
    """
    Contrôle PID pour ajuster la direction en fonction de l'erreur de distance.
    
    :param error: écart actuel par rapport à la distance cible.
    :param dt: intervalle de temps depuis la dernière mesure.
    :return: correction d'angle pour la direction.
    """
    global previous_error, integral
    
    # Calcul des termes PID
    integral += error * dt
    derivative = (error - previous_error) / dt
    output = kp * error + ki * integral + kd * derivative
    
    # Mettre à jour l'erreur précédente
    previous_error = error
    
    return output

def wall_following_control(lidar_data, target_distance, dt):
    """
    Fonction principale pour suivre le mur en ajustant la direction.
    
    :param lidar_data: liste des distances mesurées par le lidar.
    :param target_distance: distance cible au mur en mètres.
    :param dt: intervalle de temps entre les mesures.
    :return: ajustement d'angle pour la direction.
    """
    # Obtenir la distance actuelle au mur
    current_distance = get_distance_to_wall(lidar_data)
    
    # Calculer l'erreur
    error = target_distance - current_distance
    
    # Calcul de la correction d'angle avec le PID
    correction = pid_control(error, dt)
    
    return correction



########################










driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

#Lidar
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud() 

#claviera
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

tableau_lidar_mm=[0]*360

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

while driver.step() != -1:
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
   
          # Intervalle de temps (dt)
        dt = 0.1  # En secondes (ajuster selon la fréquence des mesures)
        
        # Obtenir la correction de direction
        angle_correction = wall_following_control(tableau_lidar_mm, target_distance, dt)
        
        # Appliquer l'angle de correction pour ajuster la direction du véhicule
        #print(f"Correction de direction: {angle_correction:.2f}")
   
    #######################################################
   
        #un secteur par tranche de 20° donc 10 secteurs numérotés de 0 à 9    
        #angle_degre = 0.02*(tableau_lidar_mm[60]-tableau_lidar_mm[-60])
        set_direction_degre(angle_correction)
        vitesse_m_s = 0.5
        set_vitesse_m_s(vitesse_m_s)
 
    #########################################################

