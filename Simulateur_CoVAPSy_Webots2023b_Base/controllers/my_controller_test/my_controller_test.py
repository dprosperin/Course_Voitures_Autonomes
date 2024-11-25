import numpy as np
import time

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

# Exemple d'utilisation dans une boucle de contrôle
try:
    while True:
        # Simulation des données LIDAR (remplacer par les vraies données du lidar)
        lidar_data = np.random.normal(1.0, 0.1, 360)  # Exemple de données simulées
        
        # Intervalle de temps (dt)
        dt = 0.1  # En secondes (ajuster selon la fréquence des mesures)
        
        # Obtenir la correction de direction
        angle_correction = wall_following_control(lidar_data, target_distance, dt)
        
        # Appliquer l'angle de correction pour ajuster la direction du véhicule
        print(f"Correction de direction: {angle_correction:.2f}")
        
        # Pause pour simuler l'intervalle de temps
        time.sleep(dt)

except KeyboardInterrupt:
    print("Arrêt du programme.")