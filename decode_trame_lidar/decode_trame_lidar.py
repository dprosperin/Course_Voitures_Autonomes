print("""
Programme : Programme simple qui décode une trame du LIDAR
Auteur : David PROSPERIN
""")

print("Veuillez entrer les cinq octets de la trame")
byte1_input = input("Byte1 >>>")

# Convertir l'entrée en entier (base 16)
try:
    byte1 = int(byte1_input, 16)
    print(f"Byte1 vaut : {byte1}")
except ValueError:
    print("Entrée invalide ! Assurez-vous que c'est un nombre hexadécimal valide.")

byte2_input = input("Byte2 >>>")

# Convertir l'entrée en entier (base 16)
try:
    byte2 = int(byte2_input, 16)
    print(f"Byte2 vaut : {byte2}")
except ValueError:
    print("Entrée invalide ! Assurez-vous que c'est un nombre hexadécimal valide.")

byte3_input = input("Byte3 >>>")

# Convertir l'entrée en entier (base 16)
try:
    byte3 = int(byte3_input, 16)
    print(f"Byte3 vaut : {byte3}")
except ValueError:
    print("Entrée invalide ! Assurez-vous que c'est un nombre hexadécimal valide.")

byte4_input = input("Byte4 >>>")

# Convertir l'entrée en entier (base 16)
try:
    byte4 = int(byte4_input, 16)
    print(f"Byte4 vaut : {byte4}")
except ValueError:
    print("Entrée invalide ! Assurez-vous que c'est un nombre hexadécimal valide.")

byte5_input = input("Byte5 >>>")

# Convertir l'entrée en entier (base 16)
try:
    byte5 = int(byte5_input, 16)
    print(f"Byte5 vaut : {byte5}")
except ValueError:
    print("Entrée invalide ! Assurez-vous que c'est un nombre hexadécimal valide.")

s = byte1 & 1
not_s = 1 if (byte1 & 2)  else 0
quality = byte1 >> 2
c = byte2 & 1
angle = (((byte3 << 7) & 0xFF00) | byte2) / 64.0
distance = ((byte5 << 8) | byte4) / 4.0

print("""
=== Resultat ===
S = %d      
/S = %d
Quality = %d
C = %d
Angle = %f
Distance = %f
""" % (s, not_s, quality, c, angle, distance))