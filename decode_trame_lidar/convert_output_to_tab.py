import re

# Lire le fichier d'entrée
with open('lidar_star_scan.txt', 'r') as infile:
    data = infile.read()

# Transformer le texte
transformed_data = re.sub(r'(?<=\b)[A-F0-9]{2}(?=,?)', r'0x\g<0>', data)

# Écrire le résultat dans un nouveau fichier
with open('lidar_star_scan.h', 'w') as outfile:
    outfile.write(transformed_data)