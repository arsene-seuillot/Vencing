import csv
import threading
import time
import bpy
import numpy as np
import math as m


#------------------------------------------------------------------------------------#
#---------------------------------FONCTIONS UTILITAIRES------------------------------#


# Variable globale pour contrôler l'arrêt de la rotation
stop_rotation = False

# Fonction pour lire un fichier CSV et retourner des listes de données (remplacement de pandas)
def lire_csv(fichier_csv):
    with open(fichier_csv, newline='') as fichier:
        lecteur_csv = csv.reader(fichier)
        donnees = [ligne for ligne in lecteur_csv]
    
    # Extraction des colonnes, ici nous considérons que la première ligne est l'en-tête
    colonnes = list(zip(*donnees[1:]))  # On ignore l'en-tête et on transpose pour avoir des colonnes
    # Convertir les valeurs en float
    colonnes_float = [[float(valeur) for valeur in colonne] for colonne in colonnes]
    
    return colonnes_float

# Fonction pour calculer les pas de temps à partir de la liste des temps (timestamps)
def calculer_pas_de_temps(timestamps):
    L_pas = []
    for i in range(1, len(timestamps)):
        # Calcul de la différence de temps entre chaque point
        pas = timestamps[i] - timestamps[i - 1]
        # Si le pas de temps est négatif ou nul, on le remplace par une petite valeur positive (ex: 0.001)
        if pas <= 0:
            pas = 0.001  # Valeur minimale pour éviter les problèmes avec sleep
        L_pas.append(pas)
    
    # Retourner les pas de temps calculés
    return L_pas

# Fonction d'intégration (intégrer une liste de données)
def integrate(L, L_pas, I0):
    taille = len(L)
    L_integree = [I0]
    aire = I0
    for i in range(taille - 1):
        largeur = L_pas[i] if i < len(L_pas) else L_pas[-1]  # Prendre le dernier pas si out of range
        hauteur = (L[i] + L[i + 1]) / 2
        aire += largeur * hauteur
        L_integree.append(aire)
    return L_integree

# Calcul des angles à partir des données de gyroscope
def calcule_angle(gyro_x, gyro_y, gyro_z, L_pas, a0, b0, c0):
    Nb_gyro = len(gyro_x)
    AngleX = integrate(gyro_x, L_pas, a0)
    AngleY = integrate(gyro_y, L_pas, b0)
    AngleZ = integrate(gyro_z, L_pas, c0)
    for i in range(Nb_gyro):
        AngleX[i] *= 180 / m.pi
        AngleY[i] *= 180 / m.pi
        AngleZ[i] *= 180 / m.pi
    return AngleX, AngleY, AngleZ

#------------------------------------------------------------------------------------#
#------------------------------ROTATION DE LA CHAUSSURE------------------------------#


# Fonction pour faire tourner l'objet "Shoe" dans un thread séparé
def rotate_object(L_angleX, L_angleY, L_angleZ, L_pas):
    global stop_rotation  # Utiliser la variable globale pour contrôler l'arrêt
    obj = bpy.data.objects.get('Shoe')

    if obj is None:
        print("L'objet 'Shoe' n'existe pas.")
        return

    # Initialiser l'index pour les rotations
    i = 0
    max_len = max(len(L_angleX), len(L_angleY), len(L_angleZ), len(L_pas))

    while not stop_rotation and i < max_len:
        # Appliquer les rotations calculées à partir des données du CSV
        x_rotation = L_angleX[i] if i < len(L_angleX) else 0
        y_rotation = L_angleY[i] if i < len(L_angleY) else 0
        z_rotation = L_angleZ[i] if i < len(L_angleZ) else 0
        pas_temps = L_pas[i] if i < len(L_pas) else 0.05  # Valeur par défaut si pas de temps manquant

        obj.rotation_euler = (
            m.radians(x_rotation),  # Rotation autour de l'axe X
            m.radians(y_rotation),  # Rotation autour de l'axe Y
            m.radians(z_rotation)   # Rotation autour de l'axe Z
        )

        # Insérer un keyframe optionnel si tu veux animer (facultatif)
        obj.keyframe_insert(data_path="rotation_euler", frame=i)

        i += 1
        time.sleep(pas_temps)  # Pause pour synchroniser la rotation avec le pas de temps

    print("Fin de l'animation.")

# Démarrer la fonction de rotation dans un thread séparé
def start_rotation_thread(L_angleX, L_angleY, L_angleZ, L_pas):
    rotation_thread = threading.Thread(target=rotate_object, args=(L_angleX, L_angleY, L_angleZ, L_pas))
    rotation_thread.daemon = True  # Le thread s'arrête quand Blender se ferme
    rotation_thread.start()
    
    
#------------------------------------------------------------------------------------#
#---------------------------------LECTURE DES DONNÉES--------------------------------#

# Lecture des fichiers CSV (exemples)
colonnes_accel = lire_csv('/Users/arseneseuillot/Library/CloudStorage/GoogleDrive-arsene.seuillot@gmail.com/Mon Drive/COURS/Centrale Lille/G2/S7/Challenges/Imagine&Make/Conception/Blender/Data/accelero.csv')
colonnes_gyro = lire_csv('/Users/arseneseuillot/Library/CloudStorage/GoogleDrive-arsene.seuillot@gmail.com/Mon Drive/COURS/Centrale Lille/G2/S7/Challenges/Imagine&Make/Conception/Blender/Data/gyro.csv')

# Suppose que les colonnes sont : [accel_x, accel_y, accel_z, temps] et [gyro_x, gyro_y, gyro_z]
accel_x = colonnes_accel[0]
accel_y = colonnes_accel[1]
accel_z = colonnes_accel[2]
L_temps = colonnes_accel[3]  # Colonne des timestamps (temps)

gyro_z = colonnes_gyro[0]
gyro_x = colonnes_gyro[1]
gyro_y = colonnes_gyro[2]

# Calcul des pas de temps (différences entre les timestamps)
L_pas = calculer_pas_de_temps(L_temps)

# Initialisation des angles (par exemple à zéro)
a0, b0, c0 = 0, 0, 0

# Calcul des angles de rotation à partir des données gyroscopiques
L_angleX, L_angleY, L_angleZ = calcule_angle(gyro_x, gyro_y, gyro_z, L_pas, a0, b0, c0)

#------------------------------------------------------------------------------------#
#--------------------------------DEMARRAGE DU PROGRAMME------------------------------#

# Démarrer le thread de rotation de l'objet en fonction des données du CSV
start_rotation_thread(L_angleX, L_angleY, L_angleZ, L_pas)
 
