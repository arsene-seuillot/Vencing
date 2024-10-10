import csv
import bpy
import math as m

#------------------------------------------------------------------------------------#
#---------------------------------FONCTIONS UTILITAIRES------------------------------#

# Fonction pour lire un fichier CSV et retourner des listes de données
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
        if pas <= 0:
            pas = 0.001  # Valeur minimale pour éviter les problèmes
        L_pas.append(pas)
    
    return L_pas

# Fonction d'intégration (intégrer une liste de données)
def integrate(L, L_pas, I0):
    L_integree = [I0]  # Liste pour stocker les angles intégrés
    angle = I0         # Initialisation de l'angle à la valeur initiale I0
    
    # Effectuer l'intégration avec la règle des trapèzes
    for i in range(len(L_pas)):
        angle += L[i] * L_pas[i]  # Approximation : vitesse angulaire * intervalle de temps
        L_integree.append(angle)
    
    return L_integree

# Fonction pour appliquer une moyenne glissante (filtre passe-bas simple)
def moyenne_glissante(L, taille_fenetre=5):
    L_filtre = []
    for i in range(len(L)):
        # Prendre une moyenne sur une fenêtre centrée autour du point actuel
        start = max(0, i - taille_fenetre // 2)
        end = min(len(L), i + taille_fenetre // 2 + 1)
        moyenne = sum(L[start:end]) / (end - start)
        L_filtre.append(moyenne)
    return L_filtre

# Calcul des angles à partir des données de gyroscope
def calcule_angle(gyro_x, gyro_y, gyro_z, L_pas, a0, b0, c0, taille_fenetre=5):
    # Appliquer un filtre passe-bas (moyenne glissante) aux données du gyroscope
    gyro_x_filtre = moyenne_glissante(gyro_x, taille_fenetre)
    gyro_y_filtre = moyenne_glissante(gyro_y, taille_fenetre)
    gyro_z_filtre = moyenne_glissante(gyro_z, taille_fenetre)

    # Intégrer les vitesses angulaires filtrées pour obtenir les angles en radians
    AngleX = integrate(gyro_x_filtre, L_pas, a0)
    AngleY = integrate(gyro_y_filtre, L_pas, b0)
    AngleZ = integrate(gyro_z_filtre, L_pas, c0)
    
    # Conversion des angles en degrés
    for i in range(len(AngleX)):
        AngleX[i] = m.degrees(AngleX[i])
        AngleY[i] = m.degrees(AngleY[i])
        AngleZ[i] = m.degrees(AngleZ[i])
    
    return AngleX, AngleY, AngleZ

#------------------------------------------------------------------------------------#
#------------------------------ROTATION DE LA CHAUSSURE------------------------------#

# Fonction pour nettoyer les keyframes existants
def clear_keyframes(obj):
    if obj.animation_data:  # Vérifier si l'objet a des données d'animation
        obj.animation_data_clear()

# Fonction pour nettoyer tous les keyframes de tous les objets
def clear_all_keyframes():
    for obj in bpy.data.objects:
        if obj.animation_data:
            clear_keyframes(obj)
    print("Tous les keyframes ont été supprimés.")

# Fonction pour insérer les keyframes sans multithreading
def insert_keyframes(L_angleX, L_angleY, L_angleZ):
    obj = bpy.data.objects.get('Shoe')

    if obj is None:
        print("L'objet 'Shoe' n'existe pas.")
        return

    # Supprimer les keyframes précédents
    clear_keyframes(obj)

    # Initialiser l'index pour les rotations
    max_len = max(len(L_angleX), len(L_angleY), len(L_angleZ))

    for i in range(max_len):
        # Appliquer les rotations calculées à partir des données du CSV
        x_rotation = L_angleX[i] if i < len(L_angleX) else 0
        y_rotation = L_angleY[i] if i < len(L_angleY) else 0
        z_rotation = L_angleZ[i] if i < len(L_angleZ) else 0

        # Debug: imprimer les valeurs des rotations
        print(f"Frame {i+1}: X={x_rotation}, Y={y_rotation}, Z={z_rotation}")

        # Assurez-vous d'utiliser l'ordre correct pour Blender, souvent ZYX est utilisé
        obj.rotation_euler = (
            m.radians(z_rotation),  # Rotation autour de l'axe Z
            m.radians(y_rotation),  # Rotation autour de l'axe Y
            m.radians(x_rotation)   # Rotation autour de l'axe X
        )

        # Insérer un keyframe pour la rotation
        obj.keyframe_insert(data_path="rotation_euler", frame=i + 1)  # i+1 pour éviter la frame 0

    print("Keyframes insérés.")

#------------------------------------------------------------------------------------#
#--------------------------------LECTURE DES DONNÉES--------------------------------#

# Lecture des fichiers CSV
colonnes_gyro = lire_csv('/Users/arseneseuillot/Library/CloudStorage/GoogleDrive-arsene.seuillot@gmail.com/Mon Drive/COURS/Centrale Lille/G2/S7/Challenges/Imagine&Make/Conception/Blender/Data/gyro.csv')

# Suppose que les colonnes sont : [gyro_x, gyro_y, gyro_z, temps]
gyro_z = colonnes_gyro[0]
gyro_x = colonnes_gyro[1]
gyro_y = colonnes_gyro[2]
L_temps = colonnes_gyro[3]  # Colonne des timestamps (temps)

# Calcul des pas de temps (différences entre les timestamps)
L_pas = calculer_pas_de_temps(L_temps)

# Initialisation des angles (à zéro pour commencer)
a0, b0, c0 = 0, 0, 0

# Calcul des angles de rotation à partir des données gyroscopiques avec un filtrage (taille fenêtre de 5 points)
L_angleX, L_angleY, L_angleZ = calcule_angle(gyro_x, gyro_y, gyro_z, L_pas, a0, b0, c0, taille_fenetre=5)

#------------------------------------------------------------------------------------#
#--------------------------------DEMARRAGE DU PROGRAMME------------------------------#

# Effacer tous les keyframes avant d'insérer de nouveaux keyframes
clear_all_keyframes()

# Insérer les keyframes dans la timeline de Blender
insert_keyframes(L_angleX, L_angleY, L_angleZ)
