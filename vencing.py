import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R

#------------------------------------------------------------------------------------#
#----------------------------- LECTURE DES DONNÉES CSV -----------------------------#

def lire_csv(fichier_csv):
    with open(fichier_csv, newline='') as fichier:
        lecteur_csv = csv.reader(fichier)
        next(lecteur_csv)  # Ignorer la première ligne (l'en-tête)
        donnees = [list(map(float, ligne)) for ligne in lecteur_csv]  # Conversion des données en float
    return np.array(donnees)

#------------------------------------------------------------------------------------#
#----------------------- FILTRE DE KALMAN POUR LA FUSION DE DONNÉES ----------------#

class KalmanFilter:
    def __init__(self, dim_x, dim_z):
        self.x = np.zeros((dim_x, 1))  # L'état du système (position, vitesse, etc.)
        self.P = np.eye(dim_x)         # Covariance de l'estimation
        self.F = np.eye(dim_x)         # Modèle de transition d'état (prédiction)
        self.H = np.zeros((dim_z, dim_x))  # Matrice de mesure
        self.R = np.eye(dim_z)         # Bruit de mesure (incertitude des capteurs)
        self.Q = np.eye(dim_x)         # Bruit du processus (incertitude du modèle)
        self.I = np.eye(dim_x)         # Matrice identité pour la mise à jour

    def predict(self):
        # Phase de prédiction de l'état
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        # Phase de mise à jour avec les nouvelles mesures
        y = z - np.dot(self.H, self.x)  # Erreur d'innovation
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R  # Covariance de l'erreur
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # Gain de Kalman

        # Mettre à jour l'estimation
        self.x = self.x + np.dot(K, y)
        self.P = np.dot(self.I - np.dot(K, self.H), self.P)

#------------------------------------------------------------------------------------#
#---------------------- CALCUL DES ANGLES VIA GYROSCOPE ----------------------------#

def calculer_pas_de_temps(timestamps):
    L_pas = np.diff(timestamps)
    L_pas[L_pas <= 0] = 0.001  # Éviter les pas de temps négatifs ou nuls
    return L_pas

def calculer_orientation_gyro(gyro_x, gyro_y, gyro_z, pas_de_temps):
    orientation = []
    quat = Quaternion()  # Quaternion initial

    for i in range(len(pas_de_temps)):
        delta_t = pas_de_temps[i]

        # Vitesse angulaire en radians par seconde
        wx, wy, wz = np.radians(gyro_x[i]), np.radians(gyro_y[i]), np.radians(gyro_z[i])
        
        # Calcul du quaternion de rotation basé sur le gyroscope
        dq = Quaternion(axis=[wx, wy, wz], radians=np.linalg.norm([wx, wy, wz]) * delta_t)
        
        # Mise à jour de l'orientation
        quat = quat * dq
        orientation.append(quat)

    return orientation

#------------------------------------------------------------------------------------#
#---------------------- CALCUL DE LA POSITION VIA ACCÉLÉROMÈTRE ---------------------#

def calculer_position(accel_x, accel_y, accel_z, orientations, pas_de_temps):
    position = np.zeros((len(pas_de_temps) + 1, 3))  # Initialisation à l'origine
    vitesse = np.zeros((len(pas_de_temps) + 1, 3))   # Vitesse initiale nulle
    filtre_kalman = KalmanFilter(dim_x=6, dim_z=3)  # Exemple avec 6 dimensions (position, vitesse, orientation)

    for i in range(1, len(pas_de_temps)):
        delta_t = pas_de_temps[i-1]

        # Accélération dans le repère du capteur
        accel = np.array([accel_x[i], accel_y[i], accel_z[i]])

        # Appliquer la rotation au repère global
        quat = orientations[i]
        accel_global = quat.rotate(accel)

        # Filtre de Kalman - prédiction de la vitesse
        filtre_kalman.predict()

        # Intégration de l'accélération
        vitesse[i] = vitesse[i-1] + accel_global * delta_t
        position[i] = position[i-1] + vitesse[i] * delta_t

        # Mise à jour du filtre de Kalman avec l'accélération
        filtre_kalman.update(accel_global.reshape(-1, 1))

    return position

#------------------------------------------------------------------------------------#
#--------------------- AFFICHAGE DES ANGLES ET DE LA TRAJECTOIRE -------------------#

def afficher_angles(angles):
    angles = np.array(angles)
    fig, ax = plt.subplots(3, 1, figsize=(10, 8))
    temps = np.arange(len(angles))

    ax[0].plot(temps, angles[:, 0], label="Roll (X)")
    ax[1].plot(temps, angles[:, 1], label="Pitch (Y)")
    ax[2].plot(temps, angles[:, 2], label="Yaw (Z)")

    ax[0].set_ylabel("Roll (degrés)")
    ax[1].set_ylabel("Pitch (degrés)")
    ax[2].set_ylabel("Yaw (degrés)")
    ax[2].set_xlabel("Temps (échantillons)")

    for a in ax:
        a.legend()
        a.grid(False)

    plt.tight_layout()
    plt.show()

def afficher_trajectoire(position):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Tracer les points
    ax.scatter(position[:, 0], position[:, 1], position[:, 2], c='k', s=3, label="Positions")

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.set_title("Trajectoire de la chaussure (accélération intégrée)")

    plt.show()


#------------------------------------------------------------------------------------#
#------------------------------ PROGRAMME PRINCIPAL ------------------------------#

fichier_gyro = 'gyro.csv'
fichier_accel = 'accel.csv'

# Lecture des données
donnees_gyro = lire_csv(fichier_gyro)
temps_gyro, gyro_x, gyro_y, gyro_z = donnees_gyro.T

donnees_accel = lire_csv(fichier_accel)
temps_accel, accel_x, accel_y, accel_z = donnees_accel.T

# Calcul des pas de temps
pas_de_temps = calculer_pas_de_temps(temps_gyro)

# Calcul des orientations
orientations = calculer_orientation_gyro(gyro_x, gyro_y, gyro_z, pas_de_temps)

# Convertir les quaternions en angles d'Euler
angles_euler = [R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True) for q in orientations]

# Affichage des angles
afficher_angles(angles_euler)

# Calcul de la position
position = calculer_position(accel_x, accel_y, accel_z, orientations, pas_de_temps)

# Affichage de la trajectoire
afficher_trajectoire(position)
