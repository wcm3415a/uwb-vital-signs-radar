import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import struct
import numpy as np
import threading
import traceback
import time
import scipy.ndimage as ndimage
from collections import deque
from vmdpy import VMD

# ==========================================
# CONFIGURATION MATÉRIELLE
# ==========================================
PORT = '/dev/cu.usbmodem0007602221481'  
BAUDRATE = 460800
HEADER = b'\xde\xca\xad\xde'
TAILLE_PAYLOAD = 1024
NB_ECHANTILLONS = 256
AFFICHAGE_BINS = 100

# ==========================================
# PARAMÈTRES CIR & CIBLE
# ==========================================
ALPHA_RAPIDE = 0.1         
ALPHA_ENVIRONNEMENT = 0.02 
SEUIL_MOUVEMENT = 800      
BIN_CIBLE = 19  # Cible fixe à 60 cm
SEUIL_PRESENCE_PHASE = 900

# ==========================================
# PARAMÈTRES RESPIRATION & VMD
# ==========================================
FPS_ESTIME = 15
HISTORIQUE_TAILLE = 300
MARGE_AFFICHAGE = 15
ALPHA_LISSAGE_PHASE = 0.15
SEUIL_REFERENCE = 400

# ==========================================
# VARIABLES GLOBALES
# ==========================================
verrou = threading.Lock()
arret_demande = False
premier_echantillon_recu = False
BIN_REFERENCE = 0  

# Buffers CIR
buffer_cir_median = deque(maxlen=3) # NOUVEAU: Buffer pour filtre médian
cir_brut = np.zeros(NB_ECHANTILLONS)
cir_lisse = np.zeros(NB_ECHANTILLONS)
cir_mouvement = np.zeros(NB_ECHANTILLONS)
amp_rapide = np.zeros(NB_ECHANTILLONS)
amp_lente = np.zeros(NB_ECHANTILLONS)

# Buffers Temporels
historique_phases_cible_brute = deque(maxlen=HISTORIQUE_TAILLE)
historique_phases_corrigees = deque(maxlen=HISTORIQUE_TAILLE)

signal_cible_brute_centre = np.zeros(HISTORIQUE_TAILLE) 
signal_phase_corrigee_centre = np.zeros(HISTORIQUE_TAILLE) 
signal_respiratoire_vmd = np.zeros(HISTORIQUE_TAILLE) 

I_lisse_phase, Q_lisse_phase = 0.0, 0.0
premier_echantillon_phase = False

I_lisse_ref, Q_lisse_ref = 0.0, 0.0
premier_echantillon_ref = False

# Buffers pour le lissage complexe de la différence
I_lisse_diff, Q_lisse_diff = 0.0, 0.0
premier_echantillon_diff = False

# ==========================================
# THREAD 1 : TRAITEMENT RADAR (TEMPS RÉEL)
# ==========================================
def lecture_serie():
    global cir_brut, cir_lisse, cir_mouvement, amp_rapide, amp_lente
    global I_lisse_phase, Q_lisse_phase, premier_echantillon_phase
    global I_lisse_ref, Q_lisse_ref, premier_echantillon_ref
    global I_lisse_diff, Q_lisse_diff, premier_echantillon_diff
    global premier_echantillon_recu, arret_demande
    global signal_cible_brute_centre, signal_phase_corrigee_centre
    global BIN_REFERENCE, buffer_cir_median
    
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print("✅ Radar connecté. Lecture en cours...")
        ser.reset_input_buffer()
    except Exception as e:
        print(f"❌ Erreur de connexion : {e}")
        return

    while not arret_demande:
        try:
            ser.read_until(HEADER)
            payload = ser.read(TAILLE_PAYLOAD)
            
            if len(payload) == TAILLE_PAYLOAD:
                valeurs_iq = struct.unpack('<512h', payload)
                I = np.array(valeurs_iq[0::2], dtype=float)
                Q = np.array(valeurs_iq[1::2], dtype=float)
                
                # --- PRE-PROCESSING ---
                I = I - np.mean(I)
                Q = Q - np.mean(Q)
                I = ndimage.gaussian_filter1d(I, sigma=1.0)
                Q = ndimage.gaussian_filter1d(Q, sigma=1.0)
                
                amplitude = np.sqrt(I**2 + Q**2)

                # --- SOLUTION 2: FILTRE MÉDIAN TEMPOREL (SLOW-TIME) ---
                buffer_cir_median.append(amplitude)
                if len(buffer_cir_median) == 3:
                    # Applique la médiane sur les 3 derniers CIR reçus
                    amplitude_filtree = np.median(buffer_cir_median, axis=0)
                else:
                    amplitude_filtree = amplitude

                with verrou:
                    # On utilise dorénavant l'amplitude stabilisée
                    cir_brut = amplitude_filtree.copy()
                    
                    if not premier_echantillon_recu:
                        amp_rapide = amplitude_filtree.copy()
                        amp_lente = amplitude_filtree.copy()
                        premier_echantillon_recu = True
                    else:
                        amp_rapide = (ALPHA_RAPIDE * amplitude_filtree) + ((1 - ALPHA_RAPIDE) * amp_rapide)
                        amp_lente = (ALPHA_ENVIRONNEMENT * amplitude_filtree) + ((1 - ALPHA_ENVIRONNEMENT) * amp_lente)
                        
                    cir_lisse = amp_lente.copy()
                    cir_mouvement = np.abs(amp_rapide - amp_lente)
                    cir_mouvement[cir_mouvement < SEUIL_MOUVEMENT] = 0

                    # --- VERROUILLAGE RÉFÉRENCE ---
                    amp_recherche = amp_lente[:AFFICHAGE_BINS].copy()
                    idx_pic1 = np.argmax(amp_recherche)
                    borne_sup = min(AFFICHAGE_BINS, idx_pic1 + 5)
                    amp_recherche[:borne_sup] = 0 
                    
                    nouveau_bin_ref = np.argmax(amp_recherche)
                    val_max_recherche = amp_recherche[nouveau_bin_ref]
                    val_ref_actuelle = amp_lente[BIN_REFERENCE] if BIN_REFERENCE < AFFICHAGE_BINS else 0

                    if val_max_recherche > SEUIL_REFERENCE:
                        if BIN_REFERENCE == 0 or val_ref_actuelle < SEUIL_REFERENCE or val_max_recherche > val_ref_actuelle * 1.50:
                            if nouveau_bin_ref != BIN_REFERENCE:
                                BIN_REFERENCE = nouveau_bin_ref
                                premier_echantillon_ref = False
                                premier_echantillon_diff = False 
                                historique_phases_cible_brute.clear()
                                historique_phases_corrigees.clear()
                                signal_cible_brute_centre.fill(0)
                                signal_phase_corrigee_centre.fill(0)

                    # --- EXTRACTION I/Q ET PORTE DE BRUIT ---
                    I_cible, Q_cible = I[BIN_CIBLE], Q[BIN_CIBLE]
                    mouvement_cible = cir_mouvement[BIN_CIBLE]
                    amp_cible = amplitude_filtree[BIN_CIBLE] # Utilisation de l'amplitude stabilisée
                    
                    bin_ref_securise = BIN_REFERENCE if BIN_REFERENCE > 0 else BIN_CIBLE
                    I_ref, Q_ref = I[bin_ref_securise], Q[bin_ref_securise]
                    
                    # CONDITION DE PRÉSENCE
                    if amp_cible > SEUIL_PRESENCE_PHASE:
                        if not premier_echantillon_phase:
                            I_lisse_phase, Q_lisse_phase = I_cible, Q_cible
                            premier_echantillon_phase = True
                        else:
                            I_lisse_phase = (ALPHA_LISSAGE_PHASE * I_cible) + ((1 - ALPHA_LISSAGE_PHASE) * I_lisse_phase)
                            Q_lisse_phase = (ALPHA_LISSAGE_PHASE * Q_cible) + ((1 - ALPHA_LISSAGE_PHASE) * Q_lisse_phase)

                        if not premier_echantillon_ref:
                            I_lisse_ref, Q_lisse_ref = I_ref, Q_ref
                            premier_echantillon_ref = True
                        else:
                            I_lisse_ref = (ALPHA_LISSAGE_PHASE * I_ref) + ((1 - ALPHA_LISSAGE_PHASE) * I_lisse_ref)
                            Q_lisse_ref = (ALPHA_LISSAGE_PHASE * Q_ref) + ((1 - ALPHA_LISSAGE_PHASE) * Q_lisse_ref)
                            
                        phase_cible_brute = np.arctan2(Q_lisse_phase, I_lisse_phase)
                        
                        if BIN_REFERENCE > 0:
                            # 1. Calcul de la différence brute
                            I_diff_brut = (I_lisse_phase * I_lisse_ref) + (Q_lisse_phase * Q_lisse_ref)
                            Q_diff_brut = (Q_lisse_phase * I_lisse_ref) - (I_lisse_phase * Q_lisse_ref)
                            
                            # 2. Lissage des composantes complexes I_diff et Q_diff
                            if not premier_echantillon_diff:
                                I_lisse_diff = I_diff_brut
                                Q_lisse_diff = Q_diff_brut
                                premier_echantillon_diff = True
                            else:
                                I_lisse_diff = (ALPHA_LISSAGE_PHASE * I_diff_brut) + ((1 - ALPHA_LISSAGE_PHASE) * I_lisse_diff)
                                Q_lisse_diff = (ALPHA_LISSAGE_PHASE * Q_diff_brut) + ((1 - ALPHA_LISSAGE_PHASE) * Q_lisse_diff)
                            
                            # 3. Extraction de l'angle assaini
                            phase_corrigee = np.arctan2(Q_lisse_diff, I_lisse_diff)
                        else:
                            phase_corrigee = phase_cible_brute
                    else:
                        premier_echantillon_phase = False
                        premier_echantillon_ref = False
                        premier_echantillon_diff = False 
                        phase_cible_brute = 0.0
                        phase_corrigee = 0.0
                        
                    historique_phases_cible_brute.append(phase_cible_brute)
                    historique_phases_corrigees.append(phase_corrigee)
                    
                    if len(historique_phases_corrigees) > 50:
                        phases_cible_array = np.array(historique_phases_cible_brute)
                        phases_cible_unwrapped = np.unwrap(phases_cible_array)
                        signal_cible_brute_centre = phases_cible_unwrapped - np.mean(phases_cible_unwrapped)

                        phases_corr_array = np.array(historique_phases_corrigees)
                        phases_corr_unwrapped = np.unwrap(phases_corr_array)
                        signal_phase_corrigee_centre = phases_corr_unwrapped - np.mean(phases_corr_unwrapped)

        except Exception:
            pass
            
    ser.close()

# ==========================================
# THREAD 2 : MOTEUR VMD (ASYNCHRONE)
# ==========================================
def traitement_vmd_asynchrone():
    global signal_respiratoire_vmd, arret_demande
    
    alpha = 2000       
    tau = 0.           
    K = 3              
    DC = 0             
    init = 1           
    tol = 1e-7         
    
    while not arret_demande:
        time.sleep(0.2) 
        
        with verrou:
            if len(historique_phases_corrigees) < 100:
                continue
            phases_a_traiter = np.array(signal_phase_corrigee_centre)
            
        if np.max(np.abs(phases_a_traiter)) < 0.01:
            with verrou:
                signal_respiratoire_vmd = np.zeros(len(phases_a_traiter))
            continue
            
        try:
            u, u_hat, omega = VMD(phases_a_traiter, alpha, tau, K, DC, init, tol)
            frequences_hz = omega[-1] * FPS_ESTIME
            idx_resp = np.argmin(np.abs(frequences_hz - 0.3))
            mode_respiration = u[idx_resp, :]
            
            with verrou:
                signal_respiratoire_vmd = mode_respiration
                
        except Exception as e:
            pass

# ==========================================
# GUI (MATPLOTLIB DASHBOARD)
# ==========================================
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 12))
fig.canvas.manager.set_window_title('UWB Radar - VMD Processing Dashboard')

# --- PLOT 1: CIR ---
ligne_brute, = ax1.plot(range(AFFICHAGE_BINS), np.zeros(AFFICHAGE_BINS), color='lightgray', linewidth=1, label='Raw CIR', alpha=0.5)
ligne_lisse, = ax1.plot(range(AFFICHAGE_BINS), np.zeros(AFFICHAGE_BINS), color='blue', linewidth=2, label='Static Env', alpha=0.7)
ligne_mouvement, = ax1.plot(range(AFFICHAGE_BINS), np.zeros(AFFICHAGE_BINS), color='red', linewidth=2, label='Motion')

ligne_cible = ax1.axvline(x=BIN_CIBLE, color='green', linestyle='--', label=f'Target Bin ({BIN_CIBLE})')
ligne_ref = ax1.axvline(x=0, color='orange', linestyle='--', label='Ref Bin (Locked)')

ax1.set_ylim(0, 8000)
ax1.set_xlim(0, AFFICHAGE_BINS)
ax1.set_title("Spatial Analysis: CIR (Green: Fixed Target, Orange: Locked Reference)")
ax1.set_ylabel("Amplitude")
ax1.legend(loc="upper right")
ax1.grid(True, linestyle='--', alpha=0.5)

# --- PLOT 2: Raw Target Phase ---
ligne_raw_cible, = ax2.plot(np.zeros(HISTORIQUE_TAILLE), color='magenta', linewidth=2)
ax2.set_xlim(0, HISTORIQUE_TAILLE)
ax2.set_ylim(-20, 20)
ax2.set_title(f"Temporal Analysis: Raw Phase of Target (Bin {BIN_CIBLE})")
ax2.set_ylabel("Phase (Rads)")
ax2.grid(True, linestyle='--', alpha=0.5)

# --- PLOT 3: Corrected Phase (Target - Ref) ---
ligne_corr_phase, = ax3.plot(np.zeros(HISTORIQUE_TAILLE), color='purple', linewidth=2)
ax3.set_xlim(0, HISTORIQUE_TAILLE)
ax3.set_ylim(-10, 10)
ax3.set_title("Temporal Analysis: Corrected Phase (Phase Referencing)")
ax3.set_ylabel("Phase (Rads)")
ax3.grid(True, linestyle='--', alpha=0.5)

# --- PLOT 4: VMD Respiration Signal ---
ligne_resp, = ax4.plot(np.zeros(HISTORIQUE_TAILLE), color='darkorange', linewidth=2, label='VMD IMF (Respiration Mode)')
ax4.set_xlim(0, HISTORIQUE_TAILLE - MARGE_AFFICHAGE)
ax4.set_ylim(-5, 5)
ax4.set_title("Temporal Analysis: VMD Extracted Respiration Signal")
ax4.set_xlabel("Time (frames)")
ax4.set_ylabel("Phase (Rads)")
ax4.legend(loc="upper right")
ax4.grid(True, linestyle='--', alpha=0.5)

plt.tight_layout()

# ==========================================
# ANIMATION LOOP
# ==========================================
def actualiser_graphique(frame):
    with verrou:
        ligne_brute.set_ydata(cir_brut[:AFFICHAGE_BINS])
        ligne_lisse.set_ydata(cir_lisse[:AFFICHAGE_BINS])
        ligne_mouvement.set_ydata(cir_mouvement[:AFFICHAGE_BINS])
        ligne_ref.set_xdata([BIN_REFERENCE, BIN_REFERENCE])
        
        label_ref = f'Ref Bin ({BIN_REFERENCE})' if BIN_REFERENCE > 0 else 'Ref Bin (No Ref > 400)'
        ligne_ref.set_label(label_ref)
        ax1.legend(loc="upper right")

        def pad_signal(signal_in):
            taille_actuelle = len(signal_in)
            return np.pad(signal_in, (0, HISTORIQUE_TAILLE - taille_actuelle), 'constant', constant_values=0)

        ligne_raw_cible.set_ydata(pad_signal(signal_cible_brute_centre))
        ligne_corr_phase.set_ydata(pad_signal(signal_phase_corrigee_centre))
        
        donnees_a_afficher = np.copy(signal_respiratoire_vmd)
        if len(historique_phases_corrigees) > MARGE_AFFICHAGE * 2:
            donnees_propres = donnees_a_afficher[:-MARGE_AFFICHAGE]
        else:
            donnees_propres = donnees_a_afficher
            
        ligne_resp.set_ydata(pad_signal(donnees_propres))
        
    return ligne_brute, ligne_lisse, ligne_mouvement, ligne_cible, ligne_ref, ligne_raw_cible, ligne_corr_phase, ligne_resp

if __name__ == '__main__':
    thread_serie = threading.Thread(target=lecture_serie)
    thread_serie.daemon = True
    thread_serie.start()
    
    thread_vmd = threading.Thread(target=traitement_vmd_asynchrone)
    thread_vmd.daemon = True
    thread_vmd.start()

    try:
        print("⏳ Starting graphical interface...")
        ani = animation.FuncAnimation(fig, actualiser_graphique, interval=50, blit=False)
        plt.show()
    except KeyboardInterrupt:
        print("\n🛑 Stop requested by user.")
    finally:
        arret_demande = True
        thread_serie.join(timeout=1)
        thread_vmd.join(timeout=1)
        print("✅ Program terminated properly.")