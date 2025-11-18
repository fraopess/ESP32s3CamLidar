#!/usr/bin/env python3
"""
Visualiseur temps réel pour ESP32-CAM - Protocole Binaire
Lecture rapide et robuste avec checksum

Installation:
pip install pyserial matplotlib numpy

Usage:
python visualizer_binary.py /dev/ttyUSB0
python visualizer_binary.py COM3  # Windows
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
import numpy as np
from collections import deque
import sys
import time
import struct

# Configuration
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
MAX_POINTS = 500

# Format du paquet binaire
# [0xAA 0x55][timestamp_ms(4)][vx_mm/s(2)][vy_mm/s(2)][dist_cm(2)][checksum(1)]
PACKET_HEADER1 = 0xAA
PACKET_HEADER2 = 0x55
PACKET_SIZE = 13  # 2 header + 4 timestamp + 2 vx + 2 vy + 2 dist + 1 checksum


class BinaryPacketParser:
    """Parser pour paquets binaires avec checksum"""
    
    def __init__(self):
        self.buffer = bytearray()
        self.packets_received = 0
        self.packets_corrupted = 0
        
    def calculate_checksum(self, data):
        """Calcul checksum XOR"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    
    def parse_packet(self, packet_bytes):
        """Parse un paquet binaire complet de 13 bytes"""
        if len(packet_bytes) != PACKET_SIZE:
            return None
            
        # Vérifier header
        if packet_bytes[0] != PACKET_HEADER1 or packet_bytes[1] != PACKET_HEADER2:
            return None
        
        # Extraire données (Little Endian)
        try:
            # timestamp_ms: uint32 (4 bytes)
            timestamp_ms = struct.unpack('<I', packet_bytes[2:6])[0]
            
            # velocity_x: int16 (2 bytes) en mm/s
            velocity_x_mms = struct.unpack('<h', packet_bytes[6:8])[0]
            
            # velocity_y: int16 (2 bytes) en mm/s
            velocity_y_mms = struct.unpack('<h', packet_bytes[8:10])[0]
            
            # distance: uint16 (2 bytes) en cm
            distance_cm = struct.unpack('<H', packet_bytes[10:12])[0]
            
            # checksum: uint8 (1 byte)
            checksum_received = packet_bytes[12]
            
            # Vérifier checksum (XOR des bytes 2 à 11)
            checksum_calculated = self.calculate_checksum(packet_bytes[2:12])
            
            if checksum_calculated != checksum_received:
                self.packets_corrupted += 1
                return None
            
            self.packets_received += 1
            
            # Conversion en unités réelles
            return {
                'timestamp_ms': timestamp_ms,
                'velocity_x': velocity_x_mms / 1000.0,  # mm/s -> m/s
                'velocity_y': velocity_y_mms / 1000.0,  # mm/s -> m/s
                'distance': distance_cm  # déjà en cm
            }
            
        except struct.error as e:
            print(f"Erreur struct.unpack: {e}")
            return None
    
    def add_bytes(self, new_bytes):
        """Ajoute des bytes au buffer et extrait les paquets valides"""
        self.buffer.extend(new_bytes)
        packets = []
        
        while len(self.buffer) >= PACKET_SIZE:
            # Rechercher header 0xAA 0x55
            header_idx = -1
            for i in range(len(self.buffer) - 1):
                if self.buffer[i] == PACKET_HEADER1 and self.buffer[i+1] == PACKET_HEADER2:
                    header_idx = i
                    break
            
            if header_idx == -1:
                # Pas de header trouvé, garder seulement le dernier byte
                # (au cas où c'est 0xAA et que 0x55 arrive ensuite)
                if len(self.buffer) > 0 and self.buffer[-1] == PACKET_HEADER1:
                    self.buffer = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            
            # Supprimer bytes avant header
            if header_idx > 0:
                self.buffer = self.buffer[header_idx:]
            
            # Vérifier si on a un paquet complet
            if len(self.buffer) >= PACKET_SIZE:
                packet_bytes = bytes(self.buffer[:PACKET_SIZE])
                packet = self.parse_packet(packet_bytes)
                
                if packet is not None:
                    packets.append(packet)
                
                # Retirer le paquet du buffer
                self.buffer = self.buffer[PACKET_SIZE:]
            else:
                # Pas assez de bytes pour un paquet complet
                break
        
        return packets


class OpticalFlowVisualizer:
    """Visualiseur temps réel pour données de flux optique"""
    
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.parser = BinaryPacketParser()
        
        # Buffers de données avec taille maximale
        self.times = deque(maxlen=MAX_POINTS)
        self.velocity_x = deque(maxlen=MAX_POINTS)
        self.velocity_y = deque(maxlen=MAX_POINTS)
        self.velocity_mag = deque(maxlen=MAX_POINTS)
        self.distance = deque(maxlen=MAX_POINTS)
        
        # Statistiques
        self.start_time_ms = None
        self.data_count = 0
        self.last_fps_calc = time.time()
        self.fps_counter = 0
        self.current_fps = 0.0
        
    def connect(self):
        """Connexion au port série"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"✓ Connecté à {self.port} @ {self.baudrate} baud")
            time.sleep(2)
            
            # Lire messages texte initiaux (avant le mode binaire)
            print("\n" + "="*60)
            print("Messages d'initialisation ESP32:")
            print("="*60)
            start = time.time()
            while time.time() - start < 3:
                if self.ser.in_waiting:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line and not line.startswith('\x00'):
                            print(f"  {line}")
                    except:
                        break
            
            self.ser.reset_input_buffer()
            print("="*60)
            print("✓ Réception mode binaire activée\n")
            return True
            
        except Exception as e:
            print(f"✗ Erreur connexion: {e}")
            return False
    
    def read_data(self):
        """Lecture et parsing des données binaires"""
        if not self.ser or not self.ser.in_waiting:
            return False
        
        # Lire tous les bytes disponibles
        try:
            new_bytes = self.ser.read(self.ser.in_waiting)
        except Exception as e:
            print(f"Erreur lecture série: {e}")
            return False
        
        # Parser les paquets
        packets = self.parser.add_bytes(new_bytes)
        
        # Traiter chaque paquet reçu
        for packet in packets:
            # Premier timestamp = référence temps 0
            if self.start_time_ms is None:
                self.start_time_ms = packet['timestamp_ms']
            
            # Temps relatif en secondes
            rel_time = (packet['timestamp_ms'] - self.start_time_ms) / 1000.0
            
            # Calcul magnitude vitesse
            vel_mag = np.sqrt(packet['velocity_x']**2 + packet['velocity_y']**2)
            
            # Ajout aux buffers
            self.times.append(rel_time)
            self.velocity_x.append(packet['velocity_x'])
            self.velocity_y.append(packet['velocity_y'])
            self.velocity_mag.append(vel_mag)
            self.distance.append(packet['distance'])
            
            self.data_count += 1
            self.fps_counter += 1
        
        # Calcul FPS de réception
        now = time.time()
        if now - self.last_fps_calc >= 1.0:
            self.current_fps = self.fps_counter / (now - self.last_fps_calc)
            self.fps_counter = 0
            self.last_fps_calc = now
        
        return len(packets) > 0
    
    def init_plot(self):
        """Initialisation des graphiques matplotlib"""
        self.fig = plt.figure(figsize=(15, 10))
        self.fig.suptitle('ESP32-CAM Optical Flow + LiDAR - Protocole Binaire', 
                         fontsize=14, fontweight='bold')
        gs = GridSpec(3, 2, figure=self.fig, hspace=0.35, wspace=0.3)
        
        # Graphique 1: Vitesse X et Y (ligne du temps)
        self.ax1 = self.fig.add_subplot(gs[0, :])
        self.line_vx, = self.ax1.plot([], [], 'r-', label='Velocity X', linewidth=1.5, alpha=0.8)
        self.line_vy, = self.ax1.plot([], [], 'b-', label='Velocity Y', linewidth=1.5, alpha=0.8)
        self.ax1.set_ylabel('Vitesse (m/s)', fontsize=10)
        self.ax1.set_title('Vitesse X et Y', fontsize=11, fontweight='bold')
        self.ax1.legend(loc='upper right', fontsize=9)
        self.ax1.grid(True, alpha=0.3, linestyle='--')
        self.ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.5)
        
        # Graphique 2: Magnitude vitesse
        self.ax2 = self.fig.add_subplot(gs[1, 0])
        self.line_vmag, = self.ax2.plot([], [], 'g-', linewidth=2)
        self.ax2.set_ylabel('Magnitude (m/s)', fontsize=10)
        self.ax2.set_xlabel('Temps (s)', fontsize=10)
        self.ax2.set_title('Magnitude de la Vitesse', fontsize=11, fontweight='bold')
        self.ax2.grid(True, alpha=0.3, linestyle='--')
        
        # Graphique 3: Distance LiDAR
        self.ax3 = self.fig.add_subplot(gs[1, 1])
        self.line_dist, = self.ax3.plot([], [], 'm-', linewidth=2)
        self.ax3.set_ylabel('Distance (cm)', fontsize=10)
        self.ax3.set_xlabel('Temps (s)', fontsize=10)
        self.ax3.set_title('Distance LiDAR', fontsize=11, fontweight='bold')
        self.ax3.grid(True, alpha=0.3, linestyle='--')
        
        # Graphique 4: Trajectoire 2D (Vx vs Vy)
        self.ax4 = self.fig.add_subplot(gs[2, 0])
        self.ax4.set_xlabel('Velocity X (m/s)', fontsize=10)
        self.ax4.set_ylabel('Velocity Y (m/s)', fontsize=10)
        self.ax4.set_title('Trajectoire Vitesse (Vx vs Vy)', fontsize=11, fontweight='bold')
        self.ax4.grid(True, alpha=0.3, linestyle='--')
        self.ax4.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        self.ax4.axvline(x=0, color='k', linestyle='-', linewidth=0.5)
        self.ax4.set_aspect('equal', adjustable='box')
        
        # Graphique 5: Statistiques temps réel
        self.ax5 = self.fig.add_subplot(gs[2, 1])
        self.ax5.axis('off')
        self.stats_text = self.ax5.text(0.05, 0.95, '', fontsize=10, 
                                        verticalalignment='top',
                                        family='monospace',
                                        bbox=dict(boxstyle='round', 
                                                facecolor='lightblue', 
                                                alpha=0.8,
                                                edgecolor='navy',
                                                linewidth=2))
        
    def update_plot(self, frame):
        """Mise à jour des graphiques (appelée par FuncAnimation)"""
        # Lecture de nouvelles données (jusqu'à 20 paquets par frame)
        for _ in range(20):
            self.read_data()
        
        if len(self.times) < 2:
            return
        
        # Conversion en numpy arrays pour traitement
        times_array = np.array(self.times)
        vx_array = np.array(self.velocity_x)
        vy_array = np.array(self.velocity_y)
        vmag_array = np.array(self.velocity_mag)
        dist_array = np.array(self.distance)
        
        # Mise à jour vitesse X et Y
        self.line_vx.set_data(times_array, vx_array)
        self.line_vy.set_data(times_array, vy_array)
        self.ax1.relim()
        self.ax1.autoscale_view()
        
        # Mise à jour magnitude
        self.line_vmag.set_data(times_array, vmag_array)
        self.ax2.relim()
        self.ax2.autoscale_view()
        
        # Mise à jour distance
        self.line_dist.set_data(times_array, dist_array)
        self.ax3.relim()
        self.ax3.autoscale_view()
        
        # Mise à jour trajectoire 2D avec couleur basée sur le temps
        self.ax4.clear()
        if len(vx_array) > 0:
            scatter = self.ax4.scatter(vx_array, vy_array, c=times_array, 
                                       cmap='viridis', s=30, alpha=0.6, 
                                       edgecolors='black', linewidth=0.5)
            # Point actuel en rouge
            self.ax4.scatter(vx_array[-1], vy_array[-1], c='red', s=100, 
                           marker='o', edgecolors='darkred', linewidth=2, 
                           label='Position actuelle', zorder=5)
        
        self.ax4.set_xlabel('Velocity X (m/s)', fontsize=10)
        self.ax4.set_ylabel('Velocity Y (m/s)', fontsize=10)
        self.ax4.set_title('Trajectoire Vitesse (Vx vs Vy)', fontsize=11, fontweight='bold')
        self.ax4.grid(True, alpha=0.3, linestyle='--')
        self.ax4.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        self.ax4.axvline(x=0, color='k', linestyle='-', linewidth=0.5)
        self.ax4.legend(fontsize=8, loc='upper right')
        
        # Calcul statistiques
        valid_distances = dist_array[dist_array > 0]
        
        # Formatage du texte de statistiques
        stats = "╔═══════════════════════════════╗\n"
        stats += "║  STATISTIQUES TEMPS RÉEL      ║\n"
        stats += "╚═══════════════════════════════╝\n\n"
        
        stats += f"📊 Acquisition:\n"
        stats += f"  • Points reçus: {self.data_count}\n"
        stats += f"  • FPS réception: {self.current_fps:.1f}\n"
        stats += f"  • Durée: {times_array[-1]:.1f} s\n\n"
        
        stats += f"🏃 Vitesse:\n"
        stats += f"  • Vx moyen: {np.mean(vx_array):+.4f} m/s\n"
        stats += f"  • Vy moyen: {np.mean(vy_array):+.4f} m/s\n"
        stats += f"  • Magnitude moy: {np.mean(vmag_array):.4f} m/s\n"
        stats += f"  • Magnitude max: {np.max(vmag_array):.4f} m/s\n\n"
        
        if len(valid_distances) > 0:
            stats += f"📏 Distance LiDAR:\n"
            stats += f"  • Moyenne: {np.mean(valid_distances):.1f} cm\n"
            stats += f"  • Min: {np.min(valid_distances):.0f} cm\n"
            stats += f"  • Max: {np.max(valid_distances):.0f} cm\n"
            stats += f"  • Validité: {100*len(valid_distances)/len(dist_array):.1f}%\n\n"
        else:
            stats += f"📏 Distance LiDAR:\n"
            stats += f"  • Aucune mesure valide\n\n"
        
        stats += f"🔒 Qualité transmission:\n"
        stats += f"  • Paquets OK: {self.parser.packets_received}\n"
        stats += f"  • Paquets corrompus: {self.parser.packets_corrupted}\n"
        
        if self.parser.packets_received + self.parser.packets_corrupted > 0:
            total_packets = self.parser.packets_received + self.parser.packets_corrupted
            error_rate = 100 * self.parser.packets_corrupted / total_packets
            stats += f"  • Taux d'erreur: {error_rate:.3f}%"
        
        self.stats_text.set_text(stats)
        
    def run(self):
        """Lancement de la visualisation temps réel"""
        if not self.connect():
            return
        
        print("Initialisation des graphiques...")
        self.init_plot()
        
        print("\n" + "="*60)
        print("✓ Visualisation démarrée")
        print(f"  Format paquet: {PACKET_SIZE} bytes")
        print(f"  Affichage: {MAX_POINTS} points max")
        print("\nAppuyez sur Ctrl+C pour arrêter")
        print("="*60 + "\n")
        
        # Animation matplotlib (mise à jour toutes les 50ms)
        ani = animation.FuncAnimation(
            self.fig, 
            self.update_plot,
            interval=50,
            blit=False,
            cache_frame_data=False
        )
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n✓ Arrêt demandé")
        
        # Fermeture propre
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Port série fermé")
            
        # Statistiques finales
        print("\n" + "="*60)
        print("STATISTIQUES FINALES")
        print("="*60)
        print(f"Total paquets reçus: {self.parser.packets_received}")
        print(f"Total paquets corrompus: {self.parser.packets_corrupted}")
        if self.parser.packets_received > 0:
            total = self.parser.packets_received + self.parser.packets_corrupted
            print(f"Taux de réussite: {100*self.parser.packets_received/total:.2f}%")
        print("="*60)


def main():
    """Point d'entrée principal"""
    # Port série depuis argument ou défaut
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    
    print("\n" + "="*60)
    print("  ESP32-CAM OPTICAL FLOW + LIDAR")
    print("  Visualiseur Protocole Binaire")
    print("="*60)
    print(f"Port série: {port}")
    print(f"Baudrate: {BAUDRATE}")
    print(f"Taille paquet: {PACKET_SIZE} bytes")
    print(f"Format: [0xAA 0x55][ts(4)][vx(2)][vy(2)][dist(2)][chk(1)]")
    print("="*60 + "\n")
    
    visualizer = OpticalFlowVisualizer(port, BAUDRATE)
    
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("\n✓ Arrêt demandé par l'utilisateur")
    except Exception as e:
        print(f"\n✗ Erreur: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
