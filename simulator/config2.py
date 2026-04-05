"""
Modulo 1: config2.py - Core System, Hardware & Warehouse Specs
Funge da "Single Source of Truth" (Unica fonte di verità) per tutto il simulatore.
Definisce le costanti hardware e geometriche usate dagli altri moduli.
"""

from dataclasses import dataclass
from typing import Tuple

@dataclass
class UAVSpecs:
    """
    Specifiche hardware del drone (UAV).
    Questi dati servono per il calcolo cinetico (batteria e spostamento).
    """
    mass_kg: float = 1.2             # Massa del drone in kg
    p_hover_w: float = 150.0         # Potenza consumata da fermo in hovering (Watt)
    p_move_w: float = 170.0          # Potenza consumata in movimento (Watt)
    p_radio_w: float = 2.0           # Potenza consumata dal modulo radio 6G (Watt)
    max_angle_deg: float = 15.0      # Inclinazione massima consentita (pitch/roll) per evitare schianti


@dataclass
class RISSpecs:
    """
    Specifiche hardware delle Superfici Intelligenti (RIS).
    Servono all'SDN per attivare/disattivare i pannelli e per il channel_model.
    """
    p_sleep_w: float = 0.5           # Consumo minimo quando il pannello è spento (Sleep mode)
    p_active_w: float = 50.0         # Consumo quando il pannello irradia (Active mode)
    gain_db: float = 20.0            # Guadagno di amplificazione del segnale (Delta gain)
    noise_figure_db: float = 3.0     # Rumore termico introdotto dai componenti (F)


@dataclass
class NetworkSpecs:
    """
    Specifiche della Rete mobile 6G.
    """
    carrier_freq_ghz: float = 5.9    # Frequenza operativa (Banda U-NII-4 / 6G-ready)
    outage_snr_db: float = 5.0       # Sotto questo valore di SNR, il pacchetto viene perso
    dt_seconds: float = 0.1          # Delta-T del ciclo di simulazione (10 cicli al secondo)


@dataclass
class WarehouseBase:
    """
    Base per le misure standard di un capannone e degli scaffali.
    """
    shelf_x_m: float = 1.2           # Profondità dello scaffale
    shelf_y_m: float = 1.0           # Larghezza di un singolo modulo scaffale
    shelf_z_spacing_m: float = 0.6   # Distanza in altezza tra le mensole
    vna_width_m: float = 3.0         # Larghezza del corridoio Very Narrow Aisle
    wall_spacing_m: float = 2.5      # Distanza di rispetto (margine) tra le mura e gli scaffali
    penetration_loss_db_m: float = 15.0 # Attenuazione del segnale per ogni metro di metallo attraversato


@dataclass
class LayoutConfig(WarehouseBase):
    """
    Estensione delle misure base, aggiungendo le dimensioni totali del capannone.
    """
    name: str = "Generic"
    x_dim_m: float = 0.0             # Dimensione X totale (Larghezza)
    y_dim_m: float = 0.0             # Dimensione Y totale (Profondità)
    z_dim_m: float = 0.0             # Dimensione Z totale (Altezza)


# === ISTANZIAZIONE DEI 3 LAYOUT RICHIESTI === #

LAYOUT_A = LayoutConfig(
    name="Layout A (Piccolo)",
    x_dim_m=50.0,
    y_dim_m=40.0,
    z_dim_m=10.0
)

LAYOUT_B = LayoutConfig(
    name="Layout B (Medio)",
    x_dim_m=100.0,
    y_dim_m=100.0,
    z_dim_m=10.0
)

LAYOUT_C = LayoutConfig(
    name="Layout C (Grande)",
    x_dim_m=250.0,
    y_dim_m=140.0,
    z_dim_m=15.0
)

# Istanze fisiche pronte per essere importanti negli altri script
UAV_HARDWARE = UAVSpecs()
RIS_HARDWARE = RISSpecs()
NETWORK_6G = NetworkSpecs()
