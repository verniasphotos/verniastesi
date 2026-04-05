# ==============================================================================
# simulator/__init__.py
# Rende la cartella 'simulator' un package Python importabile.
# ==============================================================================
"""
Package principale del Simulatore Tracking Indoor 6G.

Questo package contiene tutti i moduli del Digital Twin:
    - config2: Costanti fisiche e specifiche hardware (Single Source of Truth)
    - environment: Motore fisico 3D e ray-casting
    - networking: IPC broker, shared memory e gRPC
    - channel_model: Modello di canale 3GPP
    - kinematics_ekf: Cinematica UAV e Filtro di Kalman Esteso
    - sdn_controller: Controller SDN e ottimizzazione RIS
    - telemetry: Visualizzazione Digital Twin e database SQLite
"""
