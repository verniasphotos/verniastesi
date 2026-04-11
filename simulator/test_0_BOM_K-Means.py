"""
Modulo 8: test_suite.py -> test_0_BOM_K-Means.py
Questo script esegue il Test 0: Bill of Materials (BOM) & K-Means Deployment
e genera le relative mappe topologiche.
"""

import time # libreria per misurare il tempo
import math # libreria matematica
import random # libreria per generare numeri casuali
import numpy as np # libreria per calcoli scientifici
from threading import Thread # libreria per multithreading

# Importiamo la configurazione e le specifiche HW dal Modulo 1
from simulator.modulo_1_config import LAYOUT_A, LAYOUT_B, LAYOUT_C, RIS_HARDWARE, NETWORK_6G

# Importiamo l'Intelligenza SDN (Modulo 6)
from simulator.modulo_6_sdn_controller import SDNController

# Importiamo l'Ambiente Fisco (Modulo 2) e Telemetria (Modulo 7)
from simulator.modulo_2_environment import Environment
from simulator.modulo_7_telemetria import DigitalTwinVisualizer

def test_0_bom_testing(controller: SDNController):
    """
    TEST 0: Bill of Materials (BOM) & K-Means Deployment
    Simula il piazzamento delle RIS per coprire i punti d'ombra (NLoS).
    """
    print("\n--- [TEST 0] Avvio Ottimizzazione Piazzamento RIS (K-Means + Greedy) ---")
    # Creiamo punti ciechi fittizi (NLoS) per il layout corrente
    nlos_points = np.array([
        [10.0, 5.0, 1.0], [12.0, 6.0, 1.5], [11.0, 5.5, 1.2], # Cluster 1
        [40.0, 35.0, 2.0], [42.0, 36.0, 1.8], [41.0, 35.5, 2.1] # Cluster 2
    ])
    num_ris_available = 4
    
    start_time = time.time()
    deployed = controller.deploy_ris_kmeans_greedy(nlos_points, num_ris_available)
    calc_time = (time.time() - start_time) * 1000  # ms
    
    print(f"[*] Trovati N={len(deployed)} Punti di Ancoraggio RIS Ottimali in {calc_time:.2f} ms.")
    for i, pos in enumerate(deployed):
        print(f"    - RIS_ID_{i} ancorata al muro in coordinate: X={pos[0]:.1f}, Y={pos[1]:.1f}, Z={pos[2]:.1f}")
    time.sleep(1)

def generate_all_topological_maps():
    """
    Genera le planimetrie (Mappe Topologiche 2D) per tutti i layout del capannone.
    """
    print("\n--- [TEST TOPOLOGIA] Generazione Mappe Topologiche 2D ---")
    layouts = [LAYOUT_A, LAYOUT_B, LAYOUT_C]
    # Determiniamo il numero di RIS a seconda del layout:
    # A (50x40): 4 Soffitto, 0 Parete (verificato)
    # B (100x100): 8 Soffitto, 4 Parete
    # C (250x140): 16 Soffitto, 8 Parete
    ris_counts = {
        "Layout A (Piccolo)": {"soffitto": 4, "parete": 0},
        "Layout B (Medio)": {"soffitto": 8, "parete": 4},
        "Layout C (Grande)": {"soffitto": 16, "parete": 8}
    }
    
    viewer = DigitalTwinVisualizer(db_path="simulation_data.db")
    
    for layout in layouts:
        env = Environment(layout=layout)
        centers = env.shelf_centers
        
        # Posizionamenti di base
        nodes = {
            "SuperServer": (0.0, 0.0, 1.0),
            "BaseRicarica": (layout.x_dim_m / 2, 0.0, 1.0),
            "BS": (layout.x_dim_m / 2, layout.y_dim_m / 2, layout.z_dim_m)
        }
        
        # --- Ottimizzazione Intelligente SDN (K-Means + Greedy) ---
        total_ris_budget = ris_counts[layout.name]["soffitto"] + ris_counts[layout.name]["parete"]
        
        if total_ris_budget > 0 and len(centers) > 0:
            # 1. Istanziamo temporaneamente l'SDN Controller
            controller = SDNController(layout=layout, ris_specs=RIS_HARDWARE)
            
            # 2. Sperimentiamo simulando delle letture fisiche di zone d'ombra NLoS
            # Simuliamo che circa il 20% degli scaffali causi estrema perdita di segnale.
            rng = np.random.default_rng(42)  # Seed fisso per la riproducibilità dei grafici della tesi
            samples = max(1, int(len(centers) * 0.20))
            indices = rng.choice(len(centers), size=samples, replace=False)
            nlos_simulated = centers[indices]
            
            # 3. Lanciamo l'Addestramento K-Means e l'ancoraggio Greedy!
            print(f"    -> [SDN Opt] Elaborazione algoritmi AI su {samples} punti d'ombra (NLoS)...")
            deployed_positions = controller.deploy_ris_kmeans_greedy(nlos_simulated, total_ris_budget)
            
            # 4. Aggiorniamo i nodi classificandoli per stampa (Soffitto o Parete)
            for opt_idx, pos in enumerate(deployed_positions):
                x, y, z = pos
                if z >= float(layout.z_dim_m * 0.95):
                    # È a soffitto!
                    nodes[f"RIS_Soffitto_{opt_idx}"] = pos
                else:
                    nodes[f"RIS_Parete_{opt_idx}"] = pos
                
        print(f"[*] Generando mappa 2D (SDN-Aided) per {layout.name}...")
        viewer.plot_topological_map(layout, centers, nodes)


def main_orchestrator():
    print("================================================================")
    print("      UNIVERSITÀ - SIMULATORE 6G TRACKING DIGITAL TWIN")
    print("      MAIN RUNNER - EXECUTION SUITE PER TESI DI LAUREA")
    print("================================================================")
    
    # Inizializziamo il Cervello Centralizzato (SDN) col Layout A
    controller = SDNController(layout=LAYOUT_A, ris_specs=RIS_HARDWARE)
    
    # Esecuzione Batteria di Test accademici (Solo Test 0 come richiesto)
    test_0_bom_testing(controller)
    
    # Digital Twin Rendering per visualizzare il piazzamento
    print("\n================================================================")
    print("         DATA SCIENCE & VISUALIZATION SERVER WARMUP")
    print("================================================================")
    
    generate_all_topological_maps()
    
    print("\n[SUCCESS] Test 0 (BOM & K-Means) Concluso Magistralmente! Artefatti Pronti.")

if __name__ == "__main__":
    main_orchestrator()

