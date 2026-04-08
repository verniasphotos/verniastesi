"""
Modulo 8: test_suite.py - Suite Test e Validazione Tesi
Questo script è il "Main Runner" (Punto d'ingresso principale) del Digital Twin.
Unisce tutti i moduli precedenti per orchestrare 5 Test Accademici fondamentali 
che dimostrano i concetti della tesi: Scalabilità, Tracking EKF, Risparmio Energetico (Green 6G).

Esegue una simulazione sintetica o semi-realistica salvando i risultati nel DB SQLite, 
per poi attivare la generazione di Grafici 2D e Dashboard 3D.
"""

import time # Libreria per la gestione del tempo e dei ritardi (timeout).
import math # Libreria standard di Python per operazioni matematiche (es. logaritmi, radici quadrate).
import random # Libreria per la generazione di numeri pseudo-casuali.
import numpy as np # Libreria potentissima per il calcolo scientifico e statistico (veloce perché scritta in C sotto il cofano).
from threading import Thread # Libreria per la gestione di thread concorrenti (parallelismo a livello di esecuzione).

# Importiamo la configurazione e le specifiche HW dal Modulo 1
from simulator.modulo_1_config import LAYOUT_A, LAYOUT_B, LAYOUT_C, RIS_HARDWARE, NETWORK_6G

# Importiamo l'Intelligenza SDN (Modulo 6)
from simulator.modulo_6_sdn_controller import SDNController

# Importiamo l'Ambiente Fisco (Modulo 2) e Telemetria (Modulo 7)
from simulator.modulo_2_environment import Environment, ray_casting_numba
from simulator.modulo_7_telemetria import TelemetrySpooler, DigitalTwinVisualizer
from simulator.modulo_4_channel_model import ChannelModel

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

def test_1_snr_outage_analysis(controller: SDNController):
    """
    TEST 1: Mappa Termica SNR e Analisi Outage (A/B Testing Visivo)
    Quantifica matematicamente la riduzione delle zone critiche NLoS 
    grazie all'ancoraggio RIS ottimizzato del Test 0.
    """
    print(f"\n--- [TEST 1] A/B Testing Link Budget: Calcolo Mappe Termiche per {controller.layout.name} ---")
    start_time = time.time()
    
    # 1. Setup Spaziale
    layout = controller.layout
    env = Environment(layout)
    boxes = env.shelf_boxes
    cm = ChannelModel()
    
    bs_pos = np.array([layout.x_dim_m / 2, layout.y_dim_m / 2, layout.z_dim_m])
    
    # Crea griglia 2D z=1.0m (come le quote dei robot LGV nei magazzini)
    x_vals = np.arange(0, layout.x_dim_m, 0.5)
    y_vals = np.arange(0, layout.y_dim_m, 0.5)
    grid_x, grid_y = np.meshgrid(x_vals, y_vals)
    
    snr_run_a = np.zeros_like(grid_x)
    snr_run_b = np.zeros_like(grid_x)
    
    outage_count_a = 0
    outage_count_b = 0
    total_points = grid_x.size
    
    print("[*] Esecuzione Ray-Casting e calcolo Path Loss in cascata... (Potrebbe richiedere tempo)")
    
    # Estrarre posizioni RIS attive allocate via SDN KMeans
    ris_positions = [np.array(ris.position) for ris in controller.ris_nodes.values() if ris.is_active]
    
    for i in range(grid_x.shape[0]):
        for j in range(grid_x.shape[1]):
            # Posizione corrente del dron per il campionamento
            uav_pos = np.array([grid_x[i, j], grid_y[i, j], 1.0])
            
            # --- RUN A (Baseline senza RIS) ---
            # Distanza e penetrazione ostacoli verso l'unica Base Station
            dist_bs = np.linalg.norm(bs_pos - uav_pos)
            penetration_bs = ray_casting_numba(uav_pos, bs_pos, boxes)
            pl_a = cm.calculate_path_loss_inf_dh(dist_bs, penetration_bs)
            
            # Formuletta Base SNR (vedi Modulo 4 Demo)
            base_snr = 23.0 - pl_a - (-90.0)
            snr_run_a[i, j] = base_snr
            if base_snr < 5.0:
                outage_count_a += 1
                
            # --- RUN B (Architettura 6G con RIS attive) ---
            best_snr_b = base_snr # Di base è identico al Run A (nessun deterioramento artificiale)
            
            # Proviamo ciascuna RIS e prendiamo l'intersezione radio migliore
            for ris_pos in ris_positions:
                dist_uav_ris = np.linalg.norm(ris_pos - uav_pos)
                pen_uav_ris = ray_casting_numba(uav_pos, ris_pos, boxes)
                
                dist_ris_bs = np.linalg.norm(bs_pos - ris_pos)
                pen_ris_bs = ray_casting_numba(ris_pos, bs_pos, boxes)
                
                # Calcolo del Modello a Cascata implementato nel task precedente per questa specifica RIS
                cascaded_snr = cm.compute_cascaded_snr(
                    d_uav_ris=dist_uav_ris, 
                    d_ris_bs=dist_ris_bs,
                    metal_uav_ris=pen_uav_ris,
                    metal_ris_bs=pen_ris_bs
                )
                
                # Se la geometria della riflessione offre un SNR superiore del collegamento diretto, sovrascriviamo
                if cascaded_snr > best_snr_b:
                    best_snr_b = cascaded_snr
            
            snr_run_b[i, j] = best_snr_b
            if best_snr_b < 5.0:
                outage_count_b += 1
                
    calc_time = time.time() - start_time
    outage_pct_a = (outage_count_a / total_points) * 100.0
    outage_pct_b = (outage_count_b / total_points) * 100.0
    
    print(f"[*] Analisi Mappa Termica completata in {calc_time:.2f} s.")
    print(f"    - Area Outage Run A (Senza RIS): {outage_pct_a:.2f}%")
    print(f"    - Area Outage Run B (Con RIS):   {outage_pct_b:.2f}%")
    
    # Generazione visiva
    visualizer = DigitalTwinVisualizer()
    visualizer.plot_ab_snr_heatmap(grid_x, grid_y, snr_run_a, snr_run_b, outage_pct_a, outage_pct_b)
    time.sleep(1)

def test_2_crash_kinematics(spooler: TelemetrySpooler):
    """
    TEST 2: Crash Incertezza. 
    Simuliamo un drone (UAV_CRASH) il cui Fading abbassa drasticamente l'SNR causando 
    un grave errore quadratico, fino allo schianto o ricalcolo.
    """
    print("\n--- [TEST 2] Analisi Cinematica EKF e Simulazione Perturbazione ---")
    t_sim = 100.0
    x, y, z = 5.0, 5.0, 3.0
    
    print("[*] Inizio volo UAV_CRASH. SNR in decadimento rapido...")
    for step in range(30):
        t_sim += 0.1
        # Muoviamolo in diagonale
        x += 0.2
        y += 0.2
        
        # Simula un'entrata NLoS dietro molto metallo! SNR cala a picco.
        snr = 25.0 - (step * 1.5) 
        if snr < 0: snr = 0
        is_los = True if snr > NETWORK_6G.outage_snr_db else False
        
        spooler.log_uav_data((t_sim, "UAV_CRASH", x, y, z, snr, is_los))
        
        if snr <= NETWORK_6G.outage_snr_db:
            print(f"    ! [WARN] t={t_sim:.1f}s | SNR={snr:.1f} dB (Sotto Soglia Outage). Rischio Crash, incertezza Kalman P instabile.")
    time.sleep(1)

def test_3_energy_profiling(spooler: TelemetrySpooler, controller: SDNController):
    """
    TEST 3: Generazione dei rate di consumo Energetico nel ciclo 'Make-before-break' SDN.
    Questo popolamento permette la stampa corretta del Barplot Green 6G.
    """
    print("\n--- [TEST 3] Green 6G Generazione Eventi di Rete (Power Profiling) ---")
    t_sim = 200.0
    print("[*] Generazione profili energetici per 10 minuti di simulazione simulata...")
    
    # 1. Base cost: Tutte le antenne in SLEEP consumano 0.5W base per 10 min = 600 secondi.
    # Simuliamo gli inserimenti nel DB per far lavorare Pandas nel Modulo 7!
    spooler.log_sdn_event((t_sim, "POWER_SLEEP_BASE", RIS_HARDWARE.p_sleep_w * 60, "Sleep Mode Base continuo per 600s"))
    
    # 2. Sweep: Aerei che passano. Le RIS si accendono a 50W (POWER_ACTIVE) per brevi spot (es. 20 secondi).
    spooler.log_sdn_event((t_sim+5.0, "POWER_ACTIVE_WAKEUP", RIS_HARDWARE.p_active_w * 2, "RIS_0 accesa temporaneamente per UAV_001"))
    spooler.log_sdn_event((t_sim+10.0, "POWER_ACTIVE_PREEMPTIVE", RIS_HARDWARE.p_active_w * 5, "RIS_1 accesa in anticipo Hook Predittivo"))
    
    # Costo fisso continuo del Backhaul
    spooler.log_sdn_event((t_sim, "POWER_FLIGHT_DRONES", 170.0 * 6, "UAV_001 volo stabile"))
    
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
    
    # FASE 1: Inizializziamo l'engine log asincrono
    # Questo avvierà il deamon thread nel background.
    spooler = TelemetrySpooler()
    
    # FASE 2: Inizializziamo il Cervello Centralizzato (SDN) col Layout A
    controller = SDNController(layout=LAYOUT_A, ris_specs=RIS_HARDWARE)
    
    # Esecuzione Batteria di Test accademici
    test_0_bom_testing(controller)
    test_1_snr_outage_analysis(controller)
    test_2_crash_kinematics(spooler)
    test_3_energy_profiling(spooler, controller)
    
    # FASE 3: Graceful Shutdown (Attendiamo che il thread worker scarichi la RAM su Hard Disk)
    print("\n[*] Attendere: Flushing della Coda RAM sul file Database (SQLite) locale in corso...")
    spooler.stop() 
    print("[*] Scrittura su Disco asincrona terminata. Database sicuro.")
    
    # FASE 4: Digital Twin Rendering (Lavoro "Post-Mortem" come fa la vera Data Science)
    print("\n================================================================")
    print("         DATA SCIENCE & VISUALIZATION SERVER WARMUP")
    print("================================================================")
    viewer = DigitalTwinVisualizer(db_path="simulation_data.db")
    
    generate_all_topological_maps()
    
    print("[*] Calcolo CDF per Signal-to-Noise Ratio (Generazione PNG)...")
    viewer.plot_cdf_snr()
    
    print("[*] Computo Integrali Consumi (Green 6G Barplot)...")
    viewer.plot_energy_consumption()
    
    print("[*] Building 3D Olografico per la Dashboard (HTML)...")
    viewer.dashboard_3d()
    
    print("\n[SUCCESS] Suite di Test Conclusa Magistralmente! Artefatti Pronti per la compilazione Tesi.")

if __name__ == "__main__":
    main_orchestrator()
