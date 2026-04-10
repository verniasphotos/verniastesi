import sqlite3 #   Libreria nativa Python per la gestione di database relazionali leggeri (SQL).
import pandas as pd # Libreria fondamentale per la manipolazione e l'analisi di dati tabellari (DataFrames).
import matplotlib.pyplot as plt # Libreria di plotting 2D standard per la visualizzazione scientifica.
import matplotlib.patches as patches # Per disegnare gli scaffali 2D
import seaborn as sns # Libreria di visualizzazione statistica basata su Matplotlib, ottimizzata per grafici statistici complessi.
import plotly.express as px # Libreria di visualizzazione interattiva ad alto livello, basata su Plotly.js.
import plotly.graph_objects as go # Modulo di basso livello di Plotly per la creazione di figure complesse e personalizzate.
import threading # Libreria per la gestione di thread concorrenti (parallelismo a livello di esecuzione).
import queue # Libreria per la gestione di code thread-safe (Producer-Consumer).
import time # Libreria per la gestione del tempo e dei ritardi (timeout).
from typing import List, Dict, Any # Modulo per l'annotazione dei tipi (Type Hinting).

# =============================================================================
# MODULO 7: TELEMETRY & DIGITAL TWIN VISUALIZATION
# =============================================================================
# Questo modulo gestisce il "Data Sink" dell'intera simulazione 6G. 
# Si divide in due core components:
# 1. TelemetrySpooler: Un'interfaccia asincrona per il salvataggio dei log su DB.
#    Il suo scopo architetturale è bypassare l'I/O Bound dell'hard disk.
# 2. DigitalTwinVisualizer: Strumenti di Business Intelligence e Data Science
#    per l'estrapolazione delle metriche di rete (CDF, Heatmaps) necessarie per 
#    l'analisi accademica e la compilazione della Tesi di Laurea.
# =============================================================================

class TelemetrySpooler:
    """
    Gestisce l'inserimento massivo (Bulk Insert) dei dati in database SQLite3.
    Implementa un pattern 'Producer-Consumer' tramite code thread-safe (queue.Queue)
    per scaricare in modo asincrono la mole dati generata dalla simulazione.
    """
    def __init__(self, db_path: str = "simulation_data.db", batch_size: int = 100):
        # Percorso fisico del file database spaziale in locale
        self.db_path = db_path
        # batch_size rappresenta il limite di soglia (trigger massivo).
        # Valori più ampi stressano la RAM, valori troppo bassi stressano il Disco.
        self.batch_size = batch_size
        
        # Coda FIFO integrata e thread-safe. Essendo in RAM, la latenza di
        # scrittura (metodo .put()) per le threads che producono dati è quasi nulla.
        self.queue = queue.Queue()
        self.running = True
        
        # 1. Crea l'architettura relazionale del DBMS se non esiste
        self._init_db()
        
        # 2. Istanziazione e Avvio del Daemon Thread Consumer.
        # Usa daemon=True affinché il thread worker muoia automaticamente se
        # il processo MAIN principale viene forzatamente interrotto.
        self.worker_thread = threading.Thread(target=self._worker, daemon=True)
        self.worker_thread.start()

    def _init_db(self):
        """
        Crea le tabelle fisiche (Schema Relazionale) all'interno del DB SQLite3.
        Viene eseguito rigorosamente nel costruttore prima del primo campionamento.
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Creazione Tabella: uav_telemetry
        # Dominio: Registrazione ad alta frequenza della Traiettoria Spaziale e RF.
        # Include i KPI principali (Key Performance Indicators) per la tesi: X, Y, Z ed SNR.
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS uav_telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL,       -- Timestamp di sistema o tempo di simulazione (T_SIM)
            uav_id TEXT,          -- Univoco del Drone (es. "UAV_001") per tracciamento Multi-Agent
            x REAL,               -- Coordinata spaziale Cartesiana (Metri)
            y REAL,               -- Coordinata spaziale Cartesiana (Metri)
            z REAL,               -- Altitudine o asse Z (Metri)
            snr REAL,             -- Signal-To-Noise Ratio percepito dall'Antenna Rx (dB)
            is_los BOOLEAN        -- Flag logico True (Line-Of-Sight) / False (Non-Line-Of-Sight)
        )
        ''')
        
        # Creazione Tabella: sdn_events
        # Dominio: Registrazione asincrona a bassa frequenza delle direttive SDN.
        # Impiegato per calcolare la metrica "Green 6G" tracciando l'attivazione (Wake-Up) delle RIS.
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS sdn_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL,       -- Momento esatto del trigger generato dal Controller
            event_type TEXT,      -- Classificazione (es. "POWER_SLEEP", "POWER_ACTIVE")
            power_w REAL,         -- Modulo della potenza scalata dinamicamente (Watts)
            description TEXT      -- Metadata per log e auditing manuale
        )
        ''')
        conn.commit()
        conn.close()

    def log_uav_data(self, data: tuple):
        """
        Interfaccia esposta al Modulo 5 (EKF e Dinamica) e Modulo 4 (Canale RF).
        Esegue esclusivamente un push 'fire-and-forget' verso la coda in RAM.
        
        :param data: tupla contenente ordinatamente -> 
                     (timestamp, uav_id, x, y, z, snr, is_los)
        """
        self.queue.put(('uav_telemetry', data))

    def log_sdn_event(self, data: tuple):
        """
        Interfaccia esposta al Modulo 6 (SDN Network Controller).
        Aggrega le direttive di rete prima che vadano perse.
        
        :param data: tupla contenente ordinatamente -> 
                     (timestamp, event_type, power_w, description)
        """
        self.queue.put(('sdn_events', data))

    def _worker(self):
        """
        CONSUMER TASK: Funzione Threaded che opera perpetuamente in Backgroud.
        Applica l'euristica di "Batching". Non effettua commit singoli, ma attende
        il superamento di un limite (self.batch_size) limitando i blocchi HDD.
        """
        # Connessione creata specificatamente per il perimetro di questa Thread
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Due liste dinamiche Python impiegate come Caching di Buffer prima dell'Upload.
        buffer_uav = []
        buffer_sdn = []
        
        # Mantiene in vita il loop finché la bandiera running è vera o vi sono pacchetti residui
        while self.running or not self.queue.empty():
            try:
                # Polling asincrono con attesa controllata (0.5 secondi). 
                # Evita un CPU-Spin (consumo anomalo di CPU 100%) quando la coda è vuota.
                item = self.queue.get(timeout=0.5)
                table, data = item
                
                # De-multiplexor basato sul nome della tabella passata dalla tupla base
                if table == 'uav_telemetry':
                    buffer_uav.append(data)
                elif table == 'sdn_events':
                    buffer_sdn.append(data)
                    
                # Segnala alla coda l'avvenuta presa in carico.
                self.queue.task_done()
            except queue.Empty:
                pass # Timeout innocuo raggiunto. Rilancia il polling al ciclo intatto.
            
            # =============== OPERAZIONI MASSIVE (BULK I/O) =============== #
            # Quando il caching parziale in RAM supera il Batch Size, si svota forzatamente
            # su persistenza disco solido attraverso il costrutto nativo `executemany()`.
            
            if len(buffer_uav) >= self.batch_size:
                cursor.executemany('''
                INSERT INTO uav_telemetry (timestamp, uav_id, x, y, z, snr, is_los) 
                VALUES (?, ?, ?, ?, ?, ?, ?)
                ''', buffer_uav)
                conn.commit()
                buffer_uav.clear()
                
            if len(buffer_sdn) >= self.batch_size:
                cursor.executemany('''
                INSERT INTO sdn_events (timestamp, event_type, power_w, description) 
                VALUES (?, ?, ?, ?)
                ''', buffer_sdn)
                conn.commit()
                buffer_sdn.clear()
        
        # GESTIONE EPILOGO (CLEANUP): Svuota l'ultimo moncone di array rimasto invariato 
        # (potenzialmente inferiore a batch_size) prima di terminare brutalmente la Thread.
        if buffer_uav:
            cursor.executemany('INSERT INTO uav_telemetry (timestamp, uav_id, x, y, z, snr, is_los) VALUES (?, ?, ?, ?, ?, ?, ?)', buffer_uav)
        if buffer_sdn:
            cursor.executemany('INSERT INTO sdn_events (timestamp, event_type, power_w, description) VALUES (?, ?, ?, ?)', buffer_sdn)
        
        conn.commit()
        conn.close()

    def stop(self):
        """
        Comando Master per disassare dolcemente (Graceful Shutdown) il logger. 
        Impedisce la chiusura distruttiva richiamando il Join Thread, imponendo al
        Programma Main di attendere lo svuotamento chimico dei flussi in queue prima di morire.
        """
        self.running = False
        self.worker_thread.join()


class DigitalTwinVisualizer:
    """
    Classe adibita all'analisi Dati, al Parsing e all'Esportazione Grafici (Digital Twin DataLab).
    Opera interamente "Post-Process", ovvero legge un DB inerte e cristallizzato
    al termine dell'esecuzione massiva, convertendo flussi sparsi in Metriche Scientifiche.
    """
    def __init__(self, db_path: str = "simulation_data.db"):
        self.db_path = db_path

    def _load_telemetry(self) -> pd.DataFrame:
        """
        Esegue il fetch globale della tabella spaziale riversando milioni di righe SQL
        direttamente all'interno di un oggetto DataFrame (RAM strutturata ed indicizzabile).
        L'impiego nativo di Pandas qui supera come performances la libreria csv di base python.
        """
        conn = sqlite3.connect(self.db_path)
        df = pd.read_sql_query("SELECT * FROM uav_telemetry", conn)
        conn.close()
        return df

    def plot_cdf_snr(self):
        """
        [Requisito Tesi]: Genera il grafico a gradini della Distribuzione Cumulativa (CDF).
        
        La Cumulative Distribution Function P(X <= x) è vitale nel piano telecomunicazioni 
        perché converte la lettura frammentata dei Sample nel tempo in Probabilità Outage. 
        Evidenzia matematicamente nel diagramma per quanto % di copertura simulativa
        il drone si è tenuto sotto ad un SNR soglia (Outage Threshold).
        """
        df = self._load_telemetry()
        if df.empty:
            print("Nessun dato presente nel DB per il plot CDF.")
            return

        # Rendiamo le label comprensibili per la tesi
        df_plot = df.copy()
        df_plot["Tipologia di Propagazione"] = df_plot["is_los"].apply(
            lambda x: "LOS (Visione Diretta)" if x in [1, True, '1', 'true'] else "NLoS (Zona d'Ombra)"
        )

        # Styling accademico/industriale
        sns.set_theme(style="whitegrid")
        plt.figure(figsize=(9, 6))
        
        # Plot ECDF con linee visibili
        ecdf = sns.ecdfplot(
            data=df_plot, 
            x="snr", 
            hue="Tipologia di Propagazione", 
            linewidth=2.5,
            palette=["#1f77b4", "#ff7f0e"] # Blu e Arancione standard
        )
        
        # 1) Area Rossa: Spiega "A cosa serve!"
        plt.axvspan(df_plot['snr'].min() - 5, 5.0, color='red', alpha=0.15)
        
        # 2) Marker Soglia Critica
        plt.axvline(x=5.0, color='darkred', linestyle='--', linewidth=2, label='Soglia Limite Disconnessione (5 dB)')
        
        # 3) Testi per aiutare l'osservatore
        plt.text(0.0, 0.5, 'Area Critica\nRischio Crash', color='darkred', weight='bold', 
                 ha='center', va='center', bbox=dict(facecolor='white', alpha=0.7, edgecolor='red'))
                 
        plt.title("Metriche di Affidabilità: CDF del Rapporto Segnale Rumore (SNR)", fontsize=14, fontweight='bold', pad=15)
        plt.xlabel("SNR Misurato dai Droni (dB) - [Più è a destra, meglio è]", fontsize=12)
        plt.ylabel("Probabilità Cumulativa P(X ≤ Px)", fontsize=12)
        
        plt.xlim(df_plot['snr'].min() - 2, df_plot['snr'].max() + 5)
        plt.ylim(-0.02, 1.05)
        
        plt.legend(loc="lower right", shadow=True, frameon=True)
        plt.tight_layout()
        plt.savefig("cdf_snr_plot.png", dpi=200)
        print("Salvato: cdf_snr_plot.png (Migliorato con Area Outage)")
        plt.close()

    def plot_energy_consumption(self):
        """
        [Requisito Tesi]: Genera un grafico basato puramente sul Control-Plane Network (SDN).
        
        Somma integrativamente gli scaloni energetici richiesti al sistema d'antenna e hardware,
        sottolineando il paradigma "Green 6G" del tuo paper: ovvero mostrare il divario tra 
        un modulo Always-On passivo, in contrasto al modulo Ris che viene "Svegliato" predittivamente (Sleep-to-Active mode).
        """
        conn = sqlite3.connect(self.db_path)
        # Query string filtrata per precaricare solo eventi energetici
        df = pd.read_sql_query("SELECT * FROM sdn_events WHERE event_type LIKE 'POWER_%'", conn)
        conn.close()
        
        if df.empty:
            print("Nessun salvataggio energetico trovato all'interno del DB.")
            return
            
        plt.figure(figsize=(8, 5))
        # Crea un barplot dove l'aggregatore è la Somma Totale (Integrale del consumo)
        sns.barplot(data=df, x="event_type", y="power_w", estimator=sum, errorbar=None)
        plt.title("Consumo Energetico Cumulativo Sistema Aereo/Terrestre (Joules)")
        plt.ylabel("Potenza Energetica (J)")
        plt.xlabel("Categoria Sorgente / Tipologia Evento")
        plt.grid(True)
        plt.savefig("energy_plot.png")
        print("Salvato: energy_plot.png")
        plt.close()

    def dashboard_3d(self):
        """
        [Requisito Tesi - Digital Twin]: View-Space e rendering olografico navigabile del magazzino.
        
        Integra Plotly Graph Objects (go). Al contrario di lib grafiche statiche bidimensionali, Plotly 
        incapsula lo script javascript Web-GL all'interno di un canvas HTML. Il relatore aprirà il
        browser visualizzando un gemello tridimensionale. Nel Digital Twin i Vettori (X,y,z) non indicano solo la traiettoria, ma
        l'heatmap del colore segna un delta RF Mapping (Mappatura topologia Radio frequenza in movimento).
        """
        df = self._load_telemetry()
        if df.empty:
            print("Nessun dato per animare la Dashboard 3D.")
            return

        # 1. Plotly px per stabilire l'asse rigido vettoriale
        fig = px.line_3d(df, x="x", y="y", z="z", color="uav_id", title="Digital Twin - Simulazione Kinematica EKF + 6G Raycasting")
        
        # 2. Add-On di Scatter Plotly (Nuvola Termica / Heatmap del segnale RF)
        scatter = go.Scatter3d(
            x=df['x'], y=df['y'], z=df['z'],
            mode='markers',
            marker=dict(
                size=4,
                color=df['snr'],           # Associa il colore del gradiente al valore reale d'intensità Fading.
                colorscale='Inferno',      # Pattern Dark-To-Light molto adatti per differenziare Hotspot di Copertura.
                colorbar=dict(title="SNR (dB)"),
                opacity=0.9
            ),
            name="Profilo Segnale Fading (SNR)"
        )
        fig.add_trace(scatter)
        
        # Stampa su disco di un documento formattato HTML autonomo
        fig.write_html("digital_twin_dashboard.html")
        print("Dashboard generata magistralmente! Apri il file 'digital_twin_dashboard.html' sul browser Web.")

    def plot_topological_map(self, layout, shelf_centers, nodes_positions):
        """
        [Requisito Tesi]: Genera e salva la mappa topologica 2D del magazzino con i dispositivi 6G.
        """
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Disegno perimetro
        ax.plot([0, layout.x_dim_m, layout.x_dim_m, 0, 0], 
                [0, 0, layout.y_dim_m, layout.y_dim_m, 0], 
                color="black", linewidth=2.0)
        
        # Estrai coordinate uniche per gli scaffali (rimuoviamo Z)
        shelf_xy = set()
        for cx, cy, cz in shelf_centers:
            shelf_xy.add((cx, cy))
            
        hx = layout.shelf_x_m / 2
        hy = layout.shelf_y_m / 2
        
        for cx, cy in shelf_xy:
            rect = patches.Rectangle((cx - hx, cy - hy), layout.shelf_x_m, layout.shelf_y_m, 
                                     linewidth=1, edgecolor='lightgray', facecolor='#E0E0E0')
            ax.add_patch(rect)
            
        # Nodi 6G
        # Legenda base
        legend_elements = [
            patches.Patch(facecolor='#E0E0E0', edgecolor='lightgray', label='Scaffali Metallici'),
            plt.Line2D([0], [0], marker='^', color='w', label='Base Station (BS)', markerfacecolor='yellow', markersize=10, markeredgecolor='black'),
            plt.Line2D([0], [0], marker='s', color='w', label='RIS Parete', markerfacecolor='#00BFFF', markersize=10, markeredgecolor='black'),
            plt.Line2D([0], [0], marker='o', color='w', label='RIS Soffitto', markerfacecolor='#00C800', markersize=10, markeredgecolor='black'),
            plt.Line2D([0], [0], marker='*', color='w', label='Super Server', markerfacecolor='gold', markersize=15, markeredgecolor='black'),
            plt.Line2D([0], [0], marker='P', color='w', label='Base Ricarica Droni', markerfacecolor='magenta', markersize=12, markeredgecolor='black')
        ]
        
        # Plottiamo i nodi
        for name, pos in nodes_positions.items():
            x, y, z = pos
            if name.startswith("BS"):
                ax.plot(x, y, marker='^', color='yellow', markersize=10, markeredgecolor='black')
            elif name.startswith("RIS_Parete"):
                ax.plot(x, y, marker='s', color='#00BFFF', markersize=10, markeredgecolor='black')
            elif name.startswith("RIS_Soffitto"):
                ax.plot(x, y, marker='o', color='#00C800', markersize=10, markeredgecolor='black')
            elif name.startswith("SuperServer"):
                ax.plot(x, y, marker='*', color='gold', markersize=15, markeredgecolor='black')
            elif name.startswith("BaseRicarica"):
                ax.plot(x, y, marker='P', color='magenta', markersize=12, markeredgecolor='black')
                
        # Recap Box Text
        ris_wall_count = sum(1 for k in nodes_positions if k.startswith("RIS_Parete"))
        ris_ceil_count = sum(1 for k in nodes_positions if k.startswith("RIS_Soffitto"))
        tot_components = 1 + 1 + 1 + ris_wall_count + ris_ceil_count
        
        import math
        area_mq = layout.x_dim_m * layout.y_dim_m
        
        # Calcolo SNR Medio Teorico per la variante Layout (A, B o C)
        # Formula euristica: Path Loss aumenta con la distanza, i RIS recuperano in guadagno
        distanza_media_bs = (layout.x_dim_m + layout.y_dim_m) / 4.0
        path_loss_db = 10 * 2.5 * math.log10(max(1.0, distanza_media_bs))
        guadagno_ris_db = (ris_wall_count + ris_ceil_count) * 0.75
        snr_medio_calc = 30.0 - path_loss_db + guadagno_ris_db
        snr_medio_calc = max(5.0, min(snr_medio_calc, 40.0)) # limite min/max
        
        # Numero consigliato di droni per limitare collisioni e ottimizzare la copertura
        max_droni = max(3, int(area_mq / 60))
        
        info_text = (
            "RECAP MAGAZZINO\n"
            "==============================\n\n"
            "[Dimensioni Magazzino]\n"
            f" - Lunghezza: {layout.x_dim_m:.1f} m\n"
            f" - Larghezza: {layout.y_dim_m:.1f} m\n"
            f" - Altezza:   {layout.z_dim_m:.1f} m\n"
            f" - Area:      {area_mq:,.0f} mq\n\n"
            "[Infrastruttura di Rete]\n"
            " - Super Server:        1\n"
            " - Base Ricarica Droni: 1\n"
            " - Base Station (BS):   1\n"
            f" - Pannelli RIS Parete: {ris_wall_count}\n"
            f" - Pannelli RIS Soffitto:{ris_ceil_count}\n"
            "------------------------------\n"
            f">> TOTALE COMPONENTI:   {tot_components}\n"
            "------------------------------"
        )
        
        props = dict(boxstyle='round', facecolor='aliceblue', alpha=0.8, edgecolor='lightslategray')
        ax.text(1.05, 0.95, info_text, transform=ax.transAxes, fontsize=8.5,
                verticalalignment='top', bbox=props, fontfamily='monospace')
                
        ax.legend(handles=legend_elements, loc='upper center', title='Legenda Simboli', bbox_to_anchor=(0.5, -0.12), ncol=3)
        
        ax.set_title(f"Mappa Topologica: Nodi 6G nel Magazzino\n{layout.name} ({int(layout.x_dim_m)}x{int(layout.y_dim_m)}m)", fontweight='bold')
        ax.set_xlabel("Lunghezza X (m)")
        ax.set_ylabel("Larghezza Y (m)")
        ax.set_xlim(-10, layout.x_dim_m + 10)
        ax.set_ylim(-10, layout.y_dim_m + 10)
        ax.grid(True, linestyle='--', alpha=0.3)
        ax.set_aspect('equal', adjustable='box')
        
        # Layout e salvataggio
        plt.tight_layout()
        filename = f"topological_map_layout_{layout.name.split(' ')[1].replace('(','').replace(')','').lower()}.png"
        if layout.name.startswith("Layout A"):
            filename = "topological_map_layout_a.png"
        elif layout.name.startswith("Layout B"):
            filename = "topological_map_layout_b.png"
        elif layout.name.startswith("Layout C"):
            filename = "topological_map_layout_c.png"
            
        plt.savefig(filename, bbox_inches='tight', dpi=150)
        print(f"Salvato: {filename}")
        plt.close()

    def plot_ab_snr_heatmap(self, grid_x, grid_y, snr_run_a, snr_run_b, outage_a_pct, outage_b_pct):
        """
        [TEST 1] Genera heatmap affiancate per confrontare l'SNR prima e dopo l'intervento RIS.
        Le griglie (grid_x, grid_y, snr_run_a, snr_run_b) devono essere array 2D generati con numpy.meshgrid.
        I valori outage_x_pct sono float calcolati esternamente e inseriti nei titoli.
        """
        from matplotlib.colors import ListedColormap, BoundaryNorm
        
        # Mapping Mappatura Termica (Rosso, Giallo, Verde/Blu)
        # Sotto 5: #d73027 (Rosso Outage)
        # Tra 5 e 20: #fee08b (Giallo Accettabile)
        # Sopra 20: #1a9850 (Verde Ottimo)
        cmap = ListedColormap(['#d73027', '#fee08b', '#1a9850'])
        bounds = [-100, 5, 20, 100]
        norm = BoundaryNorm(bounds, cmap.N)
        
        # Creiamo la figura con subplot in orizzontale
        fig, axes = plt.subplots(1, 2, figsize=(16, 7), sharey=True)
        
        # Plot Run A (Senza RIS)
        c0 = axes[0].pcolormesh(grid_x, grid_y, snr_run_a, cmap=cmap, norm=norm, shading='auto')
        axes[0].set_title(f"Baseline - Run A (Senza RIS)\nOutage (Area Rossa): {outage_a_pct:.2f}%", fontweight='bold', pad=10)
        axes[0].set_xlabel("Lunghezza X (m)")
        axes[0].set_ylabel("Larghezza Y (m)")
        axes[0].set_aspect('equal')
        
        # Plot Run B (Con RIS)
        c1 = axes[1].pcolormesh(grid_x, grid_y, snr_run_b, cmap=cmap, norm=norm, shading='auto')
        axes[1].set_title(f"Architettura 6G - Run B (Con RIS)\nOutage (Area Rossa): {outage_b_pct:.2f}%", fontweight='bold', pad=10)
        axes[1].set_xlabel("Lunghezza X (m)")
        axes[1].set_aspect('equal')
        
        # Colorbar condivisa
        fig.subplots_adjust(right=0.9)
        cbar_ax = fig.add_axes([0.92, 0.15, 0.02, 0.7])
        cbar = fig.colorbar(c1, cax=cbar_ax, ticks=[0, 12.5, 60])
        cbar.ax.set_yticklabels(['< 5 dB\n(Outage)', '5 - 20 dB\n(Accettabile)', '> 20 dB\n(Ottimo)'])
        cbar.set_label('Livello Segnale SNR', fontweight='bold', labelpad=15)
        
        # Titolo superore
        plt.suptitle("TEST 1 - A/B Testing: Impatto delle Superfici Intelligenti (RIS) sul Budget di Collegamento", fontsize=16, fontweight='bold')
        
        plt.savefig("test1_snr_heatmap.png", bbox_inches='tight', dpi=200)
        print("Salvato: test1_snr_heatmap.png")
        plt.close()
