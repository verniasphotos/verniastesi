# Piano di Implementazione Dettagliato - Simulatore Tracking Indoor 6G

Questo documento rappresenta il piano di sviluppo formale basato sul PRD. Il piano è strutturato con la massima granularità, esplodendo ciascun modulo nei propri requisiti funzionali e linee di codice.
*Checklist da spuntare durante l'avanzamento dei lavori.*

Fai semppre il codice in modo modulare, senza fare file monolitici, rispettando le best practice della programmazione.

---

## Modulo 1: `config2.py` - Core System, Hardware & Warehouse Specs
- [ ] **1.1. Inizializzazione:** Creare `config2.py` e predisporre gli import principali (`dataclasses`, `typing`).
- [ ] **1.2. UAV Hardware Data:** Implementare `@dataclass UAVSpecs` includendo i parametri fisici: massa (1.2), p_hover (150W), p_move (170W), p_radio (2W), max_angle (15°).
- [ ] **1.3. RIS Hardware Data:** Implementare `@dataclass RISSpecs` includendo p_sleep (0.5W), p_active (50.0W), gain_factor, noise_figure.
- [ ] **1.4. Network 6G Data:** Implementare `@dataclass NetworkSpecs` includendo carrier_freq (5.9 GHz) e outage_snr (5 dB).
- [ ] **1.5. Warehouse Base Parameters:** Implementare `@dataclass WarehouseBase` contenente `shelf_x=1.2`, `shelf_y=1.0`, `shelf_z_spacing=0.6`, `vna_width=3.0`, `wall_spacing=2.5`, `penetration_loss=15.0`.
- [ ] **1.6. Layout Definitions:** Creare le configurazioni per i layout `LayoutA` (50x40x10m), `LayoutB` (100x100x10m), `LayoutC` (250x140x15m).
- [ ] **📌 TASK SPIEGAZIONE:** Ti spiegherò riga per riga a cosa servono i Type Hints e il pattern `dataclass`. Ti darò i comandi per installare l'estensione **SQLite Viewer** per farti trovare pronto per i moduli successivi.

## Modulo 2: `environment.py` - Physics Engine (Mondo Fisico Virtuale)
- [x] **2.1. Inizializzazione:** Creare `environment.py`. Importare `numpy`, `scipy.spatial.KDTree` e `numba` (`@njit`).
- [x] **2.2. Grid Generator Matrix:** Creare una funzione che riceve un layout da `config2.py` e restituisce una lista di coordinate centrali $(X, Y, Z)$ per ogni modulo di scaffale presente nel capannone.
- [x] **2.3. Spatial Indexing con KDTree:** Creare un oggetto spaziale per inserire tutte le coordinate generate allo step 2.2 all'interno di un `scipy.spatial.KDTree`.
- [x] **2.4. JIT Ray-Casting (Numba):** Scrivere una funzione pura con `@njit(nogil=True)` che presi due vettori 3D (Drone e Antenna) calcola matematicamente le intersezioni della retta con gli ostacoli (per determinare LoS o calcolare i metri di penetrazione in NLoS).
- [x] **2.5. Collision Check:** Scrivere la funzione `validate_clearance()` che verifichi lo spessore dell'UAV con i margini (± 1.5 metri). Deve sollevare un'eccezione se c'è scontro.
- [x] **📌 TASK SPIEGAZIONE:** Spiegazione visiva e intuitiva di cos'è un KDTree per la ricerca di vicinanza e cosa significa che Numba aggira il GIL in C.

## Modulo 3: `networking.py` - IPC Broker & Protocol Stack
- [x] **3.1. Inizializzazione:** Creare `networking.py`. Importare `multiprocessing.shared_memory`, `time`, e i moduli `grpc`.
- [x] **3.2. Shared Memory Allocator:** Creare classe per allocare due blocchi di memoria Python a basso livello che faranno da "Buffer" tra l'UAV in volo e la logica SDNN. I dati trasmessi saranno array `numpy` formattati per simulare coordinate e log.
- [x] **3.3. Simulation Clock:** Inserire un gestore del tempo deterministico per avanzamenti rigorosi a cicli di $dt = 0.1$ secondi.
- [x] **3.4. gRPC Interface Mock:** Impostare l'impalcatura di base del server gRPC per simulare l'ingaggio backhaul ad alta latenza.
- [x] **📌 TASK SPIEGAZIONE:** Traduzione analogica per spiegare il Global Interpreter Lock bloccante di Python contro il Multiprocessing via RAM. Primo utilizzo pratico di **SQLite Viewer**: generiamo alcune pseudo-comunicazioni di rete e ti guiderò a estrarre la tabella Logs dal tuo DB ed osservare la latenza.

## Modulo 4: `channel_model.py` - Modello di Canale 3GPP & RIS Physics
- [x] **4.1. Inizializzazione:** Creare `channel_model.py`.
- [x] **4.2. Algoritmo Path Loss InF-DH:** Tradurre le direttive del TR 38.901 3GPP. Costruire la funzione che determina la perdita di propagazione base sommando poi `distanza_nel_metallo * 15.0 dB`.
- [x] **4.3. Algoritmo RIS Amp:** Immettere nel codice la logica che somma il guadagno passivo al bilancio di collegamento simulando anche il rumore iniettato dal circuito attivo.
- [x] **4.4. Beam Misalignment:** Sviluppare funzione trigonometrica per diminuire l'SNR ricevuto se i vettori pitch/roll superano un certo delta di allineamento dal picco del fascio dell'antenna.
- [x] **📌 TASK SPIEGAZIONE:** Spiegazione ingegneristica semplice su Fading ed RSSI. Hands-On sul database: memorizzeremo campioni SNR generati per simulazione statica e ti farò usare la barra di ordinamento del SQLite Viewer su VS Code per trovare quali posizioni del magazzino hanno SNR inferiore a 5.0 dB (l'outage citato nel PRD).

## Modulo 5: `kinematics_ekf.py` - UAV Dynamics & Tracking Engine
- [x] **5.1. Inizializzazione:** Creare `kinematics_ekf.py`. Importare `filterpy` e le librerie matematiche.
- [x] **5.2. UAV Physics Engine:** Scrivere il modello di stato cinetico. Equazioni differenziali (discretizzate a 0.1s) per muovere coordinate X, Y, Z con gravità e inerzia.
- [x] **5.3. Extended Kalman Filter (EKF):** Configurare le matrici del Kalman ($F, H, Q, R$) dove $P$ indicherà la covarianza d'errore. La funzione fonderà la posizione GPS approssimata con i dati pseudo-reali di attenuazione RSSI e ricalcolerà $(X, Y, Z)$ stimate.
- [x] **5.4. Calcolo Metriche (RMSE):** Inserire codice di telemetria per salvare regolarmente l'errore metrico quadritico (RMSE) tra rotta stimata e reale. Se superati i 1.5 metri chiamare le regole collisione generate allo step 2.5.
- [x] **📌 TASK SPIEGAZIONE:** Demo semplice per chiarire come l'EKF mitiga l'incertezza dei sensori sporchi. Utilizzo di SQLite Viewer: ti mostrerò come navigare la tabella incrociata generata a questo scopo, mettendo in confronto "Stima" e "Verità" e facendomi spiegare da te come le query riescono a restituire i risultati che vedi.

## Modulo 6: `sdn_controller.py` - Optimization & Placement
- [x] **6.1. Inizializzazione:** Creare `sdn_controller.py`. Importare ML models da `sklearn.cluster` per le posizioni.
- [x] **6.2. Algoritmo di Deployment (Test 0):** Sviluppare funzione K-Means fusa a un approccio Greedy. Ricerca le centroidi di NLoS tra tutti gli scaffali nel Layout scelto e ci assegna la miglior RIS disponibile sui muri vicini per garantire link ottici diretti al >99%.
- [x] **6.3. Green 6G Engine:** Sviluppare l'euristica logica per SDN che controlla periodicamente (ogni 0.5s) quali RIS non incidono o hanno UAV lontani e setta la loro variabile di stato in Sleep (0.5 W) anziché Attiva (50 W).
- [x] **6.4. Hook per Predizione Tracking (Test 4):** L'SDN legge l'estrapolazione di traiettoria dell'EKF dei prossimi 5 metri e accende in modo preventivo la RIS necessaria per un "Make-before-Break" logico.
- [x] **📌 TASK SPIEGAZIONE:** Discuteremo il decision-making della rete centralizzata e l'economia dell'intelligenza AI. Attraverso l'estensione SQLite affronteremo dei log di Audit con cui ti chiederò di ricavare statisticamente con quanti millisecondi di anticipo la RIS era stata attivata.

## Modulo 7: `telemetry.py` - Digital Twin Visualization
- [x] **7.1. Motore SQLite Bulk Insert:** Centralizzare lo spooler del log (salvataggi per coordinate, log SNR, eventi eccezione) creando uno script con commit raggruppati ogni tot centinaia per non intasare l'IO del disco (usando il modulo sqlite3 nativo in modo asincrono).
- [x] **7.2. Metriche Plot Statiche:** Scrivere integrazione `matplotlib/seaborn` per CDF plot, Heatmap visuale degli ostacoli VS copertura radio, e grafico a barre per consumo energetico (Green 6G).
- [x] **7.3. 3D Digital Twin Viewer:** Codificare con libreria plotly una route dashboard per visualizzare i risultati tridimensionali in un formato "animato" con i percorsi interpolati.
- [x] **📌 TASK SPIEGAZIONE:** Spacchettamento totale del risultato. Focus sulle query. A questo punto dovrai padroneggiare in SQLite Viewer le query SQL minime per calcolare automaticamente medie di SNR o quantificare le posizioni lette nel server. Ti farò usare SQLite Export per salvare il CSV in locale sul Desktop.

## Modulo 8: `test_suite.py` - Suite Test e Validazione Tesi
- [x] **8.1. Inizializzazione Runner:** Creare file `test_suite.py` come main runner del digital twin.
- [x] **8.2. Logica Test 0:** Inserire le chiamate di avvio al layout A, B e C e avviare routine di posizionamento RIS (BOM testing).
- [x] **8.3. Logica Test 1 (Scalabilità):** Avviare istanze parallele multiprocessate per 50 UAV e lanciare log dei delay di gRPC per osservare ritardi.
- [x] **8.4. Logica Test 2 (Crash e Cinematica):** Scriptare un path di volo erratico per il drone, registrare l'aumento dell'incertezza e salvare timestamp di "incidente".
- [x] **8.5. Logica Test 3 (Energia):** Sommare e plottare la curva dell'uso combinato Potenza volo + Potenza Control-Plane + Potenza Moduli RIS (sempre attivi vs smart sleep).
- [x] **8.6. Logica Test 4 (Predizione):** Validazione finale del drone che esegue curve dietro gli scaffali senza perdere pacchetti usando le estrapolazioni per handover.
- [x] **📌 TASK SPIEGAZIONE:** Guida per la discussione di laurea. Come tradurre i grafici esportati in narrazione per la tua tesi; dimostrazione pratica d'uso estensivo del DB esportato via Viewer e conclusioni architetturali del programma Python su come le sue performance permettono il realtime rispetto ad altre tecnologie.
