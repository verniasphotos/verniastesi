# PRD - Simulatore Tracking Indoor 6G (Digital Twin)

## 0. Istruzioni di Sistema per l'Assistente AI
- Ruolo: Senior Python Software Engineer e Tutor Universitario.
- Utente: Studente di Ingegneria (livello laurea Triennale).
- Librerie Consentite (Python Puro):
  - **Fisica e Matematica**: `numpy`, `scipy`, `numba`
  - **Tracking EKF**: `filterpy`, `sympy`
  - **Ottimizzazione (ML)**: `scikit-learn`
  - **Networking SDN**: `grpcio`, `grpcio-tools`, `protobuf`
  - **Telemetria e Rendering**: `pandas`, `matplotlib`, `seaborn`, `plotly`
- **Vincoli Architetturali**:
  - Codice pulito, modulare, fortemente tipizzato (Type Hints).
  - Iper-commentato in italiano con docstring in stile Google.
  - Architettura rigorosa basata su **Multiprocessing + Shared Memory** per bypassare il GIL e separare *Control Plane* e *Data Plane*.
  - **JIT Compilation** (`@njit(nogil=True)`) per calcoli matriciali pesanti (niente threading classico o formule semplificate, rigoroso standard 3GPP).
  - Sviluppo iterativo: scrivere il codice un modulo alla volta.

---

## 1. Descrizione del Progetto
"Simulatore tracking Indoor 6G" è un Digital Twin ad alte prestazioni sviluppato in Python. Simula un'architettura di rete 6G centralizzata (SBA) per il tracciamento e la comunicazione di droni (UAV) in un magazzino logistico denso di ostacoli metallici. Sfrutta *Multiprocessing + Shared Memory* per aggirare il GIL di Python e garantire latenze di calcolo real-time, utilizzando *RIS Attive* controllate via *SDN* per superare il fading moltiplicativo tipico degli scenari NLoS. L'ambiente fisico modella tre tipologie di layout industriali, integrando vincoli di sicurezza per la navigazione autonoma.

---

## 2. Architettura Modulare

### Modulo 1: `modulo_1_config.py` - Core System, Hardware & Warehouse Specs
Funge da *Single Source of Truth*. Definisce l'estensione delle classi e le costanti fisiche estratte dalla scheda tecnica.
- **Hardware UAV**: Massa (1.2 kg), profili di potenza in volo (150 W hover, 170 W movimento, 2 W modulo Radio 6G), limiti cinematici (15° massimi di pitch/roll).
- **Hardware RIS Attiva**: Costanti di consumo (Psleep = 0.5 W, Pactive = 50.0 W), Guadagno di amplificazione ($\Delta$gain), e Cifra di Rumore (F).
- **Network Specs**: Frequenza operativa 5.9 GHz (Banda U-NII-4 / 6G-ready per scenari industriali), soglia SNR di outage al ricevitore (5 dB).
- **Layout Magazzino (Dimensioni Architetturali)**:
  - **Modulo Scaffale**: 1.2 x 1.0 m (X, Y) con mensole distanziate a 0.6 m in altezza (Z).
  - **Corridoi & Mura**: Corridoi VNA (Very Narrow Aisle) larghi 3.0 m; Margine di rispetto mura perimetrali di 2.5 m.
  - **Tipologie di Deployment**:
    - **Layout A (Piccolo)**: 50 x 40 x 10 m (Area: 2.000 mq).
    - **Layout B (Medio)**: 100 x 100 x 10 m (Area: 10.000 mq).
    - **Layout C (Grande)**: 250 x 140 x 15 m (Area: 35.000 mq).
  - **Penetration Loss Factor**: Attenuazione specifica del clutter metallico impostata a 15.0 dB/m.

### Modulo 2: `modulo_2_environment.py` - Physics Engine (Mondo Fisico Virtuale)
Sostituisce il magazzino reale, implementando la validazione geometrica 3D.
- **Generazione Parametrica**: Legge il config e popola automaticamente i volumi interni calcolando la quantità massima di file e scaffali.
- **Collision & Ray-Casting**: Mappa i vertici con `scipy.spatial.KDTree`. Valuta ad ogni step il link RF (LoS o NLoS).
- **Sicurezza Cinematica**: Margin di ±1.5 m nel corridoio. In caso di errore EKF o manovra brusca, l'intersezione col volume dello scaffale innesca un'eccezione di "Collisione Fatale".
- **JIT Acceleration**: Calcolo spaziale decorato con `@njit(nogil=True)` (Numba) per performance su decine di migliaia di facce metalliche.

### Modulo 3: `modulo_3_networking.py` - IPC Broker & Protocol Stack
Gestisce lo scambio dati tra processi asincroni.
- **Simulation Clock**: Tick deterministico impostato a $dt = 0.1$ s (10 Hz).
- **Shared Memory**: Dati di posizione dell'UAV e comandi SDN viaggiano su memoria condivisa a bassissima latenza.
- **gRPC Interface**: Backhaul tra Base Station e Super Server via Protobuf che modella l'overhead reale della rete.

### Modulo 4: `modulo_4_channel_model.py` - Modello di Canale 3GPP & RIS Physics
Motore elettromagnetico del simulatore.
- **Modello InF-DH (3GPP TR 38.901)**: Path Loss mitigato dal cluttter metallico; l'attenuazione NLoS è proporzionale ai metri di penetrazione.
- **Active RIS Gain**: Modello dell'amplificazione che include rumore termico dinamico dei componenti.
- **Beam Misalignment**: Perdita di guadagno ($\Delta$tilt) dovuta a tilt di pitch/roll.

### Modulo 5: `modulo_5_cinematica_EKF.py` - UAV Dynamics & Tracking Engine
La fisica volo e l'intelligenza di localizzazione.
- **Modello Cinematico 3D**: Simula inerzia, gravità, variazioni assetto vettoriale.
- **Extended Kalman Filter (EKF)**: Stima la posa fondendo dati (RSSI/AoA) "sporcati" da canale. Se l'errore $>1.5$ m, si innesca la collisione.
- **Predizione Traiettoria (Test 4 Hook)**: Estrapola le coordinate future per l'handover predittivo.
- **Metriche**: Calcolo dell'RMSE spaziale in continuo.

### Modulo 6: `modulo_6_sdn_controller.py` - Optimization & Placement
Intelligenza centralizzata per le RIS attive.
- **Test 0 (Deployment)**: K-Means e Greedy Search per esplorare pareti/incroci, posizionando le RIS minimizzando le zone di outage.
- **Logica Euristica (Green 6G)**: Commuta stato RIS (Active 50 W o Sleep) per efficienza energetica.
- **Handover Predittivo (Test 4 Hook)**: Pre-attiva la RIS se l'EKF segnala blocco NLoS imminente.

### Modulo 7: `modulo_7_telemetria.py` - Digital Twin Visualization
- **Heatmap & Plotting**: CDF su accuratezza e layout spaziali (SNR) integrati.
- **3D Digital Twin (Plotly)**: Animazioni real-time, corridoi opachi, doppia rotta (Ground Truth vs Stima) e coni di beamforming attivi.

---

## 3. Suite di Test Integrata (Validazione della Tesi)
Racchiusa nella cartella `simulator/` e rappresenta il nucleo scientifico della validazione sperimentale, documentata tramite file di simulazione ed esporti per la tesi:
- **Test 0 - Ottimizzazione Layout (BOM) (`test_0_BOM_K-Means.py`)**: Analisi della topologia 3D per i layout A, B, C. Utilizza K-Means e un approccio Greedy per determinare l'allocazione spaziale ottimale (Bill of Materials) delle RIS minimizzando il NLoS. *Immagini generate: `topological_map_layout_{a,b,c}.png`.*
- **Test 1 - Fallimento EKF in NLoS Baseline (`test_1_layoutB_ekf_tracking.py`)**: Simulazione track top-down del drone in assenza di segnale LoS. Il Filtro di Kalman entra in "coasting" predittivo scaturendo nell'esplosione dell'ellisse di covarianza (errore crescente) e portando alla divergenza del tracciamento. *Immagini generate: `Test_1.1_TopDown_Traiettoria.png`.*
- **Test 2 - Successo Tracking RIS-Assistito (`test_2_layoutB_ris_success.py`)**: Test della compensazione del fading con l'architettura attiva RIS SDN-driven. La caduta NLoS viene sanata dal routing dinamico forzando l'Errore EKF sotto il margine di accuratezza (< 1m). *Immagini generate: `Test_2_B_Successo_RIS.png`.*
- **Test 3 - Stress Latenza di Rete 6G (`test_3_latency_stress.py`)**: Misura l'impatto dello strato applicativo (delay applicato del Control Plane 6G in ms) sul tracking. Definisce la demarcazione visiva di rottura fra navigazione sicura ("Safe Zone") e "Outage Zone" tracciando la tolleranza limite attorno ai 50ms per l'allineamento dei beam conformali. *Immagini generate: `Test_3_Stress_Latenza.png`.*
- **Test 4 - Ottimizzazione Proattiva & Handover (Architetturale)**: Pre-wake-up energetico in ottica Green 6G. Usa le stime per la pre-attivazione tattica minimizzando il NLoS passivo. Evidenziato dalle logiche SDN del controller e documentate nella tesi.

---
**Azione per l'IA:** Conferma la sincronia del file con la reale topologia del progetto. Chiedimi quale integrazione testuale nel Capitolo 5 o review finale possiamo intavolare adesso a completamento del resoconto.