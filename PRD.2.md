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

### Modulo 1: `config2.py` - Core System, Hardware & Warehouse Specs
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

### Modulo 2: `environment.py` - Physics Engine (Mondo Fisico Virtuale)
Sostituisce il magazzino reale, implementando la validazione geometrica 3D.
- **Generazione Parametrica**: Legge il config e popola automaticamente i volumi interni calcolando la quantità massima di file e scaffali.
- **Collision & Ray-Casting**: Mappa i vertici con `scipy.spatial.KDTree`. Valuta ad ogni step il link RF (LoS o NLoS).
- **Sicurezza Cinematica**: Margin di ±1.5 m nel corridoio. In caso di errore EKF o manovra brusca, l'intersezione col volume dello scaffale innesca un'eccezione di "Collisione Fatale".
- **JIT Acceleration**: Calcolo spaziale decorato con `@njit(nogil=True)` (Numba) per performance su decine di migliaia di facce metalliche.

### Modulo 3: `networking.py` - IPC Broker & Protocol Stack
Gestisce lo scambio dati tra processi asincroni.
- **Simulation Clock**: Tick deterministico impostato a $dt = 0.1$ s (10 Hz).
- **Shared Memory**: Dati di posizione dell'UAV e comandi SDN viaggiano su memoria condivisa a bassissima latenza.
- **gRPC Interface**: Backhaul tra Base Station e Super Server via Protobuf che modella l'overhead reale della rete.

### Modulo 4: `channel_model.py` - Modello di Canale 3GPP & RIS Physics
Motore elettromagnetico del simulatore.
- **Modello InF-DH (3GPP TR 38.901)**: Path Loss mitigato dal cluttter metallico; l'attenuazione NLoS è proporzionale ai metri di penetrazione.
- **Active RIS Gain**: Modello dell'amplificazione che include rumore termico dinamico dei componenti.
- **Beam Misalignment**: Perdita di guadagno ($\Delta$tilt) dovuta a tilt di pitch/roll.

### Modulo 5: `kinematics_ekf.py` - UAV Dynamics & Tracking Engine
La fisica volo e l'intelligenza di localizzazione.
- **Modello Cinematico 3D**: Simula inerzia, gravità, variazioni assetto vettoriale.
- **Extended Kalman Filter (EKF)**: Stima la posa fondendo dati (RSSI/AoA) "sporcati" da canale. Se l'errore $>1.5$ m, si innesca la collisione.
- **Predizione Traiettoria (Test 4 Hook)**: Estrapola le coordinate future per l'handover predittivo.
- **Metriche**: Calcolo dell'RMSE spaziale in continuo.

### Modulo 6: `sdn_controller.py` - Optimization & Placement
Intelligenza centralizzata per le RIS attive.
- **Test 0 (Deployment)**: K-Means e Greedy Search per esplorare pareti/incroci, posizionando le RIS minimizzando le zone di outage.
- **Logica Euristica (Green 6G)**: Commuta stato RIS (Active 50 W o Sleep) per efficienza energetica.
- **Handover Predittivo (Test 4 Hook)**: Pre-attiva la RIS se l'EKF segnala blocco NLoS imminente.

### Modulo 7: `telemetry.py` - Digital Twin Visualization
- **Heatmap & Plotting**: CDF su accuratezza e layout spaziali (SNR) integrati.
- **3D Digital Twin (Plotly)**: Animazioni real-time, corridoi opachi, doppia rotta (Ground Truth vs Stima) e coni di beamforming attivi.

---

## 3. Suite di Test Integrata (Validazione della Tesi)
Racchiusa nel Modulo 8 (`test_suite.py`) e rappresenta il nucleo scientifico:
- **Test 0 - Ottimizzazione Layout (BOM)**: Esegue algoritmi sui layout A, B, C per posizionamento ottimo delle RIS attive (99% copertura).
- **Test 1 - Scalabilità & Bottleneck gRPC**: Aumento a 50 UAV per testare degrado del Control Plane e outbreak da "Outdated CSI".
- **Test 2 - Impatto Cinematico & Crash Test**: Simulazione manovra evasiva, disallineamento della beam, divergenza dell'EKF e temporizzazione fino a "Collisione Fatale".
- **Test 3 - Green 6G (Trade-off Energetico)**: "Always-On" vs Centralizzata SDN ("Sleep").
- **Test 4 - Real-Time Predictive Tracking**: Validazione in Plotly dell'intelligenza pre-allocativa che azzera proattivamente zone d'ombra NLoS.
---
**Azione per l'IA:** Conferma di aver letto il PRD, riassumi in 2 righe l'obiettivo e chiedimi quale modulo o componente dell'architettura vuoi che iniziamo a sviluppare o modificare per primo.