# Istruzioni di Sistema per l'Assistente IA
Agisci come un Senior Python Software Engineer e Tutor Universitario. Il tuo compito è aiutare uno studente di Ingegneria (livello laurea Triennale) a sviluppare un simulatore di rete 6G. 
Il codice deve essere scritto in **Python puro** usando solo le librerie `numpy`, `matplotlib`, `math` e `sqlite3`. 
**Vincoli architetturali:** - Scrivi codice pulito, iper-commentato in italiano e facile da capire. 
- Evita complessità inutili (niente multithreading, niente machine learning, niente simulazioni elettromagnetiche complesse: usa formule geometriche e distanze euclidee semplificate).
- **Non generare tutto il codice in una volta sola.** Leggi questo PRD, conferma di averlo compreso e chiedimi quale modulo vuoi che io ti faccia generare per primo (es. "Vuoi iniziare con config.py?").

---

# PRD: Simulatore 6G per Magazzino Logistico con Droni e RIS (Tesi Triennale)

## 1. Descrizione del Progetto
Il software simula un "cervello centrale" (Controller) in un magazzino logistico dove operano droni 24/7. Poiché gli scaffali metallici bloccano il segnale (NLOS), il sistema usa protocolli simulati di 2-Way Ranging (2WAY) per stimare l'attenuazione del metallo e accende dinamicamente dei pannelli RIS ("specchi intelligenti") per far rimbalzare il segnale radio verso i droni, spegnendoli poi per risparmiare energia. Il simulatore testa il limite di rottura (stress test) della rete e include una visualizzazione topologica e un "Digital Twin" animato per testare la validità logistica dei percorsi. L'architettura è suddivisa rigorosamente in 10 moduli.

## 2. Modulo 1: Costanti e Parametri
Definisce tutte le costanti ingegneristiche e le regole d'ambiente.
- **Rete 6G**: `FREQ` (3.5 GHz), potenze di TX (Drone 20 dBm, BS 40 dBm), raggi operativi (`R_BS` 50m, `R_RIS` 15m), attenuazioni ostacoli (15 dB per scaffale), soglie SNR di attivazione RIS (5.0 dB) e rumore bianco (-100 dBm).
- **Droni**: `V_DRONE` (3.0 m/s), simulazione `DT` (0.1 s). Ottimizzazione batteria e tolleranze RT (Return To Home al 20%). Volo in `Z_DRONE_FISSO`.
- **BS/RIS**: Consumi operativi (`P_SLEEP` 0.5W, `P_PASSIVE` 5W, `P_ACTIVE` 50W, BS inoltro 30W), offset soffitto/parete. Supporto configurazioni per il multilivello e BS "ibride" (anche RIS).
- **Scaffali e Limiti**: Dimensioni moduli (1.2x1.0m, luce 0.6m tra ripiani) e capacità del controller (`MAX_RIS_CALLS_PER_DT`).

## 3. Modulo 2: Geometria e Magazzino
Genera l'infrastruttura 3D (`Magazzino`) che fa da base alla sperimentazione.
- **Parametri Costruttivi**: Accetta in input `L, W, H`, calcola piani utili/mensole ed estrae un array parametrico di scaffali (`C01-S05`) e corridoi fisici 3D.
- **Algoritmo di Deployment 6G Originale**: Effettua loop su offset maglia per piazzare la Base Station ottimale, i pannelli RIS al soffitto (almeno 1 per corsia, controllando assenza di sovrapposizione con le BS) e le RIS a parete a inizio e fine corsia in caso di magazzini massivi (`MULTILIVELLO` > 10.000mq).
- **Ray-casting e Ostacoli**: Metodo `check_LOS_and_shielding` getta un raggio geometrico 3D tra punto P1 (Tx) e punto P2 (Rx), calcolando quanti ostacoli metallici "buca", convertendoli in attenuatori.

## 4. Modulo 3: Entità della Simulazione e Hardware
Codifica orientata agli oggetti OOP per gli elementi dinamici:
- **`Pacchetto_Rete`**: Struttura header/payload del pacchetto informativo radio simulato. Supporta logging multi-hop per capire da dove rimbalza (firma drone, firma RIS, firma BS).
- **`Drone`**: Entità autonoma. Include simulazione batteria (`aggiorna_batteria`), navigazione 3D deterministica verso un obiettivo con loop temporale ad avanzamento DT (`muovi_verso`) e logica switch missione `RTH_RICARICA`.
- **`RIS`**: Gli Specchi di Rete. Espongono consumi attivi e passivi, e il core routing `inoltra_pacchetto` in grado di amplificare (+10 dBm) il segnale se in modalità attiva.
- **`BaseStation`**: Hardware fisso recettore con capacità "ibrida" (assorbe ruoli di RIS). Usa il modulo `ricevi_e_inoltra` per accreditare log e far avanzare informazioni al master controller.

## 5. Modulo 4: Fisica del Canale e Propagazione
La matematica dietro il 6G. Esegue la funzione stand-alone `esegui_2way_ranging`.
- Usa il `Free Space Path Loss` di Friis per il calo segnale base.
- Somma il blocking dovuto agli scaffali dal modulo `Geometria`.
- Analizza lo scambio dati Handshake (`Uplink` and `Downlink`). Calcola l'SNR asimmetrico.
- Decodifica la migliore RIS di emergenza disponibile nel raggio vitale escludendo quelle senza Line-Of-Sight verso il Drone e la Base Station simultaneamente.

## 6. Modulo 5: Memoria del Sistema e Database
Implementa classe `DatabaseManager` usando SQLite 3 file-based (`telemetria.db`):
- Tabella `Telemetria_Droni`: traccia storicamente 9 parametri temporali per veicolo tra cui coordinate vettoriali 3D e degrado batteria.
- Tabella `Eventi_Rete`: raccoglie la spesa di Watt/ora sul singolo RIS ed estrae dati cumulativi per log JSON complessi usati da matplotlib e digital twin. 

## 7. Modulo 6: Logica Decisionale Centralizzata (Controller)
La direttiva Master (il `SuperServer`):
- Espone il framework `ricevi_telemetria`.
- Controlla emergenze batteria e switcha il drone in RTH.
- Regola a caldo con euristiche i consumi delle RIS in modalità dinamica (`passive` vs `active` su thresholds precalcolati di emergenza SNR).
- Salva log drone e di rete su DB.

## 8. Modulo 7: Motore di Simulazione e Scenari di Test
La suite `SimulationEngine` esegue cicli fisici iterativamente (sia set standard che per set custom dell'utente):
- **Test 1 Scalabilità**: Tenta rottura server o crollo SNR aggiungendo sciami (+5 droni x step).
- **Test 2 Resilienza (Heatmap)**: Simula guasto e blackout di 4 RIS a soffitto in scenari operativi. Calcola griglia 2D SNR PRIMA/DOPO per generare marker JSON da plot.
- **Test 3 Mass RTH**: Spinge logiche dual-axis con 40% flotta scaricata artificialmente (al 21%) al secondo 20, forzando congestione massiva.
- **Test 4 Efficienza**: Run di confronto Baseline "Always-ON 50W" contro il Sistema Dinamico proposto su simulazione 150 step (15 sec).
- **Test 5 Digital Twin Animato**: Implementa logiche di picking ottimizzato (`_genera_percorso_ottimizzato`) in corsie rettilinee per un UAV, evitando diagonali in collisione. Rilascia JSON con step per animazioni. 

## 9. Modulo 8: Visualizzazione Grafica Risultati
La classe `DataPlotter` interroga in logica query i marker JSON e DB generati e produce plot matplotlib:
- `plot_scalabilita` (Linee + marker `X` rottura).
- `plot_resilienza_guasto` (Heatmap contourf con disegno infrastruttura "identico alle BOM" con base gialla, RIS Soffitto verdi. Evidenzia grossa "X" rossa sul guasto).
- `plot_consumi_mass_rth` (Plot a doppio asse: andamento min/max batteria e istogramma RIS).
- `plot_risparmio_energetico` (Barre affiancate Rosso vs Verde per kW impiegati).
- `plot_digital_twin` (Motore FFMpeg/Pillow per interpolare trajectorie e picking su planimetria dinamica).

## 10. Modulo 9: Deployment Dinamico e Visualizzazione Topologica
Il Planner `DeploymentPlanner` calcola l'hardware BOM del layout:
- Previene collisioni (`_is_too_close_to_bs`).
- Griglie Base Stations, deployment RIS (Soffitto e Parete).
- Output affiancato: Dashboard visiva Mappa (`ax_mappa`) + Testo Testo Tabellato (`ax_bom`) contenente recap Magazzino e Infrastrutture di Rete.

## 11. Modulo 10: Simulazione Dinamica (Il Loop)
- Blocco Main Console (`if __name__ == "__main__":`).
- Prende le 3 dimensioni custom utente.
- Palesa layout, stima droni consigliati contro rottura logistica.
- Calibra `Magazzino` su `FISSO` o `MULTILIVELLO`.
- Mostra statistiche immediate (Test Link 2-Way Ranging "1 shot").
- Offre loop Menù CMD (1-5 e Uscita) invocando sincronicamente per ciascun comando il Motore, Plotter per layout STD e Plotter Custom utente in array parallelo.

---
**Azione per l'IA:** Conferma di aver letto il PRD strutturato a 11 parti, riassumi in 2 righe l'obiettivo e chiedimi quale file vuoi che sviluppiamo per primo.