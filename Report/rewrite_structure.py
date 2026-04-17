import sys
import re

file_path = "/Users/vernias/Desktop/verniastesi/Report/main.tex"

with open(file_path, "r", encoding="utf-8") as f:
    text = f.read()

# We need to find the start and end of the content to replace.
start_marker = r"\\titleformat\{\\subsubsection\}\[hang\]\{\\normalfont\\normalsize\\bfseries\}\{\\thesubsubsection\}\{1em\}\{\\raggedright\}"
end_marker = r"% BIBLIOGRAFIA ----------------------------------------------------"

match_start = re.search(start_marker, text)
match_end = re.search(end_marker, text)

if match_start and match_end:
    preamble = text[:match_start.end()]
    postamble = text[match_end.start():]
    
    new_content = """\n\n% -----------------------------------------------------------------\n\n"""
    new_content += r"\section{Introduzione}" + "\n"
    new_content += r"\subsection{Contesto Applicativo: Automazione logistica 4.0 e navigazione autonoma in corridoi VNA.}" + "\n"
    new_content += r"\subsection{Motivazione: Limiti NLoS industriali e necessità di architetture 6G-ready.}" + "\n"
    new_content += r"\subsection{Obiettivi del Lavoro: Tracking sub-metrico, abbattimento dell'outage e ottimizzazione dell'efficienza energetica (Green 6G)}" + "\n"
    new_content += r"\subsection{Metodologia: Sviluppo del Digital Twin in Python e struttura dell'elaborato.}" + "\n\n"

    new_content += r"\section{Modello Fisico e Propagazione Radio 6G}" + "\n"
    new_content += r"\subsection{Il Canale 3GPP TR 38.901: Modellazione InF-DH a 5.9 GHz}" + "\n"
    new_content += r"\subsection{Validazione Spaziale NLoS: Algoritmi di Ray-Casting e attenuazione da blocchi metallici}" + "\n"
    new_content += r"\subsection{Reconfigurable Intelligent Surfaces (RIS)}" + "\n"
    new_content += r"\subsubsection{Il limite del Fading Moltiplicativo nelle RIS Passive}" + "\n"
    new_content += r"\subsubsection{Architettura delle RIS Attive e gestione \"Green 6G\"}" + "\n"
    new_content += r"\subsubsection{Transizione verso le Beyond-Diagonal RIS (BD-RIS)}" + "\n"
    new_content += r"\subsubsection{Imperfezioni Hardware e Beam Misalignment}" + "\n"
    new_content += r"\subsection{Il Link Budget in Cascata: Analisi del rumore termico dinamico nel canale UAV-RIS-BS}" + "\n\n"

    new_content += r"\section{Architettura di Rete e Protocolli SDN}" + "\n"
    new_content += r"\subsection{Evoluzione 6G: Separazione tra Data/Control Plane e Service-Based Architecture (SBA)}" + "\n"
    new_content += r"\subsection{Livello MAC (Fronthaul): Accesso Grant-Free per telemetria UAV massiva}" + "\n"
    new_content += r"\subsection{Livello di Trasporto (Backhaul): Ottimizzazione tramite gRPC e Protobuf con modellazione parametrica della latenza}" + "\n"
    new_content += r"\subsection{Logica di Controllo (O-RAN RIC): Orchestrazione e integrazione di moduli AI/ML tramite Near-RT RIC}" + "\n\n"

    new_content += r"\section{Progettazione del Digital Twin \"Simulatore Tracking Indoor 6G\"}" + "\n"
    new_content += r"\subsection{Motore Cinematico UAV: Dinamica di volo e incertezze di orientamento con l'Iniezione del Rumore di Processo (Process Noise)}" + "\n"
    new_content += r"\subsection{Fusione Sensoriale EKF: Implementazione dell'Extended Kalman Filter per tracciamento autonomo.}" + "\n"
    new_content += r"\subsection{Controller SDN: Algoritmi K-Means per l'Ottimizzazione del Deployment (BOM), Logiche Predittive Neurali (LSTM) e Controllo Elettromagnetico}" + "\n"
    new_content += r"\subsubsection{Deployment Topologico NLoS e Algoritmi di Clustering K-Means}" + "\n"
    new_content += r"\subsubsection{Limiti dell'Inerzia e Analisi di Transitorio Neurale (LSTM)}" + "\n"
    new_content += r"\subsubsection{Soppressione della Blind Reflection e Controllo ON-OFF}" + "\n"
    new_content += r"\subsection{Architettura Software e Ottimizzazione Energetica: Multiprocessing, bypass del GIL Python, JIT Acceleration (Numba) e implementazione del Metodo di Dinkelbach}" + "\n"
    new_content += r"\subsubsection{Oltre il Threading: Bypass del GIL e IPC in Zero-Copy}" + "\n"
    new_content += r"\subsubsection{Strutture ad Albero (KD-Tree) e Compilazione JIT (LLVM Numba)}" + "\n"
    new_content += r"\subsubsection{Risoluzione di Dinkelbach e Transizione \"Green 6G\" (BD-RIS)}" + "\n"
    new_content += r"\subsection{Architettura Logica del Server Centrale SDN: Il Digital Twin}" + "\n"
    new_content += r"\subsubsection{gRPC Network Gateway (Interfaccia di Rete)}" + "\n"
    new_content += r"\subsubsection{Tracking Engine (Motore EKF e Predizione Neurale)}" + "\n"
    new_content += r"\subsubsection{RIS Orchestrator (L'Ottimizzatore Topologico)}" + "\n"
    new_content += r"\subsubsection{Mission Manager (Il Pilota Virtuale)}" + "\n\n"

    new_content += r"\section{Validazione Sperimentale e Analisi dei Risultati}" + "\n"
    new_content += r"\subsection{Setup di Simulazione: Definizione dei layout e parametri hardware.}" + "\n"
    new_content += r"\subsection{Test 1: Analisi Baseline NLoS e fallimento del tracciamento (Diagnosi)}" + "\n"
    new_content += r"\subsection{Test 2: Mitigazione tramite RIS Attive e Assisted-LoS (Soluzione).}" + "\n"
    new_content += r"\subsection{Test 3: Stress Test Latenza e Beam Misalignment}" + "\n"
    new_content += r"\subsection{Test 4: Ottimizzazione Avanzata e Controllo Predittivo (Green 6G)}" + "\n"
    new_content += r"\subsubsection{Mitigazione del Beam Misalignment: Modello Operativo \"Zero-Latency-Equivalent\"}" + "\n"
    new_content += r"\subsubsection{Analisi della Dinamica Temporale: Integrità Cinematica e Handover}" + "\n"
    new_content += r"\subsubsection{Analisi dell'Ecosistema Elettromagnetico: Controllo SDN per la Soppressione della Blind Reflection}" + "\n"
    new_content += r"\subsubsection{Mappa del Trade-off Energetico e Definizione della \"Zona Ottima\" Operativa}" + "\n"
    new_content += r"\subsubsection{Sinergia Algoritmica: Il Filtro Ibrido EKF-LSTM}" + "\n\n"

    new_content += r"\section{Conclusioni e Sviluppi Futuri}" + "\n\n\n\n\n\n"

    final_text = preamble + new_content + postamble
    
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(final_text)
    print("Done")
else:
    print("Markers not found")

