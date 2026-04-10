import matplotlib.pyplot as plt
import numpy as np
import os

def generate_latency_plot():
    # Setup data
    x = np.linspace(0, 250, 500)
    y = np.zeros_like(x)
    
    # Costruiamo la curva in base alle richieste:
    # 0 - 40: 0.6
    # 40 - 250: sale in modo quasi esponenziale fino a 4.8 a 250
    # Usiamo una curva esponenziale o polinomiale morbida
    for i, val in enumerate(x):
        if val <= 40:
            y[i] = 0.6
        else:
            # Normalizziamo progress in [0, 1] per l'intervallo [40, 250]
            t = (val - 40) / 210.0
            # Usiamo potenza per dare l'effetto "quasi esponenziale" (flesso)
            y[i] = 0.6 + 4.2 * (t ** 2.0)
            
    # Per rendere il flesso tra 40 e 60 più realistico (esponenziale morbido)
    # possiamo sfumare (smooth) l'angolo a x=40 usando un filtro gaussiano o un polinomio
    # Ma la funzione t**2 è già C1-ish (non proprio ma ok) - Proviamo una transizione sigmoide?
    
    y = 0.6 + 4.2 * ((np.exp((x - 40)/45) - 1) / (np.exp(210/45) - 1))
    y[x < 40] = 0.6

    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Griglia e sfondo
    ax.set_facecolor("white")
    ax.grid(True, color='lightgray', linestyle='--', linewidth=0.7)
    
    # Asse X: 0 a 250, marker ogni 50
    ax.set_xlim(0, 250)
    ax.set_xticks(np.arange(0, 251, 50))
    ax.set_xlabel(r'Latenza di Rete $\Delta t$ [ms]', fontsize=14, fontweight='bold')
    
    # Asse Y: 0 a 5, marker ogni 1
    ax.set_ylim(0, 5)
    ax.set_yticks(np.arange(0, 6, 1))
    ax.set_ylabel('Errore Posizionamento EKF (RMSE) [m]', fontsize=14, fontweight='bold')
    
    # Titolo
    ax.set_title('Test 3: Impatto della Latenza di Rete e Beam Misalignment sull\'Errore di Tracciamento', 
                 fontsize=15, fontweight='bold', pad=20)
    
    # Linea dati
    ax.plot(x, y, color='red', linewidth=3, label='RMSE vs Latenza')
    
    # Linea soglia critica
    ax.axvline(x=50, color='black', linestyle='--', linewidth=2)
    
    # Area ombreggiata
    ax.axvspan(50, 250, color='red', alpha=0.1) # red/orange semitrasparente
    
    # Etichette di testo
    # Vicino alla linea tratteggiata
    ax.text(53, 4.0, 'Soglia Critica\nDisallineamento\nFascio (~50 ms)', 
            fontsize=12, fontweight='bold', color='black',
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='black', boxstyle='round,pad=0.5'))
    
    # Area sicura
    ax.text(20, 2.5, 'Safe Zone:\nBeam Intercepts UAV', 
            fontsize=12, fontweight='bold', color='green', ha='center',
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='green', boxstyle='round,pad=0.5'))
    
    # Area ombreggiata (outage)
    ax.text(150, 2.5, 'Outage Zone:\nEKF Coasting & Misalignment', 
            fontsize=12, fontweight='bold', color='firebrick', ha='center',
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='firebrick', boxstyle='round,pad=0.5'))
    
    plt.tight_layout()
    
    # Salvataggio
    output_path = os.path.join(os.path.dirname(__file__), 'Test_3_Stress_Latenza.png')
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Grafico salvato con successo: {output_path}")

if __name__ == '__main__':
    generate_latency_plot()
