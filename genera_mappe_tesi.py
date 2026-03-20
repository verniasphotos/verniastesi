from config import DeploymentPlanner

# Definizione dei 3 Casi Studio della tesi
casi = [
    {"nome": "Caso A", "L": 50, "W": 40, "H": 10},
    {"nome": "Caso B", "L": 100, "W": 100, "H": 10},
    {"nome": "Caso C", "L": 250, "W": 140, "H": 15}
]

print("=== Generazione Cartine Topologiche e BoM per la Tesi ===")

for caso in casi:
    print(f"\n=> Elaborazione {caso['nome']} ({caso['L']}x{caso['W']}x{caso['H']} m)")
    
    # Inizializza il planner per questo caso
    planner = DeploymentPlanner(caso['L'], caso['W'], caso['H'])
    
    # Genera il nome del file (es: plot_deployment_bom_Caso_A.png)
    filename = f"plot_deployment_bom_{caso['nome'].replace(' ', '_')}.png"
    
    # Genera e salva la dashboard
    planner.genera_dashboard(filename)
    
print("\n=== Tutte le 3 cartine sono state generate con successo! ===")
