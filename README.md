# Vehicle Dynamics MBD 🚗

Plataforma modular e profissional para Simulação e Controle de Dinâmica Veicular baseada na metodologia **Model-Based Design (MBD)**. Projeto de pesquisa/PIBIC focado no desenvolvimento e validação de estratégias avançadas de controle de estabilidade (AARB - Active Anti-Roll Bar).

## 📌 Arquitetura do Sistema

O framework foi redesenhado para seguir os padrões industriais rigorosos (MathWorks Automotive Advisory Board - MAB), dividindo responsabilidades e evitando acoplamento entre simulação, visualização e projeto de controle.

```text
VehicleDynamicsMBD/
├── controllers/                           # Códigos e matrizes dos Controladores (PID, LQR, MPC)
│   ├── create_mpc_controller.m            # Script padronizado de criação do objeto MPC
│   ├── script_and_data_PASSIVO.mlx        # Modelos de referência passivos
│   └── ...
├── data/
│   └── parameters/
│       └── vehicle_params.sldd            # Data Dictionary (Única Fonte da Verdade para dados MBD)
├── models/
│   ├── components/                        # Subsistemas reutilizáveis (Library Blocks)
│   └── variants/                          
│       ├── 5dof/                          # Modelagem validadada (Massa Suspensa, Yaw, Roll, Sideslip)
│       └── 9dof/                          # Em desenvolvimento (dinâmica vertical de rodas independente)
├── results/                               # Saídas das simulações (arquivos .mat e .png)
├── scenarios/                             # Descrições e geradores de malhas de teste (NHTSA, ISO)
│   └── create_maneuver_data.m             # Construtor padronizado de perfis de direção (timeseries)
├── utils/                                 # Ferramentas auxiliares e pipelines
│   └── data_management/                   # Funções de logging, plotting (plot_5dof_results_v2.m) e parse
├── run_5dof_simulation.m                  # Ponto de entrada MBD para 5-DOF
├── run_experiments.m                      # Batch job de múltiplas simulações (Passive, PID, MPC)
└── setup_environment.m                    # Gerenciador de cache, paths e dependências
```

## ⚙️ Modelos de Veículo

**Veículos Base Validados:**
* Chevrolet Blazer 2001 (1905 kg)
* Heavy Vehicle - Gaspar 2004 (Massa Suspensa Comercial)

**Controladores Desenvolvidos:**
1. **PID:** Tuning focado em rejeição de distúrbios de rolagem.
2. **LQR:** Gain-scheduling baseado na literatura.
3. **MPC (Model Predictive Control):** Formulação multi-variável (2 MVs) para torques independentes nos eixos dianteiro ($T_f$) e traseiro ($T_r$). Limites rígidos de saturação de atuador baseados em *Khalil (2019)* e *Gaspar (2004)*.

## 🚀 Como Utilizar

A inicialização e o fluxo de trabalho não dependem mais de scripts soltos ou *hardcoding* no Simulink. Todo o fluxo é orquestrado pelas funções base:

**1. Rodando um Teste Único**
```matlab
% Inicia e configura diretórios
setup_environment;

% Roda uma simulação com controlador MPC, manobra de Gaspar a 70 km/h
results = run_5dof_simulation('MPC', 'Gaspar', 70);
```

**2. Executando um Batch de Validação**
```matlab
% Executa todos os testes parametrizados iterativamente e salva dashboard unificado
run_experiments;
```

## 📊 Métricas de Desempenho
As métricas (ex: Máximo Ângulo de Rolagem, RMS Roll, Exec Time) são extraídas diretamente do pipeline `Simulink.SimulationData` via função `calculate_performance_metrics_v2.m`. 

---
**Autor:** Vitor Yukio (UnB/PIBIC)
**Versão:** 2.1 (MBD Compliant)
