## Citation

SOUSA, J. M. de. Controle preditivo não-linear aplicado ao sistema de piloto automático de mísseis. Dissertação (Dissertação de Mestrado) — Instituto Militar de Engenharia,
Rio de Janeiro, Brasil, 2024. Departamento de Engenharia Elétrica.

## Descrição da Dissertação (Português)

TÍTULO:
Controle Preditivo Não-Linear Aplicado ao Sistema de Piloto Automático de Mísseis

RESUMO:
Esta dissertação aborda o desenvolvimento e a aplicação do Controle Preditivo Baseado em Modelo (MPC), em sistemas de piloto automático de mísseis. A modelagem matemática detalhada de mísseis axissimétricos controlados por aletas é apresentada, incluindo simplificações práticas que mantêm a precisão necessária para o controle efetivo em todo o envelope de voo. Com base nessa modelagem, um controlador MPC Não Linear é desenvolvido e testado através de simulações, comparando seu desempenho com um controlador previamente projetado por Simões e Cavalcanti (1) (2), evidenciando a viabilidade e a eficácia do MPC em aplicações complexas e não lineares como o controle de piloto automático de mísseis. Ressalta-se a simplicidade, a intuitividade e a precisão do MPC, especialmente quando aplicado a sistemas com modelagem matemática bem definida. O desempenho do controlador MPC mostra-se superior em comparação ao controlador IQC em simulações nominais, apresentando estabilidade robusta a variações paramétricas e capacidade de implementação em tempo real, dadas as rápidas soluções de otimização alcançadas. Contribuições significativas incluem a formulação de uma equação aproximada para a pressão dinâmica em fluidos compressíveis, para diferentes valores do número de Mach e de altitude, e a proposição de um controlador MPC com dois modelos internos, um para predição e outro para atualização de estados. Limitações identificadas abrem caminho para pesquisas futuras, sugerindo a inclusão de técnicas de controle robusto no MPC, a aplicação de dados aerodinâmicos de mísseis, que sejam mais realistas, e a consideração da dinâmica completa de 6-DOF. Este trabalho não apenas avança o conhecimento no campo do controle MPC Nominal, mas também estabelece uma base sólida para pesquisas futuras na área de sistemas de controle MPC Não-linear e Robusto.

Palavras-chave: mpc. não-linear. míssil. 3dof. piloto automático. matlab. casadi.

(1) SIMÕES, A. M.; CAVALCANTI, V. M. G. B. Missile autopilot design via structured robust linear parameter-varying synthesis. Journal of Guidance, Control, and Dynamics, v. 46, n. 8, p. 1649–1656, 2023. Disponível em: <https://doi.org/10.2514/1.G007580>.

(2) CAVALCANTI, V. M. G. de B. Síntese de controladores estruturados sob restrições quadráticas integrais. 159 p. Tese (Doutorado em Engenharia de Defesa) — Instituto Militar de Engenharia, Rio de Janeiro, Brasil, 12 2021.

## Thesis Description (English)

TITLE:
Controle Preditivo Não-Linear Aplicado ao Sistema de Piloto Automático de Mísseis

ABSTRACT:
This dissertation addresses the development and application of Model-Based Predictive Control (MPC) in missile autopilot systems. Detailed mathematical modeling of axisymmetric missiles controlled by fins is presented, including practical simplifications that maintain the necessary precision for effective control throughout the flight envelope. Based on this modeling, a Nonlinear MPC controller is developed and tested through simulations, comparing its performance with a controller previously designed by Simões and Cavalcanti (1) (2), highlighting the feasibility and efficacy of MPC in complex and nonlinear applications like missile autopilot control. The simplicity, intuitiveness, and precision of the MPC, especially when applied to systems with well-defined mathematical modeling, are emphasized. The performance of the MPC controller proves superior in comparison to the IQC controller in nominal simulations, exhibiting robust stability to parametric variations and the capability of real-time implementation, given the rapid optimization solutions achieved. Significant contributions include the formulation of an approximate equation for dynamic pressure in compressible fluids, for different Mach numbers and altitudes, and the proposition of an MPC controller with two internal models, one for prediction and another for state updates. Identified limitations pave the way for future research, suggesting the inclusion of robust control techniques in MPC, the application of more realistic missile aerodynamic data, and the consideration of the complete 6-DOF dynamics. This work not only advances knowledge in the field of Nominal MPC control but also establishes a solid foundation for future research in the area of Nonlinear and Robust MPC control systems.

Keywords: mpc. nonlinear. missile. 3dof. autopilot. matlab. casadi.

(1) SIMÕES, A. M.; CAVALCANTI, V. M. G. B. Missile autopilot design via structured robust linear parameter-varying synthesis. Journal of Guidance, Control, and Dynamics, v. 46, n. 8, p. 1649–1656, 2023. Disponível em: <https://doi.org/10.2514/1.G007580>.

(2) CAVALCANTI, V. M. G. de B. Síntese de controladores estruturados sob restrições quadráticas integrais. 159 p. Tese (Doutorado em Engenharia de Defesa) — Instituto Militar de Engenharia, Rio de Janeiro, Brasil, 12 2021.

## Software Configuration

| Software                                   | Version    |
|--------------------------------------------|------------|
| Microsoft Visual Studio                    | 2022       |
| MinGW-w64 Compiler                         | 6.3        |
| MATLAB                                     | R2023a     |
| MATLAB Support for MinGW-w64 Compiler      | R2023a     |
| CasADi                                     | 3.6.3      |
| Mosek                                      | 10.0       |

## Hardware Configuration

| Component           | Specification          |
|---------------------|------------------------|
| Processor           | Core i5-1235U          |
| L3 Cache            | 12 MB                  |
| Base Frequency      | 1.30 GHz               |
| Maximum Frequency   | 4.40 GHz               |
| Cores               | 10                     |
| Threads             | 12                     |
| TDP Max             | 15 W                   |
| Integrated Graphics | Intel Iris Xe Graphics |
| Total Memory        | 16 GB                  |
| Memory Type         | LPDDR4x                |
| Memory Speed        | 4267 MHz               |

## How to

To execute the code, Matlab must be launched from the 'x64 Native Tools Command Prompt for VS 2022'. See image below:

<img src="https://github.com/jonaylton/mpc-missile/assets/75505777/a6c64e39-3e6a-4155-bac4-a4f3903e64cb" width="70%" />
