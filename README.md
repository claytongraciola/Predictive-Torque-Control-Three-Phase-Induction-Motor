# Predictive-Torque-Control-Three-Phase-Induction-Motor
Este repositório contem a implementação do controle preditivo de torque no Motor de Indução Trifásico (MIT) no ambiente computacional SIMULINK/MATLAB, neste readme.md esta descrito o calculo do controlador utilizado para fornecer a referência de torque ao inversor de frequência

# Introdução
As máquinas elétricas rotativas desempenham um papel fundamental na indústria moderna atuando na conversão eletromecânica de energia. O motor elétrico está presente em diversos equipamentos tais como compressores, esteiras, tornos, dentre outros. Sua importância consiste no fato de consumir de 44\% à 46\% de toda energia elétrica produzida no âmbito mundial. 
Dentre os motores elétricos, o Motor de Indução Trifásico (MIT) é amplamente utilizado na indústria devido ao seu baixo custo, robustez e baixa necessidade de manutenção. Ainda, pode-se citar como outra vantagem a possibilidade de controlar sua velocidade quando acionado por um inversor de frequência, possibilitando sua aplicação em uma maior gama de processos.
Diversas estratégias de controle podem ser aplicadas no MIT, neste repositório é discutido a proposta da aplicação do Controle Preditivo de Torque do inglês Predictive Torque Control (PTC) para o controle de velocidade.

![Malha de Controle](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/malha_controle.JPG?raw=true)

Os parâmetros do motor utilizados neste projeto são:

  - Potência Nominal - 746VA;
  - Tensão Nominal - 220V;
  - Frequência Nominal - 60Hz
  - Coeficiente de atrito do rotor - 0.001N.m.s
  - Momento de inercia do rotor - 0.0017kg.m^2$
  - Par de polos - 2
  - Indutância de magnetização - 694,1mH
  - Indutância do estator - 718,5mH
  - Indutância do rotor - 718,5mH
  - Resistência do rotor - 7.5022 Ohms
  - Resistência do estator - 4.8319 Ohms
  - Fluxo magnético nominal do estator - 0.78Wb
  - Torque Nominal - 4.1N.m
  - Velocidade Nominal - 188,49rad/s

# Controle preditivo de torque
Em uma máquina de indução é possível por meio de um vetor de tensão no inversor de frequência modificar o fluxo de estator e o torque eletromagnético. No controle de torque preditivo com base em uma função de custo que leva em consideração o fluxo de estator e o torque eletromagnético no instante (k + 1)  por meio de um algoritmo de predição destes dois valores, o que da o nome ao método de controle.

# Estimação dos parâmetros de operação do MIT

No MIT parâmetros como o fluxo de estator e o fluxo de rotor não podem ser medidos, e o algoritmo de predição do torque eletromagnético e do fluxo de estator, fazem o uso destas variáveis, portanto é necessário sua estimação.
	
Para se estimar o fluxo de estator no MIT faz-se o uso da equação:

![Fluxo de estator](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/fluxo_estator_presente.PNG?raw=true)

Para se estimar o fluxo de rotor no MIT faz-se o uso da equação:

![Fluxo de rotor(https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/fluxo_rotor_presente.PNG?raw=true)

# Predição dos parâmetros de operação do MIT
O controlador preditivo de torque faz o uso dos valores previstos de Torque eletromagnético e Fluxo do Rotor ambos no instante (k + 1).
Para se estimar o fluxo do estator no MIT no instante (k + 1) faz-se o uso da equação:

![Fluxo de estator no futuro](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/fluxo_estator_futuro.PNG?raw=true)

Para se estimar o torque eletromagnético no MIT no instante (k + 1) faz-se o uso da equação:

![Torque futuro](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/torque_futuro.PNG?raw=true)

Para estimar a corrente de estator no futuro faz-se o uso da equação:

![Corrente futuro](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/corrente_futuro.png?raw=true)

# Função de custo

No controle preditivo de torque a tensão aplicada no MIT é determinada por meio da minimização de uma função de custo, a função de custo é dada pela equação:

![Função de custo](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/opt_fun.png?raw=true)

A determinação do fator de peso é dada pela relação entre o Torque nominal do motor e o fluxo nominal de estator  apresentado na equação:

![Fator de Peso](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/fator_de_peso.png?raw=true)

# Determinação dos chaveamentos

Dado o calculo do custo para cada uma dos vetores de tensão e de chaveamento apresentados na tabela abaixo é determinado qual a posição das chaves por meio da função MIN onde a tensão que obtiver o menor custo é selecionado.

|   | Tensão                      | S1 | S2 | S3 |
|---|-----------------------------|----|----|----|
| 1 | 0                           | 0  | 0  | 0  |
| 2 | (2/3)Vdc                    | 1  | 0  | 0  |
| 3 | (1/3)Vdc + (jsqrt(3)/3)Vdc  | 1  | 1  | 0  |
| 4 | (-1/3)Vdc + (jsqrt(3)/3)Vdc | 0  | 1  | 0  |
| 5 | (-1/3)Vdc - (jsqrt(3)/3)Vdc | 0  | 1  | 1  |
| 6 | (-1/3)Vdc - (jsqrt(3)/3)Vdc | 0  | 0  | 1  |
| 7 | (1/3)Vdc - (jsqrt(3)/3)Vdc  | 1  | 0  | 1  |
| 8 | 0                           | 1  | 1  | 1  |

# Projeto do controlador
No controle preditivo de torque é utilizado um controlador cujo objetivo é fornecer uma referência de torque e com isso controlar a velocidade do MIT. Como a dinâmica elétrica do MIT é muito mais rápida que a dinâmica mecânica podemos simplificar o controlador por meio da função de transferência representada pela função de transferencia:

![Função de transferência](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/TF_rotor.png?raw=true)

Temos um controlador na forma descrita abaixo, e posicionando o zero do controlador em 2 vezes a posição do polo da função de transferência temos:

![Corrente futuro](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/controlador.png?raw=true)

Considerando como parâmetro de projeto um máximo overshoot de 0% e um tempo de acomodação 1ms, temos que o polo dominante do sistema tem que estar alocado em -3912,0230. Portanto calculando o ganho K com base na função de transferência de malha aberta TFma = C(s) . TF_r(s) conforme abaixo:

![Calculo Ganho](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/calculo_ganho.png?raw=true)

Sendo assim temos que o controlador em tempo contínuo e discretizado considerando Ts = 10us está representado abaixo:

![Controlador Final](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/fim_controladores.png?raw=true)

Por fim a equação de diferenças da função de transferência discretizada:

![Equação de diferenças](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/README/equacao_de_diferencas.png?raw=true)


# Testes implementados

A fim de avaliar a reposta do controlador foi desenvolvido 5 teste eles são:

  - Degrau de velocidade sem carga
  - Degrau de velocidade com carga
  - Velocidade constante com degrau de carga
  - Velocidade baixa com carga
  - Rotor bloqueado com carga

# Resultados função de transferencia ideal

Teste de degrau de velocidade sem carga:

![Velocidade Teste A/A](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_A/RESPOSTA/PNG/A_VELOCIDADE.png?raw=true)
![Carga Teste A/A](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_A/RESPOSTA/PNG/A_CARGA.png?raw=true)

Teste de degrau de velocidade com carga:

![Velocidade Teste A/B](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_A/RESPOSTA/PNG/B_VELOCIDADE.png?raw=true)
![Carga Teste A/B](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_A/RESPOSTA/PNG/B_CARGA.png?raw=true)


Teste de velocidade constante com degrau de carga:

![Velocidade Teste A/C](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_A/RESPOSTA/PNG/C_VELOCIDADE.png?raw=true)
![Carga Teste A/C](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_A/RESPOSTA/PNG/C_CARGA.png?raw=true)

Teste de velocidade baixa com carga

![Velocidade Teste A/D](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_A/RESPOSTA/PNG/D_VELOCIDADE.png?raw=true)
![Carga Teste A/D](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_A/RESPOSTA/PNG/D_CARGA.png?raw=true)

Teste de rotor bloqueado com carga

![Velocidade Teste A/E](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_A/RESPOSTA/PNG/E_VELOCIDADE.png?raw=true)
![Carga Teste A/E](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_A/RESPOSTA/PNG/E_CARGA.png?raw=true)

# Resultados Simulação real

Teste de degrau de velocidade sem carga:

![Velocidade Teste B/A](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/A_VELOCIDADE.png?raw=true)
![Carga Teste B/A](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/A_CARGA.png?raw=true)
![Corrente Teste B/A](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/A_CORRENTE.png?raw=true)

Teste de degrau de velocidade com carga:

![Velocidade Teste B/B](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/B_VELOCIDADE.png?raw=true)
![Carga Teste B/B](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/B_CARGA.png?raw=true)
![Corrente Teste B/B](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/B_CORRENTE.png?raw=true)

Teste de velocidade constante com degrau de carga:

![Velocidade Teste B/C](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/C_VELOCIDADE.png?raw=true)
![Carga Teste B/C](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/C_CARGA.png?raw=true)
![Corrente Teste B/C](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/C_CORRENTE.png?raw=true)

Teste de velocidade baixa com carga

![Velocidade Teste B/D](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/D_VELOCIDADE.png?raw=true)
![Carga Teste B/D](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/D_CARGA.png?raw=true)
![Corrente Teste B/D](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/D_CORRENTE.png?raw=true)

Teste de rotor bloqueado com carga
![Velocidade Teste B/E](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/E_VELOCIDADE.png?raw=true)
![Carga Teste B/E](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/E_CARGA.png?raw=true)
![Corrente Teste B/E](https://github.com/eduardoanog/Predictive-Torque-Control-Three-Phase-Induction-Motor/blob/master/IMAGES/SIMULATION_B/RESPOSTA/PNG/E_CORRENTE.png?raw=true)



