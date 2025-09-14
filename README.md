# PIDWise Notebook (FOPTD + IMC-PID)

Este projeto contém um **notebook em Python** para simulação de um processo de **primeira ordem com tempo morto (FOPTD)** controlado por um **PID ISA paralelo dependente**, com sintonia baseada em **IMC (Internal Model Control)**.

## Funcionalidades

- Usuário fornece os parâmetros do processo:
  - `K` (ganho do processo)  
  - `tau` (constante de tempo)  
  - `theta` (tempo morto)  

- Cálculo automático dos ganhos PID via método **IMC**:
  - `Kp`  
  - `Ti`  
  - `Td`  
  - `lambda` (parâmetro de sintonia do IMC)  

- Campos editáveis para que o usuário ajuste manualmente os ganhos.

- Simulação e **plot da resposta ao degrau unitário** em malha fechada:
  - Saída do processo (PV)  
  - Referência (SP)  
  - Sinal de controle (u)  

## Requisitos

Crie um ambiente e instale as dependências com:

```bash
pip install -r requirements.txt
