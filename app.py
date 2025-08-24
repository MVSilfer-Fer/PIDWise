import streamlit as st
import numpy as np

st.set_page_config(page_title="PIDWise", layout="centered")

# ---------------------------
# Utilidades
# ---------------------------
def imc_pid_foptd(K, tau, theta, lam):
    """
    IMC → PID (paralelo ISA dependente) para FOPTD: Gp(s)=K*e^{-θs}/(τ s + 1)
    Fórmulas (Rivera/IMC clássicas) usadas aqui:
        Kp = (tau + theta/2) / (K * (lam + theta/2))
        Ti = tau + theta/2
        Td = (tau * theta) / (2 * tau + theta)
    """
    Kp = (tau + theta/2.0) / (K * (lam + theta/2.0))
    Ti = tau + theta/2.0
    Td = (tau * theta) / (2.0 * tau + theta) if (2.0 * tau + theta) > 0 else 0.0
    return Kp, Ti, Td

def simulate_foptd_pid(K, tau, theta, Kp, Ti, Td, t_end=200.0, dt=0.05):
    """
    Simula resposta ao degrau unitário para:
      Planta FOPTD: dy/dt = (-y + K*u_delayed)/tau, com atraso θ
      Controlador PID paralelo ISA: u = Kp * [ e + (1/Ti) ∫e dt + Td de/dt ]
    Sem filtro de derivada, sem saturações (conforme solicitado).
    """
    n_steps = int(t_end / dt) + 1
    t = np.linspace(0, t_end, n_steps)

    # Atraso puro por fila de amostras
    delay_steps = max(0, int(np.round(theta / dt)))
    u_queue = [0.0] * (delay_steps + 1)

    y = np.zeros(n_steps)
    u = np.zeros(n_steps)
    e_prev = 0.0
    I = 0.0
    r = 1.0  # degrau unitário

    Ti_safe = Ti if Ti > 1e-9 else 1e9  # evita divisão por zero

    for k in range(1, n_steps):
        # Erro
        e = r - y[k-1]

        # Integração e derivada (paralelo ISA)
        I += e * dt
        D = (e - e_prev) / dt

        u[k] = Kp * (e + (I / Ti_safe) + Td * D)
        e_prev = e

        # Aplica atraso à entrada da planta
        u_queue.append(u[k])
        u_delayed = u_queue.pop(0)

        # Dinâmica de 1ª ordem
        dy = (-y[k-1] + K * u_delayed) / tau if tau > 1e-12 else (K * u_delayed)
        y[k] = y[k-1] + dt * dy

    return t, y

# ---------------------------
# UI
# ---------------------------
st.title("PIDWise")
st.caption("Inspirado em PID Tuner")

# 1) Tipo de processo
proc = st.selectbox("Tipo de processo", options=["FOPTD (Primeira ordem com tempo morto)"], index=0)

# 2) Parâmetros do processo
col1, col2, col3 = st.columns(3)
with col1:
    K = st.number_input("K (ganho do processo)", min_value=1e-6, value=1.0, step=0.1, format="%.6f")
with col2:
    tau = st.number_input("τ (constante de tempo)", min_value=1e-6, value=10.0, step=0.5, format="%.6f")
with col3:
    theta = st.number_input("θ (atraso de transporte)", min_value=0.0, value=2.0, step=0.5, format="%.6f")

# 3) Modelo de controlador
ctrl = st.selectbox("Modelo de controlador", options=["PID paralelo (ISA dependente)"], index=0)

# ---- Gerenciar estado do botão ----
if "started" not in st.session_state:
    st.session_state.started = False

if st.button("Avançar para resposta ao degrau"):
    st.session_state.started = True

# 5) Sintonia e simulação só aparecem depois que o botão for clicado 1x
if st.session_state.started:
    default_lambda = max(theta, 0.1 * tau)

    st.subheader("Ganho do PID (IMC como ponto de partida)")
    Kp0, Ti0, Td0 = imc_pid_foptd(K, tau, theta, default_lambda)

    c1, c2, c3, c4 = st.columns(4)
    with c4:
        lam = st.number_input("λ (IMC)", min_value=1e-6, value=float(default_lambda), step=0.1, format="%.6f")
    Kp0, Ti0, Td0 = imc_pid_foptd(K, tau, theta, lam)

    with c1:
        Kp = st.number_input("Kp", min_value=0.0, value=float(Kp0), step=0.05, format="%.6f")
    with c2:
        Ti = st.number_input("Ti", min_value=1e-9, value=float(Ti0), step=0.1, format="%.6f")
    with c3:
        Td = st.number_input("Td", min_value=0.0, value=float(Td0), step=0.05, format="%.6f")

    st.divider()

    # ---- Agora dá pra mudar t_end e dt sem precisar clicar em nada ----
    st.subheader("Resposta ao degrau unitário (PV)")
    t_end = st.number_input("Tempo de simulação (s)", min_value=1.0, value=200.0, step=10.0, format="%.1f")
    dt = st.number_input("Δt (passo de integração)", min_value=1e-3, value=0.05, step=0.01, format="%.3f")

    t, y = simulate_foptd_pid(K, tau, theta, Kp, Ti, Td, t_end=float(t_end), dt=float(dt))
    st.line_chart({"t (s)": t, "PV": y}, x="t (s)", y="PV")