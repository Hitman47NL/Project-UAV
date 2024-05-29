# -*- coding: utf-8 -*-
"""
Simulatie van plaats s en snelheid v van een massa m en een stuwkracht Fx.

Bewegingsvergelijking:
    Fx = m * a

Gemaakt op 25-05-2024

Auteurs: Ernst Kouwe en Mark Schelbergen
"""

import numpy as np
import matplotlib.pyplot as plt

# Systeem parameters
m = 0.8  # Massa van de UAV in [kg]
Fx_max = 5  # Maximale stuwkracht propeller voor meer realistische simulatie [N]
Fx_min = -Fx_max  # Minimale stuwkracht propeller voor meer realistische simulatie [N]

# Simulatie configuratie
t_eind = 10  # Lengte van simulatie in [s]
dt = .01  # Stapgrootte voor de simulatie in [s]
tijden = np.arange(0, t_eind, dt)  # Array met tijdstappen

s0 = 0  # Beginafstand in [m]
v0 = 0  # Beginsnelheid in [m/s]

sp = 1  # Setpoint voor afstand in [m]


def pid_regelaar(x, error_oud, error_som, gains):
    """
    Berekent de output van een PID-regelaar.

    Parameters:
        x (float): Geregelde grootheid.
        error_oud (float): De fout van de vorige tijdstap.
        error_som (float): De som van de fouten tot nu toe.
        gains (tuple): Een tuple bestaande uit drie gains (Kp, Ki, Kd) voor de PID-regelaar.

    Returns:
        tuple:
            - regelaar_output (float): De berekende regelaar actie.
            - error (float): De huidige fout.
            - error_som (float): De bijgewerkte som van de fouten.
    """
    Kp, Kd, Ki = gains

    error = sp - x  # Ingestelde waarde - geregelde grootheid
    d_error = error - error_oud
    error_som = error_som + error * dt
    regelaar_output = Kp * error# + Kd * d_error / dt + Ki * error_som
    return regelaar_output, error, error_som


def run_simulatie(gains):
    """
    Voert een simulatie uit met een PID-regelaar welke de kracht berekenent die nodig is
    om een systeem naar een gewenst punt te brengen.

    Parameters:
    gains (tuple): Een tuple bestaande uit drie gains (Kp, Ki, Kd) voor de PID-regelaar.

    Returns:
    tuple:
        - plaatsen (list): Een lijst met de positie van het systeem op elk tijdstip.
        - snelheden (list): Een lijst met de snelheid van het systeem op elk tijdstip.
        - versnellingen (list): Een lijst met de versnelling van het systeem op elk tijdstip.
        - krachten (list): Een lijst met de kracht die op het systeem wordt uitgeoefend op elk tijdstip.

    De functie simuleert de beweging van een systeem over een reeks tijdstippen.
    Het gebruikt een PID-regelaar om de gewenste kracht te berekenen om het systeem naar
    een gewenst punt te sturen. De kracht wordt vervolgens beperkt tot realistische waarden
    en wordt gebruikt om de versnelling, snelheid en positie van het systeem bij elk tijdstip te berekenen.
    """

    # Variabelen om resultaten weg te schrijven
    error_oud = 0
    error_som = 0
    plaatsen = [s0]
    snelheden = [v0]
    versnellingen = []
    krachten = []

    for ti in tijden:
        s_oud = plaatsen[-1]
        v_oud = snelheden[-1]

        # Bereken de gewenste kracht met de PID regelaar
        Fx, error_oud, error_som = pid_regelaar(s_oud, error_oud, error_som, gains)

        # Begrens de kracht naar realistische waarde
        if Fx > Fx_max:
            Fx = Fx_max
        elif Fx < Fx_min:
            Fx = Fx_min
        krachten.append(Fx)  # Wegschrijven kracht

        # Bereken bijbehorende versnelling
        a = Fx / m
        versnellingen.append(a)

        # Integreren van versnelling om snelheid en plaats te berekenen
        v = v_oud + a * dt
        snelheden.append(v)
        s = s_oud + v_oud * dt
        plaatsen.append(s)

    return plaatsen, snelheden, versnellingen, krachten


def plot_resultaten(plaatsen, snelheden, versnellingen, krachten, gains, linestyle='-', ax=None):
    """
    Maakt een plot van de resultaten van een simulatie.

    Parameters:
    plaatsen (list): Een lijst met de positie van het systeem op elk tijdstip.
    snelheden (list): Een lijst met de snelheid van het systeem op elk tijdstip.
    versnellingen (list): Een lijst met de versnelling van het systeem op elk tijdstip.
    krachten (list): Een lijst met de kracht die op het systeem wordt uitgeoefend op elk tijdstip.
    gains (tuple): Een tuple bestaande uit drie gains (Kp, Ki, Kd) voor de PID-regelaar.
    linestyle (str, optioneel): De stijl van de lijnen in de plot, standaard is '-' (doorgetrokken lijn).
    ax (matplotlib Axes, optioneel): Een Axes-object om op te plotten. Bij None, wordt er een nieuwe figuur aangemaakt.

    Returns:
    ax (matplotlib Axes): De Axes-objecten met de gemaakte plot.

    De functie maakt een figuur met vier subplots (kracht, versnelling, snelheid en plaats)
    van de simulatiegegevens. Als er geen Axes-object wordt meegegeven, maakt de functie
    een nieuwe figuur en Axes-objecten aan. De grafieken worden geplot met de tijd als
    de x-as en de bijbehorende data (kracht, versnelling, snelheid en plaats) op de y-as.
    """
    if ax is None:
        fig, ax = plt.subplots(4, 1, sharex=True, figsize=[4.8, 6.4])
        plt.subplots_adjust(top=0.97, bottom=0.08, left=0.15, right=0.97)
        for a in ax: a.grid()
        ax[0].set_ylabel('Kracht [N]')
        ax[1].set_ylabel('Versnelling [m/s$^2$]')
        ax[2].set_ylabel('Snelheid [m/s]')
        ax[3].set_ylabel('Plaats [m]')
        ax[-1].set_xlabel('Tijd [s]')
        ax[-1].set_xlim([tijden[0] - .1, tijden[-1] + .1])
    lbl = f"K$_p$={gains[0]:.1f};K$_d$={gains[1]:.1f};K$_i$={gains[2]:.1f}"
    ax[0].plot(tijden, krachten, linestyle, label=lbl)
    ax[0].legend()
    ax[1].plot(tijden, versnellingen, linestyle)
    ax[2].plot(tijden, snelheden[:-1], linestyle)
    ax[3].plot(tijden, plaatsen[:-1], linestyle)
    return ax


if __name__ == '__main__':
    # Controller parameters via poolplaatsing voor PD-regelaar    
    Re = 1
    Im = 2
    Kp = (Re * Re + Im * Im) * m
    Kd = 2 * Re * m
    Kp = 1  # Proportionele gain
    Kd = 1  # DifferentiÃ«le gain
    Ki = 0  # Integrale gain
    gains_set1 = (Kp, Kd, Ki)

    plaatsen1, snelheden1, versnellingen1, krachten1 = run_simulatie(gains_set1)
    ax = plot_resultaten(plaatsen1, snelheden1, versnellingen1, krachten1, gains_set1)

    Ki = .2  # Integrale gain
    gains_set2 = (Kp, Kd, Ki)

    plaatsen2, snelheden2, versnellingen2, krachten2 = run_simulatie(gains_set2)
    plot_resultaten(plaatsen2, snelheden2, versnellingen2, krachten2, gains_set2, '--', ax)

    plt.show()