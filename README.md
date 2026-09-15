# Background
PID (proportional-integral-derivative) controllers are integral to the reliable operation of numerous industrial systems. Despite this, there is a surprisingly disproportionate lack of resources demonstrating their software implementation from the ground up; the mechanisms of PID controllers (and how to manage their real-world limitations) are often hidden behind prebuilt function calls and model objects.

While I started this project because I thought it would be fun to implement a PID controller from scratch, I hope that anyone wondering how to translate PID theory to implementation will find the resources here helpful.  

The derivations for the discretised PID's difference equations have been included in `Discrete_PID_Derivation.md`. It might be helpful to review that document alongside the code.

# Overview
The code files in this repository are as follows:
* `include/PID_Controller/pid.h` and `src/pid.c` implement the discrete time PID controller. **Integral clamping** and **derivative on measurement** features have been implemented to safeguard against integral windups and derivative kicks respectively.
* `src/main.c` simulates the step response of the PID controller when a quadcopter receives a step signal change to its altitude setpoint (unfortunately, this simulated quadcopter can only fly up or down). The simulation employs helper functions from `include/PID_Controller/altitude_sim.h` and `src/altitude_sim.h`. The results are written to an automatically generated `altitude_sim.csv`.
* `plot_csv.py` generates plots using the data in `altitude_sim.csv` (see image below). 

![Simulation results from running plot_csv.py](https://github.com/ben-boozled/PID_Controller/blob/main/Images/sim_results.png)

In the second subplot, the line graph of the PID output is hidden behind that of the quadcopter's thrust force because they align with one another. These line graphs may not always align if, for example, the PID output exceeds the operational limits of the quadcopter's propeller; the limits are defined by C macros in `src/main.c`.

# Interpreting the PID Output
The interpretation of a PID output depends on how the controller was designed. In `src/main.c`, the units and values for $K_{p}$, $K_{i}$, and $K_{d}$ were chosen such that the PID returns the required combined thrust force of the quadcopter's propellers; the input saturation limits of the PID are in Newtons to match the output control signal units. In industrial implementations, PID outputs are often expressed as percentages or digital values. These values would be passed into a separate function call, mapping them to the final control element (e.g. pulse-width modulation) driving the actuator. 

# Dependencies
The `.c` source and `.h` header files do not require dependencies beyond the **C Standard Library** (this is automatically installed whenever you install any C compiler toolchain). However, if you intend to run `plot_csv.py` to generate the simulation plots, you will need to install the [Pandas](https://pandas.pydata.org/) and [Matplotlib](https://matplotlib.org/) Python libraries.
