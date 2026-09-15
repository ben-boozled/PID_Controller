import pandas as pd
import matplotlib.pyplot as plt

### ============================ User Inputs ============================
filepath = "out/build/GCC 16.1.0 x86_64-w64-mingw32 (ucrt64)/altitude_sim.csv" # filepath to output csv
height_cols = ['Error (m)', 'Setpoint (m)', 'Altitude (m)'] 
force_cols = ['PID Out (N)', 'Thrust (N)']  

setpoint = 5                # m
errorBand = 0.15            # m
settlingTimestamp = 2.74    # s
settlingTime = 1.74         # s
holdTime = 1                # time before sending step signal for setpoint, s

expectedHoverThrust = 4.91  # N

### =========================== Plot CSV Data =========================== 
df = pd.read_csv(filepath)

fig, axs = plt.subplots(2, 1, figsize=(8, 8))

### Height Subplot ###
for i in height_cols:
    axs[0].plot(df['Time (s)'], df[i], label=i)

lineStyleFormat = {"color": "red", 
                   "linestyle": "--", 
                   "linewidth": 0.5}

textStyleFormat = {"color": "red",
                   "size": 8}

# Markings for settling time 
axs[0].axvline(x=settlingTimestamp, **lineStyleFormat)
axs[0].text(settlingTimestamp+0.2, 0, "t = " + str(settlingTimestamp) + "s", **textStyleFormat)
axs[0].axvline(x=holdTime, **lineStyleFormat)
axs[0].text(holdTime+0.2, 0, "t = " + str(holdTime) + "s", **textStyleFormat)
axs[0].hlines(xmin=holdTime, xmax=settlingTimestamp, y=1, **lineStyleFormat)
axs[0].text(holdTime + 0.05, 1.05, "Settling time = " + str(settlingTime) + "s",
            **textStyleFormat)

# Markings for settling altitude 
axs[0].axhline(y=setpoint+errorBand, **lineStyleFormat)
axs[0].text(0, setpoint+errorBand+0.05, "z_high = " + str(setpoint+errorBand) + "m", 
            **textStyleFormat)
axs[0].axhline(y=setpoint-errorBand, **lineStyleFormat)
axs[0].text(0, setpoint-errorBand+0.05, "z_low = " + str(setpoint-errorBand) + "m", 
            **textStyleFormat)

axs[0].set_xlabel("Time (s)")
axs[0].set_ylabel("Height (m)")

axs[0].legend()

### Force & Velocity Subplot ###
for i in force_cols:
    axs[1].plot(df['Time (s)'], df[i], label=i)

axs[1].axhline(y=expectedHoverThrust, **lineStyleFormat)
axs[1].text(0, expectedHoverThrust+0.05, "Expected Hover Thrust = " 
            + str(expectedHoverThrust) + "N", **textStyleFormat)

axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("Force (N)")

axs[1].legend()

axs1_2 = axs[1].twinx()
axs1_2 .plot(df['Time (s)'], df['Velocity (m/s)'], label='Velocity (m/s)',
             color="green")
axs1_2.set_ylabel("Velocity (m/s)")

handles1, labels1 = axs[1].get_legend_handles_labels()
handles1_2, labels1_2 = axs1_2.get_legend_handles_labels()

axs[1].legend(handles1+handles1_2, labels1+labels1_2)

### Format and Show the Entire Plot
fig.suptitle("PID Simulation Results")
plt.tight_layout()
plt.show()