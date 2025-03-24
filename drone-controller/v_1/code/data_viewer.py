import pandas as pd
import ace_tools as tools

# Extracted Data for Analysis
data = [
    {"K_p": 18, "K_d": 8, "K_R": 575, "K_omega": 20, "final_error": 0.002, "RMS_error": 0.446},
    {"K_p": 18, "K_d": 8, "K_R": 575, "K_omega": 25, "final_error": 0.006, "RMS_error": 0.442},
    {"K_p": 18, "K_d": 8, "K_R": 575, "K_omega": 30, "final_error": 0.006, "RMS_error": 0.432},
    {"K_p": 18, "K_d": 8, "K_R": 600, "K_omega": 20, "final_error": 0.003, "RMS_error": 0.454},
    {"K_p": 18, "K_d": 8, "K_R": 600, "K_omega": 25, "final_error": 0.006, "RMS_error": 0.447},
    {"K_p": 18, "K_d": 8, "K_R": 600, "K_omega": 30, "final_error": 0.006, "RMS_error": 0.436},
    {"K_p": 18, "K_d": 8, "K_R": 625, "K_omega": 20, "final_error": 0.004, "RMS_error": 0.463},
    {"K_p": 18, "K_d": 8, "K_R": 625, "K_omega": 25, "final_error": 0.006, "RMS_error": 0.452},
    {"K_p": 18, "K_d": 8, "K_R": 625, "K_omega": 30, "final_error": 0.006, "RMS_error": 0.441},
    {"K_p": 18, "K_d": 10, "K_R": 575, "K_omega": 20, "final_error": 0.006, "RMS_error": 0.464},
    {"K_p": 18, "K_d": 10, "K_R": 575, "K_omega": 25, "final_error": 0.007, "RMS_error": 0.453},
    {"K_p": 18, "K_d": 10, "K_R": 575, "K_omega": 30, "final_error": 0.007, "RMS_error": 0.447},
    {"K_p": 18, "K_d": 10, "K_R": 600, "K_omega": 20, "final_error": 0.007, "RMS_error": 0.473},
    {"K_p": 18, "K_d": 10, "K_R": 600, "K_omega": 25, "final_error": 0.008, "RMS_error": 0.460},
    {"K_p": 18, "K_d": 10, "K_R": 600, "K_omega": 30, "final_error": 0.008, "RMS_error": 0.451},
    {"K_p": 18, "K_d": 12, "K_R": 575, "K_omega": 20, "final_error": 0.015, "RMS_error": 0.433},
    {"K_p": 18, "K_d": 12, "K_R": 575, "K_omega": 25, "final_error": 0.015, "RMS_error": 0.422},
    {"K_p": 18, "K_d": 12, "K_R": 575, "K_omega": 30, "final_error": 0.015, "RMS_error": 0.411},
    {"K_p": 18, "K_d": 12, "K_R": 600, "K_omega": 20, "final_error": 0.014, "RMS_error": 0.437},
    {"K_p": 18, "K_d": 12, "K_R": 600, "K_omega": 25, "final_error": 0.015, "RMS_error": 0.426},
    {"K_p": 18, "K_d": 12, "K_R": 600, "K_omega": 30, "final_error": 0.015, "RMS_error": 0.415},
    {"K_p": 20, "K_d": 8, "K_R": 575, "K_omega": 20, "final_error": 0.002, "RMS_error": 0.445},
    {"K_p": 20, "K_d": 8, "K_R": 575, "K_omega": 25, "final_error": 0.002, "RMS_error": 0.446},
    {"K_p": 20, "K_d": 8, "K_R": 575, "K_omega": 30, "final_error": 0.002, "RMS_error": 0.436},
    {"K_p": 18, "K_d": 12, "K_R": 575, "K_omega": 30, "final_error": 0.001, "RMS_error": 0.324},
]

# Convert data into a DataFrame
df = pd.DataFrame(data)

# Display the DataFrame
tools.display_dataframe_to_user(name="PID Tuning Data", dataframe=df)