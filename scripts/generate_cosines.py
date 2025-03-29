import numpy as np

WINDOW_SIZE = 32  
PI = np.pi

cos_table = np.array([
    [np.cos(PI / WINDOW_SIZE * (n + 0.5) * k) for n in range(WINDOW_SIZE)]
    for k in range(WINDOW_SIZE)
])

with open("cos_table.txt", "w") as file:
    file.write("{\n")
    for row in cos_table:
        file.write("    {" + ", ".join(f"{val:.10f}" for val in row) + "},\n")
    file.write("}\n")

print("cos_table.txt is ready!")