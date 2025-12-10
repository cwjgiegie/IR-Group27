import re
import pandas as pd
import matplotlib.pyplot as plt

log_path = r"C:\Users\Kevin\Desktop\table2.txt"

rows = []
pattern = re.compile(
    r"\[t=(\d+)\]\s+pos=\(([-\d.]+),([-\d.]+)\)\s+theta=([-\d.]+)\s+"
    r"HL_target=([A-Z]+)\s+dist=([-\d.]+)\s+batt=([-\d.]+)\s+"
    r"danger=([-\d.]+)\s+Lobs=([-\d.]+)\s+Robs=([-\d.]+)\s+behavior=([A-Z]+)"
)

with open(log_path, "r", encoding="utf-8") as f:
    for line in f:
        m = pattern.search(line)
        if m:
            t, x, y, theta, hl, dist, batt, danger, lobs, robs, beh = m.groups()
            rows.append({
                "t": int(t),
                "x": float(x),
                "y": float(y),
                "theta": float(theta),
                "HL_target": hl,
                "dist": float(dist),
                "batt": float(batt),
                "danger": float(danger),
                "Lobs": float(lobs),
                "Robs": float(robs),
                "behavior": beh
            })

df = pd.DataFrame(rows)
print(df.head())
print("totolly:", len(df))

plt.figure()
plt.plot(df["t"], df["dist"])
plt.xlabel("Simulation step t")
plt.ylabel("Distance to target (m)")
plt.title("Distance to target over time")
plt.tight_layout()



# show
plt.show()
plt.close()

# ===== 画 danger 曲线 =====
plt.figure()
plt.plot(df["t"], df["danger"], label="Danger level", color="red")
plt.xlabel("Simulation step t")
plt.ylabel("Danger value (0~1)")
plt.title("Obstacle Danger Over Time")
plt.grid(True)
plt.tight_layout()


# show
plt.show()
plt.close()
#--------------------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------------------
import re
import pandas as pd
import matplotlib.pyplot as plt

# ========= fun =========
pattern = re.compile(
    r"\[t=(\d+)\]\s+pos=\(([-\d.]+),([-\d.]+)\)\s+theta=([-\d.]+)\s+"
    r"HL_target=([A-Z]+)\s+dist=([-\d.]+)\s+batt=([-\d.]+)\s+"
    r"danger=([-\d.]+)\s+Lobs=([-\d.]+)\s+Robs=([-\d.]+)\s+behavior=([A-Z]+)"
)

def load_log(path):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            m = pattern.search(line)
            if m:
                t, x, y, theta, hl, dist, batt, danger, lobs, robs, beh = m.groups()
                rows.append({
                    "t": int(t),
                    "dist": float(dist),
                    "danger": float(danger),
                    "behavior": beh
                })
    return pd.DataFrame(rows)

# ========= read file =========
df_success = load_log(r"C:\Users\Kevin\Desktop\table.txt")
df_failure = load_log(r"C:\Users\Kevin\Desktop\table2.txt")

# ========= =========
behavior_map = {
    "GOAL": 0,
    "AVOID": 1,
    "CHARGE": 2
}

def behavior_to_int(df):
    df["behavior_code"] = df["behavior"].map(behavior_map)
    return df

df_success = behavior_to_int(df_success)
df_failure = behavior_to_int(df_failure)

plt.figure(figsize=(10, 5))

# successful curve
plt.step(df_success["t"], df_success["behavior_code"],
         where="post", label="Successful Run", linewidth=2)

# field curve
plt.step(df_failure["t"], df_failure["behavior_code"],
         where="post", label="Failed Run", linestyle="--", linewidth=2, color="orange")

plt.yticks([0, 1, 2], ["GOAL", "AVOID", "CHARGE"])
plt.xlabel("Simulation step")
plt.ylabel("Behavior State")
plt.title("Behavior State Transition Comparison (Success vs Failure)")
plt.legend()
plt.grid(alpha=0.3)

plt.tight_layout()
plt.savefig("behavior_compare_overlap.png", dpi=300)
plt.show()
