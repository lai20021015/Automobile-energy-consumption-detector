import Ed2.optimizer as optimizer
import fastsim as fsim
import numpy as np
import csv

# Function to generate random parameters
def generate_random_parameters():
    # Example: Adjust ranges based on your simulator's requirements
    constant_speed_mps = np.random.uniform(0.5, 2.0)  # Speed in m/s
    simulation_duration_s = np.random.uniform(100, 500)  # Total simulation time in seconds
    road_grade_factor = np.random.uniform(0.0, 1.0)  # Road grade factor
    return [constant_speed_mps, simulation_duration_s, road_grade_factor]

# Create dataset
data = []
for _ in range(100000):
    if _ % 10000 ==0:
        print(f"Generating random parameters...{_}th iteration")
    params = generate_random_parameters()
    
    # Extract parameters for simulation
    time_s = np.linspace(0, params[1], int(params[1]))  # Example time array
    speed_mps = np.full_like(time_s, params[0])  # Example constant speed array
    
    # Initialize the optimizer
    opt = optimizer.TrainEnergyOptimizer(
        distance_m=1000.0,  # Example distance
        time_s=params[1],   # Use the generated simulation duration
        max_speed_mps=40, # Example max speed
        veh_id=43           # Use the valid vehicle ID
    )
    
    # Perform energy simulation
    cyc = fsim.cycle.Cycle(
        time_s=time_s,
        mps=speed_mps,
        grade=np.zeros_like(time_s),
        road_type="default",  # 假設的預設值
        name="simulation"     # 假設的名稱
    )
    sim = fsim.simdrive.SimDrive(cyc, opt.veh)  # Use the vehicle from the optimizer
    sim.sim_drive()
    energy_consumption = sim.ess_cur_kwh[0] - sim.ess_cur_kwh[-1]
    
    data.append(params + [energy_consumption])

# Save to CSV
output_file = "simulator_dataset.csv"
with open(output_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["ConstantSpeed_mps", "SimulationDuration_s", "RoadGradeFactor", "EnergyConsumption"])  # Header
    writer.writerows(data)

print(f"Dataset saved to {output_file}")