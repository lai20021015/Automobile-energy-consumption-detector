import pygame
import numpy as np
from vehicle_model import Vehicle
from optimizer import TrainEnergyOptimizer
from visualization import draw_comparison_graphs

pygame.init()
width, height = 1200, 800
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("車輛加減速最佳化模擬器")
WHITE, BLACK = (255, 255, 255), (0, 0, 0)
vehicle = Vehicle()
optimizer = TrainEnergyOptimizer()
results = optimizer.optimize()
time_optimal, speed_optimal_time = results['optimal_time'], results['optimal_speed']
distance_optimal = np.cumsum(speed_optimal_time) / 3.6

def draw_dashboard():
    screen.fill(WHITE)
    pygame.draw.line(screen, BLACK, (50, height - 100), (width - 50, height - 100), 5)
    car_x = 50 + (width - 100) * (vehicle.position / distance_optimal[-1])
    pygame.draw.rect(screen, (0, 0, 255), (car_x - 20, height - 130, 40, 20))
    font = pygame.font.Font(None, 36)
    texts = [
        f"time: {vehicle.time:.1f} secs",
        f"velocity: {vehicle.speed:.1f} km/h",
        f"location: {vehicle.position:.1f} m",
        f"energy: {vehicle.energy_consumption:.1f} KW"
    ]
    for i, text in enumerate(texts):
        screen.blit(font.render(text, True, BLACK), (50, 50 + i * 50))
    draw_comparison_graphs(screen, width, vehicle, time_optimal, speed_optimal_time, distance_optimal)

running, clock = True, pygame.time.Clock()
while running:
    dt = 1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    keys = pygame.key.get_pressed()
    acceleration = 10.0 if keys[pygame.K_UP] else (-10.0 if keys[pygame.K_DOWN] else 0)
    vehicle.update(dt, acceleration)
    if vehicle.position >= distance_optimal[-1]:
        print(f"arrived at destination! Total time: {vehicle.time:.1f} s, energy: {vehicle.energy_consumption:.1f} kW")
        running = False
    draw_dashboard()
    pygame.display.flip()
    clock.tick(10)
pygame.quit()
