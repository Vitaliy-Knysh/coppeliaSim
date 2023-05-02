import cv2
import numpy as np
from math import sin, cos, radians
from zmqRemoteApi import RemoteAPIClient
from threading import Thread
import time

client = RemoteAPIClient()
sim = client.getObject('sim')

defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
sim.setInt32Param(sim.intparam_idle_fps, 0)

FUZZY_CLOSE = 1  # параметры для нечеткой логики
FUZZY_MIDDLE = 2
FUZZY_FAR = 3
VIS1 = sim.getObject('./LIDAR/sensor1')
VIS2 = sim.getObject('./LIDAR/sensor2')
MEASURES_PER_TICK = 60  # количество измерений за 1 тик для 1 сканера
ANGLE_INCREMENT = 120 / (MEASURES_PER_TICK - 1)
WINDOW_SIZE_PX = 500
WINDOW_CENTER = WINDOW_SIZE_PX // 2
MAX_MEASURE_DIST = 5

tick_ctr = 0

points_x = [0]
points_y = [0]
img = np.zeros([WINDOW_SIZE_PX, WINDOW_SIZE_PX, 1], dtype=np.uint8)
img.fill(255)


def scan():
    global tick_ctr
    sens1_raw = sim.readVisionSensor(VIS1)
    sens2_raw = sim.readVisionSensor(VIS2)
    if tick_ctr > 3:  # первые 1-3 тика датчик не сканирует
        x1, y1 = process_raw_measures(sens1_raw, -210)
        x2, y2 = process_raw_measures(sens2_raw, -90)
        return x1 + x2, y1 + y2
    else: return [], []


def process_raw_measures(data, start_angle):
    x = []
    y = []
    for ctr, measure_raw in enumerate(data[2][2:]):  # первые 2 числа не относятся к результатам измерения
        if (ctr + 1) % 4 == 0:
            if measure_raw < MAX_MEASURE_DIST:
                angle = start_angle + (ANGLE_INCREMENT * (ctr - 3) / 4)
                x.append(measure_raw * cos(radians(angle)))
                y.append(measure_raw * sin(radians(angle)))
    return x, y


def draw_points():
    while True:
        img.fill(255)
        cv2.rectangle(img, (WINDOW_CENTER - 5, WINDOW_CENTER - 5), (WINDOW_CENTER + 5, WINDOW_CENTER + 5),
                              (200, 0, 200), 5)
        for index, point in enumerate(points_x):
            cv2.circle(img, (WINDOW_CENTER + int(point * 50), WINDOW_CENTER + int(points_y[index] * 50)), 2,
                       (0, 0, 255), 2)
        cv2.imshow('graph', img)
        if cv2.waitKey(1) & 0xff == ord('q'):
            cv2.destroyAllWindows()
        time.sleep(0.1)


draw_thread = Thread(target=draw_points)


if __name__ == '__main__':
    client.setStepping(True)  # НАЧАЛО РАБОТЫ ПРОГРАММЫ - ЗАПУСК СИМУЛЯЦИИ
    sim.startSimulation()
    draw_thread.start()
    while (t := sim.getSimulationTime()) >= 0:
        points_x, points_y = scan()
        tick_ctr += 1
        client.step()  # triggers next simulation step

    sim.stopSimulation()  ### КОНЕЦ СИМУЛЯЦИИ
    # Restore the original idle loop frequency:
    sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)
