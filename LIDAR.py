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
min_point_coord = [1, 1]
img = np.zeros([WINDOW_SIZE_PX, WINDOW_SIZE_PX, 1], dtype=np.uint8)
img.fill(255)


def scan():
    """
    Получает данные с дальномера, передает другим функциям для обработки, выдает результат в Декартовых координатах

    :return: X-координаты точек, Y-координаты точек, индекс ближайшей точки, угол до ближайшей точки
    """
    global tick_ctr
    if tick_ctr > 3:  # первые 1-3 тика датчик не сканирует
        sens1_raw = sim.readVisionSensor(VIS1)[2][2:]  # первые 2 числа это не данные сканирования
        sens2_raw = sim.readVisionSensor(VIS2)[2][2:]
        x1, y1, min_index1, min_distance1, min_angle1 = process_raw_measures(sens1_raw, 120)
        x2, y2, min_index2, min_distance2, min_angle2 = process_raw_measures(sens2_raw, 0)
        if min_distance1 < min_distance2:
            min_index_total = min_index1
            min_angle_total = min_angle1
        else:
            min_index_total = min_index2
            min_angle_total = min_angle2
        return x1 + x2, y1 + y2, min_index_total, min_angle_total
    else: return [], [], [], []


def process_raw_measures(data, start_angle):
    """
    ДЛЯ ОДНОГО ДАЛЬНОМЕРА: Переводит измерения из полярных координат в Декартовы, отсеивает измерения
    за пределами видимости дальномера

    :param data: исходные данные с дальномера
    :param start_angle: угол, с которого начинается сканирование
    :return: X-координаты точек, Y-координаты точек, индекс ближайшей точки, угол до ближайшей точки
    """
    x = []
    y = []
    min_point_index = 0
    min_point_distance = 10
    min_point_angle = 0

    for index, measure_raw in enumerate(data[3::4]):  # каждое измерение выдает 4 числа, нам нужно 4-е
        if measure_raw < MAX_MEASURE_DIST:
            angle = start_angle - ANGLE_INCREMENT * index
            x.append(measure_raw * cos(radians(angle)))
            y.append(measure_raw * sin(radians(angle)))

            if measure_raw < min_point_distance:
                min_point_distance = measure_raw
                min_point_index = len(x) - 1
                min_point_angle = angle

    return x, y, min_point_index, min_point_distance, min_point_angle


def draw_points():
    """
    Отрисовывает окно на OpenCV c отсканированными точками
    """
    while True:
        img.fill(255)
        cv2.rectangle(img, (WINDOW_CENTER - 5, WINDOW_CENTER - 5), (WINDOW_CENTER + 5, WINDOW_CENTER + 5),
                              (200, 0, 200), 5)
        for index, point in enumerate(points_x):
            cv2.circle(img, (WINDOW_CENTER + int(point * 100), WINDOW_CENTER + int(points_y[index] * 100)), 2,
                       (0, 0, 255), 2)
        #      ЭТА ЛИНИЯ СОЕДИНЯЕТ ЦЕНТР И БЛИЖАЙШУЮ ТОЧКУ
        # cv2.line(img, (WINDOW_CENTER, WINDOW_CENTER), ((WINDOW_CENTER + int(min_point_coord[0] * 100), WINDOW_CENTER + int(min_point_coord[1] * 100))),
        #            (0, 0, 255), 3)

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
        points_x, points_y, min_index, min_angle = scan()
        if tick_ctr > 3:
            min_point_coord = [points_x[min_index], points_y[min_index]]

        tick_ctr += 1
        client.step()  # triggers next simulation step

    sim.stopSimulation()  ### КОНЕЦ СИМУЛЯЦИИ
    # Restore the original idle loop frequency:
    sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)
