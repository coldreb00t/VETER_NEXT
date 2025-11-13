#!/usr/bin/env python3
"""
TensorRT Benchmark - измерение чистой скорости inference без ROS2 overhead
"""
import time
import numpy as np
from ultralytics import YOLO
import cv2

def benchmark_tensorrt(engine_path, num_iterations=100, warmup=10):
    """
    Бенчмарк TensorRT engine на синтетических изображениях

    Args:
        engine_path: Путь к TensorRT .engine файлу
        num_iterations: Количество итераций для измерения
        warmup: Количество warmup итераций
    """
    print(f"Загрузка TensorRT engine: {engine_path}")
    model = YOLO(engine_path)

    # Создать тестовое изображение 640x480 (как в камере)
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    print(f"\nWarmup: {warmup} итераций...")
    for i in range(warmup):
        results = model.predict(test_image, verbose=False)
        if (i + 1) % 5 == 0:
            print(f"  Warmup {i+1}/{warmup}")

    print(f"\nБенчмарк: {num_iterations} итераций...")
    times = []

    for i in range(num_iterations):
        start_time = time.perf_counter()
        results = model.predict(test_image, verbose=False)
        end_time = time.perf_counter()

        inference_time = (end_time - start_time) * 1000  # ms
        times.append(inference_time)

        if (i + 1) % 20 == 0:
            print(f"  Итерация {i+1}/{num_iterations}: {inference_time:.2f} ms")

    # Статистика
    times = np.array(times)
    avg_time = np.mean(times)
    min_time = np.min(times)
    max_time = np.max(times)
    std_time = np.std(times)
    fps = 1000.0 / avg_time

    print("\n" + "="*60)
    print("РЕЗУЛЬТАТЫ BENCHMARK:")
    print("="*60)
    print(f"Среднее время inference: {avg_time:.2f} ms")
    print(f"Минимальное время:        {min_time:.2f} ms")
    print(f"Максимальное время:       {max_time:.2f} ms")
    print(f"Стандартное отклонение:   {std_time:.2f} ms")
    print(f"\n🚀 МАКСИМАЛЬНАЯ СКОРОСТЬ: {fps:.1f} FPS")
    print("="*60)

    return fps

if __name__ == "__main__":
    engine_path = "/home/jetson/jetson-robot-project/ros2_ws/yolov8n.engine"
    fps = benchmark_tensorrt(engine_path, num_iterations=100, warmup=10)

    print("\n💡 Сравнение:")
    print(f"   TensorRT (чистый):  {fps:.1f} FPS")
    print(f"   ROS2 + Camera:      ~4 FPS")
    print(f"   Overhead ROS2:      {fps - 4:.1f} FPS потеряно!")
