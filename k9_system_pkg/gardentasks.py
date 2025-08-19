#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
from datetime import date, datetime

from k9_interfaces_pkg.srv import GetGardenTasks, GetWeatherProfile  # assumes both srv files exist

def to_int_or_none(value):
    return int(value) if value.strip() else None


class GardenTaskServiceNode(Node):
    def __init__(self):
        super().__init__("task_service")

        # Load tasks once from CSV
        self.tasks = self.load_tasks("./resource/Garden_and_Pond_Tasks.csv")

        # Create service
        self.srv = self.create_service(GetGardenTasks, "get_tasks", self.handle_get_tasks)

        # Client for weather service
        self.weather_client = self.create_client(GetWeatherProfile, "get_weather_profile")

        self.get_logger().info("Task service ready.")

    def load_tasks(self, filepath):
        tasks = []
        with open(filepath, newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            for row in reader:
                months = [month for month in reader.fieldnames[4:] if row[month].strip()]
                task = {
                    "name": row["Task"],
                    "low_c": to_int_or_none(row["Low_C"]),
                    "high_c": to_int_or_none(row["High_C"]),
                    "rain": bool(int(row["Rain"])),  # expect 0 or 1 in CSV
                    "months": months,
                }
                tasks.append(task)
        return tasks

    def handle_get_tasks(self, request, response):
        # 1. Get weather profile for 'home'
        weather = self.call_weather_service("home")
        if not weather:
            self.get_logger().warn("No weather data, returning empty task list")
            response.summaries = []
            return response

        # 2. Work out suitable tasks
        today = date.today()
        todays_month = today.strftime("%B")

        suitable_tasks_by_hour = {}
        for hour, temp, precip in zip(weather.hours, weather.feels_like, weather.precip):
            raining = precip > 0.0
            suitable_tasks = []
            for task in self.tasks:
                if todays_month in task["months"] and task["rain"] == raining:
                    if (task["low_c"] is None or task["low_c"] <= temp) and (
                        task["high_c"] is None or task["high_c"] >= temp
                    ):
                        suitable_tasks.append(task["name"])
            suitable_tasks_by_hour[hour] = suitable_tasks or []

        # 3. Summarise without redundancy
        summaries = self.make_summary(suitable_tasks_by_hour)

        response.summaries = summaries
        return response

    def call_weather_service(self, location: str):
        if not self.weather_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Weather service not available")
            return None

        req = GetWeatherProfile.Request()
        req.location = location
        future = self.weather_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("Weather service call failed")
            return None
        return future.result()

    def make_summary(self, suitable_tasks_by_hour):
        mentioned_tasks = set()
        summary_lines = []
        current_hour = datetime.now().hour

        for hour in sorted(suitable_tasks_by_hour):
            if hour < current_hour:
                continue
            tasks_this_hour = [t for t in suitable_tasks_by_hour[hour] if t not in mentioned_tasks]
            if not tasks_this_hour:
                continue
            mentioned_tasks.update(tasks_this_hour)

            # Determine time string
            if hour == current_hour:
                time_str = "now"
            else:
                # Look ahead for same tasks in contiguous hours
                next_hours = range(hour + 1, max(suitable_tasks_by_hour.keys()) + 1)
                until_hour = None
                for nh in next_hours:
                    if any(t in suitable_tasks_by_hour[nh] for t in tasks_this_hour):
                        until_hour = nh
                    else:
                        break
                if until_hour:
                    time_str = f"{hour}:00 until {until_hour}:00"
                else:
                    time_str = f"{hour}:00"

            summary_lines.append(f"{', '.join(tasks_this_hour)} can be done from {time_str}")

        return summary_lines


def main(args=None):
    rclpy.init(args=args)
    node = GardenTaskServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()