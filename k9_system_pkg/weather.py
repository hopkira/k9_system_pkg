#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from datetime import datetime, timezone
from zoneinfo import ZoneInfo
import requests
import time

from k9_system_pkg.my_secrets import MET_API_KEY
from k9_interfaces_pkg.srv import GetWeatherProfile  # generated from .srv

uk_tz = ZoneInfo("Europe/London")

BASE_URL = "https://data.hub.api.metoffice.gov.uk/sitespecific/v0/point/"
TIMESTEPS = "hourly"
INCLUDE_LOCATION = False
EXCLUDE_METADATA = True

# Define the places you care about
LOCATIONS = {
    "home": (54.5043, -1.3399),
    "coatham": (54.5326, -1.3823),
    "london": (51.5032, -0.1162),
}

class WeatherServiceNode(Node):
    def __init__(self):
        super().__init__("weather_service")

        # Cache for all locations
        self.weather_profiles = {name: {} for name in LOCATIONS}
        self.last_updated = {name: None for name in LOCATIONS}

        # Update hourly
        self.timer = self.create_timer(3600.0, self.update_all_forecasts)
        self.update_all_forecasts()

        # Service
        self.srv = self.create_service(GetWeatherProfile, "get_weather_profile", self.handle_get_weather)

    def handle_get_weather(self, request, response):
        """Return cached forecast for requested location."""
        loc = request.location.lower()

        if loc not in self.weather_profiles:
            self.get_logger().warn(f"Unknown location '{loc}' requested")
            response.hours = []
            response.feels_like = []
            response.precip = []
            return response

        profile = self.weather_profiles[loc]
        if not profile:
            self.get_logger().warn(f"No weather data cached for '{loc}'")
            response.hours = []
            response.feels_like = []
            response.precip = []
            return response

        response.hours = list(profile.keys())
        response.feels_like = [v[0] for v in profile.values()]
        response.precip = [float(v[1]) for v in profile.values()]
        return response

    def update_all_forecasts(self):
        """Fetch forecasts for all configured locations."""
        for name, (lat, lon) in LOCATIONS.items():
            self.get_logger().info(f"Fetching forecast for {name}...")
            profile = self.retrieve_forecast(lat, lon)
            if profile:
                self.weather_profiles[name] = profile
                self.last_updated[name] = datetime.now(uk_tz)
                self.get_logger().info(f"Updated weather for {name} with {len(profile)} entries.")

    def retrieve_forecast(self, latitude, longitude):
        """Fetch and parse forecast for one location."""
        headers = {"accept": "application/json", "apikey": MET_API_KEY}
        params = {
            "excludeParameterMetadata": EXCLUDE_METADATA,
            "includeLocationName": INCLUDE_LOCATION,
            "latitude": latitude,
            "longitude": longitude,
        }

        retries = 3
        data = None
        while retries > 0 and data is None:
            try:
                req = requests.get(BASE_URL + TIMESTEPS, headers=headers, params=params, timeout=10)
                req.encoding = "utf-8"
                data = req.json()
            except Exception as e:
                self.get_logger().warn(f"Weather fetch failed for {latitude},{longitude} ({e}), retrying...")
                retries -= 1
                time.sleep(5)

        if not data:
            self.get_logger().error(f"Failed to retrieve forecast for {latitude},{longitude}")
            return None

        profile = {}
        today_local = datetime.now(uk_tz).date()

        for feature in data["features"]:
            for entry in feature["properties"]["timeSeries"]:
                dt_utc = datetime.strptime(entry["time"], "%Y-%m-%dT%H:%MZ").replace(tzinfo=timezone.utc)
                dt_local = dt_utc.astimezone(uk_tz)
                hour = dt_local.hour

                if dt_local.date() == today_local and 8 <= hour <= 20:
                    feels_like = round(entry["feelsLikeTemperature"])
                    precip = entry.get("totalPrecipAmount", 0.0)
                    profile[hour] = (feels_like, precip)

        return dict(sorted(profile.items()))


def main(args=None):
    rclpy.init(args=args)
    node = WeatherServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()