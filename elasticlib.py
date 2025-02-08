import http.server
import json
import os
import socketserver
import threading
from enum import Enum

from ntcore import NetworkTableInstance, PubSubOptions
from wpilib import DataLogManager


def start_elastic_server(address: str) -> None:
    """Starts TCP server on port 5800 on a separate thread to allow Elastic to download layouts remotely."""
    from robot import OilSpill

    def start_request_handler(target_address: str) -> None:
        # noinspection PyTypeChecker
        class DeployHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):

            def __init__(self, *args, **kwargs) -> None:
                super().__init__(*args, directory=OilSpill.get_deploy_directory(), **kwargs)

            # noinspection PyNoneFunctionAssignment
            def do_GET(self):
                if self.path == "/?format=json":
                    try:

                        layouts_directory = os.path.join(OilSpill.get_deploy_directory())
                        response_data = {
                            "files": []
                        }

                        for layout_file in os.listdir(layouts_directory):
                            if layout_file.endswith('.json'):
                                response_data["files"].append({"name": layout_file})

                        response_json = json.dumps(response_data)

                        _ = self.send_response_only(200)
                        self.send_header("Content-Type", "application/json")
                        self.end_headers()
                        self.wfile.write(response_json.encode('utf-8'))
                        DataLogManager.log(f"{target_address}: 200 OK, application/json")
                    except Exception as exc:
                        _ = self.send_response_only(500)
                        self.send_header("Content-Type", "text/plain")
                        self.end_headers()
                        self.wfile.write(f"Error: {str(exc)}".encode('utf-8'))
                        DataLogManager.log(f"{target_address}: 500, Error serving layout: {exc}")
                elif self.path.endswith("layout.json"):
                    layout_name = self.path.lstrip("/")
                    layout_path = os.path.join(OilSpill.get_deploy_directory(), layout_name)

                    if os.path.exists(layout_path):
                        try:
                            with open(layout_path, "rb") as layout_file:
                                file_data = layout_file.read()

                            _ = self.send_response_only(200)
                            self.send_header("Content-Type", "application/json")
                            self.send_header("Content-Length", str(len(file_data)))
                            self.end_headers()
                            self.wfile.write(file_data)
                            DataLogManager.log(f"{target_address}: 200 OK, {layout_name}")
                        except Exception as exc:
                            _ = self.send_response_only(500)
                            self.send_header("Content-Type", "text/plain")
                            self.end_headers()
                            self.wfile.write(f"Error serving layout: {str(exc)}".encode('utf-8'))
                            DataLogManager.log(f"{target_address}: 500, Error serving layout: {exc}")
                    else:
                        _ = self.send_response_only(404)
                        self.send_header("Content-Type", "text/plain")
                        self.end_headers()
                        self.wfile.write(b"Layout file not found.")
                        DataLogManager.log(f"{target_address}: 404, Layout file '{layout_name}' not found.")

        try:
            with socketserver.TCPServer((target_address, 5800), DeployHTTPRequestHandler) as httpd:
                try:
                    httpd.serve_forever()
                except Exception as e:
                    DataLogManager.log(f"{target_address}: Web server encountered an error: {e}")
                finally:
                    httpd.server_close()
        except OSError:
            DataLogManager.log(f"OSError for Elastic server {target_address}!")

    server_thread = threading.Thread(target=lambda: start_request_handler(address), daemon=True)
    server_thread.start()

    DataLogManager.log(f"Opened Elastic TCP at {address}")


class NotificationLevel(Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


class Notification:
    """Represents a notification with various display properties."""

    def __init__(
            self,
            level=NotificationLevel.INFO,
            title: str = "",
            description: str = "",
            display_time: int = 3000,
            width: float = 350,
            height: float = -1,
    ):
        """
        Initializes an ElasticNotification object.

        Args:
            level (str): The severity level of the notification. Default is 'INFO'.
            title (str): The title of the notification. Default is an empty string.
            description (str): The description of the notification. Default is an empty string.
            display_time (int): Time in milliseconds for which the notification should be displayed. Default is 3000 ms.
            width (float): Width of the notification display area. Default is 350.
            height (float): Height of the notification display area. Default is -1 (automatic height).
        """
        self.level = level
        self.title = title
        self.description = description
        self.display_time = display_time
        self.width = width
        self.height = height


__selected_tab_topic = None
__selected_tab_publisher = None

__notification_topic = None
__notification_publisher = None


def send_notification(notification: Notification):
    """
    Sends a notification to the Elastic dashboard.
    The notification is serialized as a JSON string before being published.

    Args:
        notification (ElasticNotification): The notification object containing the notification details.

    Raises:
        Exception: If there is an error during serialization or publishing the notification.
    """
    global __notification_topic
    global __notification_publisher

    if not __notification_topic:
        __notification_topic = NetworkTableInstance.getDefault().getStringTopic(
            "/Elastic/RobotNotifications"
        )
    if not __notification_publisher:
        __notification_publisher = __notification_topic.publish(
            PubSubOptions(sendAll=True, keepDuplicates=True)
        )

    try:
        __notification_publisher.set(
            json.dumps(
                {
                    "level": notification.level,
                    "title": notification.title,
                    "description": notification.description,
                    "displayTime": notification.display_time,
                    "width": notification.width,
                    "height": notification.height,
                }
            )
        )
    except Exception as e:
        print(f"Error serializing notification: {e}")


def select_tab(tab_name: str):
    """
    Selects the tab of the dashboard with the given name.
    If no tab matches the name, this will have no effect on the widgets or tabs in view.
    If the given name is a number, Elastic will select the tab whose index equals the number provided.

    Args:
        tab_name (str) the name of the tab to select
    """
    global __selected_tab_topic
    global __selected_tab_publisher

    if not __selected_tab_topic:
        __selected_tab_topic = NetworkTableInstance.getDefault().getStringTopic(
            "/Elastic/SelectedTab"
        )
    if not __selected_tab_publisher:
        __selected_tab_publisher = __selected_tab_topic.publish(
            PubSubOptions(keepDuplicates=True)
        )

    __selected_tab_publisher.set(tab_name)


def select_tab_index(tab_index: int):
    """
    Selects the tab of the dashboard at the given index.
    If this index is greater than or equal to the number of tabs, this will have no effect.

    Args:
        tab_index (int) the index of the tab to select
    """
    select_tab(str(tab_index))
