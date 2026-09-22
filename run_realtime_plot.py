
from src.unmanned_systems_basics.real_time_plot import MAVLINK_CONNECTION, TelemetryReceiver, TelemetryDashboard
from PyQt5 import QtWidgets
import sys


def run_realtime_plot():
    receiver = TelemetryReceiver(MAVLINK_CONNECTION)
    app = QtWidgets.QApplication(sys.argv)

    dashboard = TelemetryDashboard(receiver)

    dashboard.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    run_realtime_plot()
    