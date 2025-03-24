import math
import time
import csv

import serial
import numpy as np
import pandas as pd
import rerun as rr
import rerun.blueprint as rrb


SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
TIMEOUT = 1


def main():

    my_blueprint = rrb.Blueprint(
    rrb.Horizontal(
        rrb.Vertical(
            rrb.TimeSeriesView(origin="/sensor/acc"),
            rrb.TimeSeriesView(origin="/sensor/gyro"),
            rrb.TimeSeriesView(origin="/gt/position"),
            rrb.TimeSeriesView(origin="/gt/rotation"),
        ),
        rrb.Vertical(
            rrb.Spatial3DView(
                origin="/gt",
            ),
        ),
        column_shares=[4, 6],
    ),
    rrb.BlueprintPanel(state="collapsed"),
    rrb.SelectionPanel(state="collapsed"),
    rrb.TimePanel(state="expanded"),
    )
    rr.init("rerun_example_my_data", spawn=True)
    rr.serve()
    rr.send_blueprint(my_blueprint, make_active=True)




    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
            print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")

            while True:
                data = ser.readline().decode("utf-8", errors="ignore").strip()
                if data.startswith("ts="):
                    data_dict = {key: value for key, value in (item.split("=") for item in data.split(","))}
                    timestamp = int(data_dict["ts"])
                    temperature = float(data_dict["temp"])
                    ax, ay, az = float(data_dict["ax"]), float(data_dict["ay"]), float(data_dict["az"])
                    gx, gy, gz = float(data_dict["gx"]), float(data_dict["gy"]), float(data_dict["gz"])

                    rr.log("/sensor/acc/acc_x", rr.Scalar(ax))
                    rr.log("/sensor/acc/acc_y", rr.Scalar(ay))
                    rr.log("/sensor/acc/acc_z", rr.Scalar(az))
                    rr.log("/sensor/gyro/gyro_x", rr.Scalar(gx))
                    rr.log("/sensor/gyro/gyro_y", rr.Scalar(gy))
                    rr.log("/sensor/gyro/gyro_z", rr.Scalar(gz))
                    # rr.log(
                    #     "/sensor/acc/acc_x",
                    #     times=[rr.TimeSecondsColumn("step", timestamp)],
                    #     components=[rr.components.Scalar(x)],
                    # )

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user.")


if __name__ == "__main__":

    main()
