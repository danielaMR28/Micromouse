"""

Monitor Bluetooth Micromouse con Gyro

"""
 
import serial

from datetime import datetime
 
PUERTO = "/dev/cu.HC-05"
 
def main():

    print("=" * 65)

    print("   MICROMOUSE MONITOR")

    print("=" * 65)

    try:

        ser = serial.Serial(PUERTO, 9600, timeout=1)

        print(f"✓ Conectado a {PUERTO}\n")

    except Exception as e:

        print(f"✗ Error: {e}")

        return

    print(f"{'Tiempo':<10} {'Front':>5} {'Left':>5} {'Right':>5} {'Yaw':>7}  Acción")

    print("-" * 65)

    try:

        while True:

            if ser.in_waiting:

                linea = ser.readline().decode('utf-8', errors='ignore').strip()

                if linea == "START":

                    print("\n>>> MICROMOUSE INICIADO <<<\n")

                    continue

                partes = linea.split(",")

                if len(partes) == 5:

                    tiempo = datetime.now().strftime("%H:%M:%S")

                    front, left, right, yaw, accion = partes

                    warn = "!" if int(front) < 10 else " "

                    print(f"{tiempo:<10} {front:>5}{warn} {left:>5} {right:>5} {yaw:>7}  {accion}")

    except KeyboardInterrupt:

        print("\n\nCerrando...")

    finally:

        ser.close()
 
if __name__ == "__main__":

    main()
 
