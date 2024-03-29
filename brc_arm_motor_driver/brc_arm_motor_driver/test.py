import serial.tools.list_ports
from roboclaw_3 import Roboclaw

ports = list(serial.tools.list_ports.comports())
print(f"{ports}\n")
if len(ports) < 3:  # UPDATE!!!!
    print("Not enough COM devices detected")
    raise SystemExit
for p in ports:
    print(p)
    if "Roboclaw" in p.description:
        print(f"Roboclaw detected at port {p.device}")
        roboclaw = Roboclaw(p.device, 115200)
        try:
            print(f"Connecting to Roboclaw at {p.device}")
            roboclaw.Open()
        except Exception as e:
            print(f"Could not connect to Roboclaw at {p.device}")
            print(e)
            raise SystemExit

        # Get roboclaw address
        # Should have more robust duplicate checking and stuff but works good enough
        address = 0
        for adr in [128, 129, 130]:
            try:
                version = roboclaw.ReadVersion(adr)
            except AttributeError as e:
                print(f"Could not connect to Roboclaw at {adr}")
                print(e)
                raise SystemExit
            except Exception as e:
                print(type(e).__name__)
                print(f"Problem getting roboclaw version at {adr}")
                print(e)
                pass
            if not version[0]:
                address = -1
            else:
                address = adr
                print(version[1])
                roboclaw.SetEncM1(adr, 0)
                break

        if address == -1:
            print(f"No valid roboclaw found")
            raise SystemExit
        else:
            print(f"Connected to Roboclaw {address}")
            roboclaw.SetPinFunctions(address, 0x00, 0x62, 0x62)
