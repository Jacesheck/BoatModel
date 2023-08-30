import asyncio
import re
import struct
from bleak import BleakClient, BleakScanner
import msvcrt
import time
import pickle
import sys
import select

DEBUG_UUID     = "45c1eda2-4473-42a3-8143-dc79c30a64bf"
CMD_UUID       = "05c6cc87-7888-4588-b794-92bdf9a29330"
COORDS_UUID    = "3794c841-1b53-4029-aebb-12319386fd28"
TELEMETRY_UUID = "ccc03716-4f66-4cb8-b6fd-9b2278587add"

TELEMETRY_FILE = "telemetry/" + time.asctime().replace(':','.').replace(' ','')

command = '' 

class TelemetryRecorder:
    def __init__(self):
        """Records telemetry using callback"""
        self.data = []

    def append(self, _, data: bytes):
        """Append data with timestamp
        -------
        Parameters
        data : bytes
            Raw telemetry data
        """
        self.data.append([time.time(), data])

    def save(self):
        "Save received telemetry to file"
        print('Saving file of length', len(self.data))
        with open(TELEMETRY_FILE,'wb') as f:
            pickle.dump(self.data, f)
            # TODO: Test this
        self.data = []

class Peripheral:
    def __init__(self, address: str):
        """Used as connection object with arduino
        Parameters
        ----------
        address : str
            MAC address of arduino
        """
        self.address = address
        self.telemetry = TelemetryRecorder()

    def debug_callback(self, _, val):
        "Callback used for debug messages"
        print(' '*len(command), end='\r')
        print(val.decode('utf-8'))
        print(command, end='\r')

    async def connect(self):
        "Connect to bluetooth client and setup notify characteristics"
        self.client = BleakClient(self.address)
        await self.client.connect()
        await self.client.start_notify(DEBUG_UUID, self.debug_callback)
        await self.client.start_notify(TELEMETRY_UUID, self.telemetry.append)

        print('Device ready')

    def isConnected(self) -> bool:
        "Return whether it is still connected to the arduino"
        return self.client.is_connected

    async def disconnect(self):
        "Safely disconnect and stop all notify characteristics"
        await self.client.stop_notify(DEBUG_UUID);
        await self.client.stop_notify(TELEMETRY_UUID);
        await self.client.disconnect()

    async def writeCommand(self, data: str):
        """Write a command to the client
        -------
        Parameters
        data : str
            Command to write to arduino
        """
        await self.client.write_gatt_char(CMD_UUID, data.encode('utf-8'), True)

    async def writeCoords(self, data: bytes):
        """Write coordinates to the client. Data must be 0 terminated
        -------
        Parameters
        data : bytes
            Write data as bytes of doubles
        """
        await self.client.write_gatt_char(COORDS_UUID, data, True)

def nonBlockingInput() -> str:
    """Allow for non-blocking input
    -------
    Return command : sr
    """
    global command
    if sys.platform == "win32":
        if msvcrt.kbhit():
            char = msvcrt.getch()
            if char == b'\r':
                copy = command
                command = ''
                return copy
            else:
                command += char.decode('utf-8')
                print(command, end='\r')
    else:
        input_ready, _, _ = select.select([sys.stdin], [], [], 0)
        if input_ready:
            return sys.stdin.readline().rstrip()

    return ""

def getCoordsFromFile() -> bytes:
    """Retrieve coordinates from file and convert to doubles
    ------
    Return bytes of doubles : bytes"""
    with open('route.gpx', 'r') as f:
        text = f.read()
    found = re.findall(r'<rtept lat="(.+?)" lon="(.+?)"', text)
    ret = b''
    for coord in found:
        lat = float(coord[0])
        lng = float(coord[1])
        ret += struct.pack('<d', lat)
        ret += struct.pack('<d', lng)
    # 0 terminated
    return ret + struct.pack('<d', 0)

async def runDeviceLoop(address: str):
    """Run arduino with commands
    ------
    Parameters
    address : str
        MAC address of arduino
    """
    running = True
    while running:
        peripheral = Peripheral(address)
        await peripheral.connect()

        while peripheral.isConnected():
            command = nonBlockingInput()
            if command == None:
                await asyncio.sleep(0.01)
            elif command == 's' or command == 'send':
                coords = getCoordsFromFile()
                await peripheral.writeCoords(coords)
            elif command == 'x' or command == 'exit':
                print('Disconnecting...')
                await peripheral.disconnect()
                running = False
                break
            elif command == 't' or command == 'telemetry':
                peripheral.telemetry.save()
                print('Telemetry saved')
            else:
                await peripheral.writeCommand(command)
            # TODO: finish runDevice
        print('Device disconnected')

async def main():
    "Find device with name boat and run device"
    devices = await BleakScanner.discover(1.)
    for d in devices:
        if d.name == 'Boat':
            await runDeviceLoop(d.address)

asyncio.run(main())
