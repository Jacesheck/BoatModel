import asyncio
import re
import struct
from bleak import BleakClient, BleakScanner
import msvcrt

DEBUG_UUID  = "45c1eda2-4473-42a3-8143-dc79c30a64bf"
CMD_UUID    = "05c6cc87-7888-4588-b794-92bdf9a29330"
COORDS_UUID = "3794c841-1b53-4029-aebb-12319386fd28"

command = '' 

def debug_callback(_, val):
    print(' '*len(command), end='\r')
    print(val.decode('utf-8'))
    print(command, end='\r')

class Peripheral:
    def __init__(self, address):
        self.address = address
    async def connect(self):
        self.client = BleakClient(self.address)
        await self.client.connect()
        await self.client.start_notify(DEBUG_UUID, debug_callback)
        print('Device ready')
    async def disconnect(self):
        await self.client.disconnect()
        await self.client.stop_notify(DEBUG_UUID);
    async def writeCommand(self, data: str):
        await self.client.write_gatt_char(CMD_UUID, data.encode('utf-8'), True)
    async def writeCoords(self, data: bytes):
        await self.client.write_gatt_char(COORDS_UUID, data, True)

def nonBlockingInput() -> str:
    global command
    if msvcrt.kbhit():
        char = msvcrt.getch()
        if char == b'\r':
            copy = command
            command = ''
            return copy
        else:
            command += char.decode('utf-8')
            print(command, end='\r')
    return ""

def getCoordsFromFile() -> bytes:
    with open('route.gpx', 'r') as f:
        text = f.read()
    found = re.findall(r'<rtept lat="(.+?)" lon="(.+?)"', text)
    ret = b''
    for coord in found:
        lat = float(coord[0])
        lng = float(coord[1])
        ret += struct.pack('<d', lat)
        ret += struct.pack('<d', lng)
    # Null terminated
    return ret + struct.pack('<d', 0)

async def runDevice(address):
    peripheral = Peripheral(address)
    await peripheral.connect()
    while True:
        command = nonBlockingInput()
        if command == '':
            await asyncio.sleep(0.01)
        elif command == 's' or command == 'send':
            coords = getCoordsFromFile()
            await peripheral.writeCoords(coords)
        elif command == 'x' or command == 'exit':
            await peripheral.disconnect()
            break
        else:
            await peripheral.writeCommand(command)
    # TODO: finish runDevice

async def main():
    devices = await BleakScanner.discover(1.)
    for d in devices:
        if d.name == 'Boat':
            await runDevice(d.address)

asyncio.run(main())
