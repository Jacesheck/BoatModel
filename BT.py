import asyncio
import re
import struct
from bleak import BleakClient, BleakScanner
import msvcrt

DEBUG_UUID  = "45c1eda2-4473-42a3-8143-dc79c30a64bf"
CMD_UUID    = "05c6cc87-7888-4588-b794-92bdf9a29330"
COORDS_UUID = "3794c841-1b53-4029-aebb-12319386fd28"

command = '' 

async def print_details(address):
    async with BleakClient(address) as client:
        services = client.services
        for s in services:
            print("Service:", s)
            print("Type:", type(s))
            print("Characteristics", s.characteristics)
            for c in s.characteristics:
                print('UUID', c.uuid)
                try:
                    val = await client.read_gatt_char(c.uuid)
                    print('len', len(val))
                    print(struct.unpack('<f', val)) # Little endian
                except Exception as e:
                    print(e)
            print()

def debug_callback(_, val):
    print(' '*len(command), end='\r')
    print(val.decode('utf-8'))
    print(command, end='\r')
    #print('x', struct.unpack('<f', val[:4]))
    #print('y', struct.unpack('<f', val[4:8]))
    #print('z', struct.unpack('<f', val[8:]))

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

#async def runDevice(address):
#    coords = [8.8, 7.94, 13.782, 23.1217]
#    byteCoords = b''
#    for num in coords:
#        newByte = struct.pack('<f',num)
#        byteCoords += newByte
#        print('Size:', len(newByte))
#    print('Will send')
#    print(byteCoords)
#    central = Central(address)
#    await central.connect()
#    print('Waiting for enter')
#    while not msvcrt.kbhit():
#        await asyncio.sleep(0.1)
#    msvcrt.getch()
#    await central.write(byteCoords)
#    while not msvcrt.kbhit():
#        await asyncio.sleep(0.1)
#    await central.stop_notify()


async def main():
    devices = await BleakScanner.discover(1.)
    for d in devices:
        if d.name == 'Boat':
            await runDevice(d.address)

asyncio.run(main())
