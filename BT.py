import asyncio
import struct
from bleak import BleakClient, BleakScanner
import msvcrt

VAL_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
COORDS_UUID = "45c1eda2-4473-42a3-8143-dc79c30a64bf"

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

def print_val(_, val):
    print(len(val))
    print('x', struct.unpack('<f', val[:4]))
    print('y', struct.unpack('<f', val[4:8]))
    print('z', struct.unpack('<f', val[8:]))

class Central:
    def __init__(self, address):
        self.address = address
    async def connect(self):
        self.client = BleakClient(self.address)
        await self.client.connect()
    async def disconnect(self):
        await self.client.disconnect()
    async def start_notify(self):
        await self.client.start_notify(VAL_UUID, print_val)
    async def stop_notify(self):
        await self.client.stop_notify(VAL_UUID);
    async def write(self, data):
        await self.client.write_gatt_char(COORDS_UUID, data, True)

async def runDevice(address):
    coords = [8.8, 7.94, 13.782, 23.1217]
    byteCoords = b''
    for num in coords:
        newByte = struct.pack('<f',num)
        byteCoords += newByte
        print('Size:', len(newByte))
    print('Will send')
    print(byteCoords)
    central = Central(address)
    await central.connect()
    await central.start_notify()
    print('Waiting for enter')
    while not msvcrt.kbhit():
        await asyncio.sleep(0.1)
    msvcrt.getch()
    await central.write(byteCoords)
    while not msvcrt.kbhit():
        await asyncio.sleep(0.1)
    await central.stop_notify()


async def main():
    devices = await BleakScanner.discover(1.)
    for d in devices:
        if d.name == 'Test':
            await runDevice(d.address)

asyncio.run(main())
