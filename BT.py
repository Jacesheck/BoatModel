import asyncio
import struct
from bleak import BleakClient, BleakScanner
import msvcrt

VAL_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"

enter = asyncio.Lock()
running = True

def print_val(_, val):
    print(len(val))
    print('x', struct.unpack('<f', val[:4]))
    print('y', struct.unpack('<f', val[4:8]))
    print('z', struct.unpack('<f', val[8:]))

async def connect(address):
    async with BleakClient(address) as client:
        await client.start_notify(VAL_UUID, print_val)
        while True:
            await enter.acquire()
            if running == False:
                break
            enter.release()
            await asyncio.sleep(0.1)
        await client.stop_notify(VAL_UUID);
        # TODO: Write to file

async def press_enter():
    global running
    print('Waiting for enter')
    while True:
        await asyncio.sleep(0.1)
        if msvcrt.kbhit():
            break
    print('Pressed enter')
    await enter.acquire()
    running = False
    enter.release()

async def main():
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == 'Test':
            t1 = asyncio.create_task(connect(d.address))
            t2 = asyncio.create_task(press_enter())
            await t1
            await t2

asyncio.run(main())
