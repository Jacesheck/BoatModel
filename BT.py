import asyncio
import re
import struct
from bleak import BleakClient, BleakScanner
#from Command import Command
import time
import pickle
import pygame

DEBUG_UUID     = "45c1eda2-4473-42a3-8143-dc79c30a64bf"
STATUS_UUID    = "6f04c0a3-f201-4091-a13d-5ecafc3dc54b"
CMD_UUID       = "05c6cc87-7888-4588-b794-92bdf9a29330"
COORDS_UUID    = "3794c841-1b53-4029-aebb-12319386fd28"
TELEMETRY_UUID = "ccc03716-4f66-4cb8-b6fd-9b2278587add"

TELEMETRY_FILE = "telemetry/" + time.asctime().replace(':','.').replace(' ','')

LOOP_PERIOD = 0.01 # s

pygame.init()
FONT  = pygame.font.Font('freesansbold.ttf', 24)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED   = (255, 0, 0)

g_debug = ""
g_peripheral_connected = False
g_GPS_AVAIL  : int = 1 << 0
g_RC_AVAIL   : int = 1 << 1
g_INIT       : int = 1 << 2
g_gps_avail  : bool = False
g_rc_avail   : bool = False
g_initialised: bool = False

g_command : str | None = None

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
            # TODO: Test this?
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
        "Callback used for debug messages from arduino"
        global g_debug
        g_debug += "\n" + val.decode('utf-8')

    def status_callback(self, _, val):
        "Callback used when status is changed"
        status: int = struct.unpack('<I', val)[0]
        print(f"Status received {status}")
        global g_gps_avail
        global g_rc_avail
        global g_initialised
        g_gps_avail = bool(status & g_GPS_AVAIL)
        g_rc_avail = bool(status & g_RC_AVAIL)
        g_initialised = bool(status & g_INIT)

    async def connect(self):
        "Connect to bluetooth client and setup notify characteristics"
        self.client = BleakClient(self.address)
        await self.client.connect()
        await self.client.start_notify(DEBUG_UUID, self.debug_callback)
        await self.client.start_notify(STATUS_UUID, self.status_callback)
        await self.client.start_notify(TELEMETRY_UUID, self.telemetry.append)

        print('Notify is ready')

    def isConnected(self) -> bool:
        "Return whether it is still connected to the arduino"
        return self.client.is_connected

    async def disconnect(self):
        "Safely disconnect and stop all notify characteristics"
        await self.client.stop_notify(DEBUG_UUID);
        await self.client.stop_notify(STATUS_UUID);
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
    global g_peripheral_connected
    global g_command

    running = True
    while running:
        g_peripheral_connected = False
        peripheral = Peripheral(address)
        await peripheral.connect()
        g_peripheral_connected = True

        while peripheral.isConnected():
            c: str | None = g_command
            if c == None:
                await asyncio.sleep(LOOP_PERIOD)
            else:
                g_command = None
                if c == 's' or c == 'send':
                    coords = getCoordsFromFile()
                    await peripheral.writeCoords(coords)
                elif c == 'x' or c == 'exit':
                    print('Disconnecting...')
                    await peripheral.disconnect()
                    running = False
                    g_peripheral_connected = False
                    break
                elif c == 't' or c == 'telemetry':
                    peripheral.telemetry.save()
                    print('Telemetry saved')
                else:
                    await peripheral.writeCommand(c)
            # TODO: finish runDevice

class TextField:
    def __init__(self, x, y):
        self.black = (0, 0, 0)
        self.gray = (180, 180, 180)
        self.white = (255, 255, 255)
        self.text_field_rect = pygame.Rect(x, y, 400, 3*26)
        self.x = x
        self.y = y

    def blit(self, multi_line_text, screen):
        split: list[str] = multi_line_text.split("\n")
        y = self.y 
        pygame.draw.rect(screen, self.gray, self.text_field_rect)
        for i in range(max(-len(split), -3), 0):
            debug_text = FONT.render(split[i], True, self.black, self.gray)
            debug_rect = debug_text.get_rect()
            debug_rect.topleft = (self.x, y)
            y+= 26
            screen.blit(debug_text, debug_rect)

class Indicator:
    def __init__(self, x: int, y: int, text: str):
        self.indicator = pygame.Rect(x, y, 20, 20)
        self.text = FONT.render(text, True, BLACK, WHITE)
        self.text_rect = self.text.get_rect()
        self.text_rect.topleft = (x + 30, y)

    def blit(self, screen, on: bool):
        color = GREEN if on else RED
        pygame.draw.rect(screen, color, self.indicator)
        screen.blit(self.text, self.text_rect)

async def runDisplay():
    global g_command
    global g_debug

    screen = pygame.display.set_mode((1280, 720))
    #clock = pygame.time.Clock()
    running = True
     
    # Debug text
    debug_text = TextField(40, 10)

    # Indicators
    peripheral_indicator = Indicator(40, 100, "Arduino connected")
    gps_indicator = Indicator(40, 130, "GPS available")
    rc_indicator = Indicator(40, 160, "RC available")
    init_indicator = Indicator(40, 190, "Initialised")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    g_command = 'f'
                elif event.key == pygame.K_DOWN:
                    g_command = 'b'
                elif event.key == pygame.K_LEFT:
                    g_command = 'l'
                elif event.key == pygame.K_RIGHT:
                    g_command = 'r'

            if event.type == pygame.MOUSEBUTTONUP:
                pass
        screen.fill("white")

        # Draw indicators
        peripheral_indicator.blit(screen, g_peripheral_connected)
        gps_indicator.blit(screen, g_gps_avail)
        rc_indicator.blit(screen, g_rc_avail)
        init_indicator.blit(screen, g_initialised)

        # Debug stream
        debug_text.blit(g_debug, screen)

        pygame.display.flip()
        await asyncio.sleep(LOOP_PERIOD)
    pygame.quit()

async def main():
    "Find device with name boat and run device"
    devices = await BleakScanner.discover(1.)
    for d in devices:
        if d.name == 'Boat':
            await asyncio.gather(runDisplay(), runDeviceLoop(d.address))

asyncio.run(main())
