import asyncio
import os
import re
import struct
from bleak import BleakClient, BleakScanner
#from Command import Command
import time
import json
import pygame
import numpy as np

DEBUG_UUID     = "45c1eda2-4473-42a3-8143-dc79c30a64bf"
STATUS_UUID    = "6f04c0a3-f201-4091-a13d-5ecafc3dc54b"
CMD_UUID       = "05c6cc87-7888-4588-b794-92bdf9a29330"
COORDS_UUID    = "3794c841-1b53-4029-aebb-12319386fd28"
TELEMETRY_UUID = "ccc03716-4f66-4cb8-b6fd-9b2278587add"
KALMAN_UUID    = "933963ae-cc8e-4704-bd3c-dc53721ba956"

LOOP_PERIOD = 0.01 # s

pygame.init()
FONT  = pygame.font.Font('freesansbold.ttf', 24)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED   = (255, 0, 0)
GRAY  = (180, 180, 180)

g_debug = ""
g_peripheral_connected = False
g_GPS_AVAIL  : int = 1 << 0
g_RC_AVAIL   : int = 1 << 1
g_INIT       : int = 1 << 2
g_RC_MODE    : int = 1 << 3

g_gps_avail  : bool = False
g_rc_avail   : bool = False
g_initialised: bool = False
g_rc_mode    : bool = True

class KalmanState:
    def update(self, vals):
        self.x = vals[0]
        self.y = vals[1]
        self.vx = vals[2]
        self.vy = vals[3]
        self.heading = vals[4]
        self.dheading = vals[5]

    def isReady(self) -> bool:
        return hasattr(self, 'x')

class MotorState:
    def update(self, vals):
        self.motorLeft = vals[4]
        self.motorRight = vals[5]

    def isReady(self) -> bool:
        return hasattr(self, 'motorLeft')

g_kalmanState = KalmanState()
g_motorState = MotorState()

g_command : str | None = None

class DataRecorder:
    def __init__(self):
        """Records telemetry using callback"""
        self.telemetry: list[dict[str, float]] = []
        self.kalman: list[dict[str, float]] = []
    def appendTelem(self, named_data: dict[str, float]):
        """Append telemetry with timestamp
        -------
        Parameters
        named_data : dist[str, float]
            Telemetry data as dict
        """
        self.telemetry.append(named_data)

    def appendKalman(self, named_state: dict[str, float]):
        """Append kalman filter data with timestamp
        -------
        Parameters
        named_state : dict[str, float]
            Kalman values in dict form
        """
        self.kalman.append(named_state)

    def save(self):
        "Save received telemetry to file"
        print('Saving file of length', len(self.telemetry))
        if not os.path.exists('BoatData/telemetry'):
            os.makedirs("BoatData/telemetry")
        telemetry_file = "BoatData/telemetry/telem" + time.asctime().replace(':','.').replace(' ','_') + ".json"
        kalman_file    = "BoatData/telemetry/kalman" + time.asctime().replace(':','.').replace(' ','_') + ".json"
        with open(telemetry_file,'w') as f:
            json.dump(self.telemetry, f)
        with open(kalman_file,'w') as f:
            json.dump(self.kalman, f)
        self.telemetry = []
        self.kalman = []

class Peripheral:
    def __init__(self, address: str):
        """Used as connection object with arduino
        Parameters
        ----------
        address : str
            MAC address of arduino
        """
        self.address = address
        self.dataRecorder = DataRecorder()

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
        global g_rc_mode
        global g_initialised
        g_gps_avail = bool(status & g_GPS_AVAIL)
        g_rc_avail = bool(status & g_RC_AVAIL)
        g_rc_mode  = bool(status & g_RC_MODE)
        g_initialised = bool(status & g_INIT)

    def kalman_callback(self, _, val):
        global g_kalmanState
        state: list[float] = list(struct.unpack('<ffffff', val))
        named_state = {
            "timestamp": time.time(),
            "x": state[0],
            "y": state[1],
            "dx" : state[2],
            "dy" : state[3],
            "heading" : state[4],
            "d_heading" : state[5]
        }
        self.dataRecorder.appendKalman(named_state)
        g_kalmanState.update(state)

    def telem_callback(self, _, val):
        global g_motorState
        telem: list[float] = list(struct.unpack('<ddddfff', val))
        named_telem = {
            "timestamp": time.time(),
            "gpsX" : telem[0],
            "gpsY" : telem[1],
            "lat" : telem[2],
            "lng" : telem[3],
            "powerLeft" : telem[4],
            "powerRight" : telem[5],
            "rz" : telem[6]
        }
        self.dataRecorder.appendTelem(named_telem)
        g_motorState.update(telem)

    async def connect(self):
        global g_status

        "Connect to bluetooth client and setup notify characteristics"
        self.client = BleakClient(self.address)
        await self.client.connect()
        await self.client.start_notify(DEBUG_UUID, self.debug_callback)
        await self.client.start_notify(STATUS_UUID, self.status_callback)
        await self.client.start_notify(TELEMETRY_UUID, self.telem_callback)
        await self.client.start_notify(KALMAN_UUID, self.kalman_callback)
        g_status = await self.client.read_gatt_char(STATUS_UUID)

        print('Notify is ready')

    def isConnected(self) -> bool:
        "Return whether it is still connected to the arduino"
        return self.client.is_connected

    async def disconnect(self):
        "Safely disconnect and stop all notify characteristics"
        await self.client.stop_notify(DEBUG_UUID);
        await self.client.stop_notify(STATUS_UUID);
        await self.client.stop_notify(TELEMETRY_UUID);
        await self.client.stop_notify(KALMAN_UUID);
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
                    peripheral.dataRecorder.save()
                    print('Telemetry saved')
                else:
                    await peripheral.writeCommand(c)
            # TODO: finish runDevice

class Radial:
    def __init__(self, x, y, width, height, title):
        self.width = width
        self.height = height
        self.title = FONT.render(title, True, BLACK, WHITE)
        self.title_rect = self.title.get_rect()
        self.title_rect.topleft = (x, y)
        self.x = x
        self.y = y + 25 # For text
        self.back_rect = pygame.Rect(self.x, self.y, self.width, self.height)
        self.center_x = self.x + (self.width / 2)
        self.center_y = self.y + (self.height / 2)

    def blit(self, screen, x: float, y: float):
        """Draw Radial to screen
        --------
        Parameters
        x : float
            x component of radial display [-1, 1]
        y : float
            y component of radial display [-1, 1]
        """
        screen.blit(self.title, self.title_rect)
        pygame.draw.rect(screen, GRAY, self.back_rect)
        mover_center = (self.center_x + (x * self.width/2),
                        self.center_y - (y * self.height/2))
        pygame.draw.circle(screen, BLACK, mover_center, 5)
        pygame.draw.line(screen, BLACK,
                (self.center_x,  self.center_y), mover_center, 2)

class TextField:
    def __init__(self, x, y):
        self.text_field_rect = pygame.Rect(x, y, 400, 3*26)
        self.x = x
        self.y = y

    def blit(self, multi_line_text, screen):
        split: list[str] = multi_line_text.split("\n")
        y = self.y 
        pygame.draw.rect(screen, GRAY, self.text_field_rect)
        for i in range(max(-len(split), -3), 0):
            debug_text = FONT.render(split[i], True, BLACK, GRAY)
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
    rc_mode = Indicator(40, 190, "RC mode")
    init_indicator = Indicator(40, 220, "Initialised")

    velocity = Radial(40, 250, 200, 200, "Velocity")
    heading = Radial(250, 250, 200, 200, "Heading")
    motor  = Radial(40, 490, 200, 200, "Motors")

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
                elif event.key == pygame.K_h:
                    g_command = 'h'
                elif event.key == pygame.K_t:
                    g_command = 't'
                elif event.key == pygame.K_s:
                    g_command = 's'
                elif event.key == pygame.K_k:
                    g_command = 'k'
                elif event.key == pygame.K_m:
                    g_command = 'm'

            if event.type == pygame.MOUSEBUTTONUP:
                pass
        screen.fill("white")

        # Draw indicators
        peripheral_indicator.blit(screen, g_peripheral_connected)
        gps_indicator.blit(screen, g_gps_avail)
        rc_indicator.blit(screen, g_rc_avail)
        rc_mode.blit(screen, g_rc_mode)
        init_indicator.blit(screen, g_initialised)

        # Radials
        if g_kalmanState.isReady():
            velocity.blit(screen, g_kalmanState.vx, g_kalmanState.vy)
            heading.blit(screen, 0.5*np.sin(np.deg2rad(g_kalmanState.heading)),
                         0.5*np.cos(np.deg2rad(g_kalmanState.heading)))
        if g_motorState.isReady():
            l = g_motorState.motorLeft
            r = g_motorState.motorRight
            motor.blit(screen, l - r, l + r)

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
