import struct

IDLE_POWER: int = 1500

class DataObject:
    def __init__(self, pickled_data):
        # Create empty dict
        self.attributes = [
            'timestamp',
            'gpsX',
            'gpsY',
            'gpsLat',
            'gpsLng',
            'power1',
            'power2',
            'rz',
        ]

        # Create decoded_data dict of lists
        self.decoded_data = {}
        for attr in self.attributes:
            self.decoded_data[attr] = []

        for point in pickled_data:
            self.decoded_data['timestamp'].append(point[0])
            telemetry = point[1]
            i = 0
            self.decoded_data['gpsX'].append(struct.unpack('<d', telemetry[i: i+8])[0])
            i += 8
            self.decoded_data['gpsY'].append(struct.unpack('<d', telemetry[i: i+8])[0])
            i += 8
            self.decoded_data['gpsLat'].append(struct.unpack('<d', telemetry[i: i+8])[0])
            i += 8
            self.decoded_data['gpsLng'].append(struct.unpack('<d', telemetry[i: i+8])[0])
            i += 8
            self.decoded_data['power1'].append(struct.unpack('<l', telemetry[i: i+4])[0])
            i += 4
            self.decoded_data['power2'].append(struct.unpack('<l', telemetry[i: i+4])[0])
            i += 4
            self.decoded_data['rz'].append(struct.unpack('<f', telemetry[i: i+4])[0])
            i += 4

            # Normalise motor powers
            # TODO: Debug here
            self.decoded_data['power1'][-1] = -IDLE_POWER + self.decoded_data['power1'][-1]
            self.decoded_data['power2'][-1] = +IDLE_POWER - self.decoded_data['power2'][-1]

    def __getitem__(self, item: str) -> list[float] | list[int]:
        return self.decoded_data[item]

    def at(self, idx: int) -> dict[str, float | int]:
        return_dict = {}
        for attr in self.attributes:
            return_dict[attr] = self.decoded_data[attr][idx]
        return return_dict


    def __len__(self):
        return len(self.decoded_data['timestamp'])
