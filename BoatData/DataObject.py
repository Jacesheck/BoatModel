import struct

IDLE_POWER: int = 1500

class DataObject:
    def __init__(self, pickled_data: list[tuple[float, bytearray]]):
        """Create data object from serialised data
        ------
        Parameters
        pickled_data : list[tuple[float, bytearray]]
            List of timestamped bytearray telemetry
        """
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
            # TODO: Make this shorter
            self.decoded_data['timestamp'].append(point[0])
            telemetry = point[1]

            decoded_data = struct.unpack('<ddddllf', telemetry[:44])
            self.decoded_data['gpsX'].append(-decoded_data[0])
            self.decoded_data['gpsY'].append(decoded_data[1])
            self.decoded_data['gpsLat'].append(decoded_data[2])
            self.decoded_data['gpsLng'].append(decoded_data[3])
            self.decoded_data['power1'].append(decoded_data[4])
            self.decoded_data['power2'].append(decoded_data[5])
            self.decoded_data['rz'].append(decoded_data[6])

            # Normalise motor powers
            self.decoded_data['power1'][-1] = -IDLE_POWER + self.decoded_data['power1'][-1]
            self.decoded_data['power2'][-1] = +IDLE_POWER - self.decoded_data['power2'][-1]

    def __getitem__(self, item: str) -> list[float | int]:
        """Allows dict-like referenceing
        ------
        Return list of data : list[float | int]"""
        return self.decoded_data[item]

    def at(self, idx: int) -> dict[str, float | int]:
        """Get all data as dictionary at index
        -------
        Parameters
        idx : int
            Index to view raw telemetry
        """
        return_dict = {}
        for attr in self.attributes:
            return_dict[attr] = self.decoded_data[attr][idx]
        return return_dict


    def __len__(self):
        """Allows for length checking len(DataObject)
        ------
        Return length of telemetry : int
        """
        return len(self.decoded_data['timestamp'])
