class DataObject:
    def __init__(self, json_data: list[dict[str, float]]):
        """Create data object from serialised data
        ------
        Parameters
        json_data : list[dict[str, float]]
            List of timestamped dict telemetry
        """
        self.data = json_data

    def __getitem__(self, item: str) -> list[float]:
        """Allows dict-like referenceing
        ------
        Return list of data : list[float]"""
        arr = []
        for pt in self.data:
            arr.append(pt[item])
        return arr

    def at(self, idx: int) -> dict[str, float]:
        """Get all data as dictionary at index
        -------
        Parameters
        idx : int
            Index to view raw telemetry
        """
        return self.data[idx]


    def __len__(self):
        """Allows for length checking len(DataObject)
        ------
        Return length of telemetry : int
        """
        return len(self.data)
