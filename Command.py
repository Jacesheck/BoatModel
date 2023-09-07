import sys
import select
if sys.platform == "win32":
    import msvcrt

class Command():
    def __init__(self):
        "Used to get non blocking commands"
        self.command: str | None = None 

    def getNonBlock(self) -> str | None:
        """Allow for non-blocking input
        -------
        Return command : sr
        """

        if sys.platform == "win32":
            if msvcrt.kbhit():
                char = msvcrt.getch()
                if char == b'\r':
                    copy = self.command
                    self.command = None
                    return copy
                else:
                    self.command += char.decode('utf-8')
        else:
            input_ready, _, _ = select.select([sys.stdin], [], [], 0)
            if input_ready:
                return sys.stdin.readline().rstrip()

        return self.command
