import numpy as np

class BinaryRoutines():
    """
    Class handler for `setb` command to send binary tables to the dds-sweeper
    """

    def __init__(self, conn):
        self.conn = conn
        self.mirror_channels = False
        self.debug_run = True
        self.MAX_SIZE = 256
        self.instructions = None
        self.timing_mode = None

    def send(self, command: str, echo = True):
        if command[-1] != '\n':
            command += '\n'

        self.conn.write(command.encode())
        if echo:
            print(self.catch_response())

    def catch_response(self):
        resp = self.conn.readline().decode().strip()
        return resp

    def assert_OK(self):
        resp = self.conn.readline().decode().strip()
        assert resp == 'ok', 'Expected "ok", received "%s"' % resp

    def debug_on(self):
        self.conn.write(b'debug on\n')
        self.assert_OK()

    def debug_off(self):
        self.conn.write(b'debug off\n')
        self.assert_OK()

    def set_mode(self, sweep_mode: int, timing_mode: int, num_channels: int):
        self.conn.write(b'reset\n')
        self.assert_OK()

        self.timing_mode = timing_mode
        self.conn.write(b'mode %d %d \n' % (sweep_mode, timing_mode))
        self.assert_OK()
        self.conn.write(b'setchannels %d\n' % num_channels)
        self.assert_OK()
        if self.debug_run == True:
            print('sending commands to %d channels' % num_channels)

    def allocate(self, start_address: int, instructions: np.ndarray):
        self.conn.write(b'setb %d %d\n' % (start_address, len(instructions)))
        response = self.conn.readline().decode()
        if not response.startswith('ready'):
            response += ''.join([r.decode() for r in self.conn.readlines()])
            raise Exception(f'setb command failed, response: {repr(response)}')
        if self.debug_run == True:
            print(f'Got response: {response}')

    def table_to_memory(self, instructions: np.ndarray):
        self.conn.write(instructions.tobytes())
        self.assert_OK(), 'table not written correctly'
        if self.debug_run == True:
            print('table written to memory')

    def end_table(self, instructions: np.ndarray):
        self.conn.write(b'set 4 %d\n' % len(instructions))
        self.assert_OK(), 'table not stopped correctly'
        self.instructions = instructions
        if self.debug_run == True:
            print('table end command sent')

    def start_routine(self, hwstart = False):
        if hwstart:
            self.conn.write(b'hwstart\n')
        else:
            self.conn.write(b'start\n')
        self.assert_OK(), 'table did not start'
        if self.debug_run == True:
            print('table executed')

    def read_table(self, indices = None):
        self.conn.write(b'readtable\n')
        for i in range(self.instructions.size + 2): # plus two to catch header and footer
            resp = self.catch_response()
            print(resp)
            if "End of" in resp:
                break
            elif "Cannot" in resp:
                break
            elif len(resp) == 0:
                break
            
    def get_memory_layout(self):
        self.conn.write(b'getmode\n')
        mode = self.catch_response()
        channels = self.catch_response()
        memory_layout = self.catch_response()
        print(f"mode {mode}")
        print(channels)
        print(memory_layout)