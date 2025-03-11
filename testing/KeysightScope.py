import pyvisa

class KeysightScope:
    """
    Helper class using pyvisa for a USB connected Keysight Oscilloscope 
    """
    def __init__(self, 
                 addr='USB?*::INSTR',
                 timeout=1, 
                 termination='\n'
                 ):
        rm = pyvisa.ResourceManager()
        devs = rm.list_resources(addr)
        assert len(devs), "pyvisa didn't find any connected devices matching " + addr
        self.dev = rm.open_resource(devs[0])
        self.dev.timeout = 15_000 * timeout
        self.dev.read_termination = termination
        self.idn = self.dev.query('*IDN?')
        self.read = self.dev.read
        self.write = self.dev.write
        self.query = self.dev.query

    def _get_screenshot(self, verbose=False):
        if verbose == True:
            print('Acquiring screen image...')
        return(self.dev.query_binary_values(':DISPlay:DATA? PNG, COLor', datatype='s'))

    def save_screenshot(self, filepath: str, verbose=False, inksaver=False):
        if verbose == True:
            print('Saving screen image...')
        result = self._get_screenshot()

        # Keysight scope defaults to inksaving for image save
        if inksaver == True:
            self.dev.write(':HARDcopy:INKSaver 1')
        else: 
            self.dev.write(':HARDcopy:INKSaver OFF')

        with open(f'{filepath}', 'wb+') as ofile:
            ofile.write(bytes(result))

    def set_time_delay(self, time: float):
        
        self.write(f':TIMebase:DELay {time}')

    def set_time_scale(self, time: float):
        self.write(f':TIMebase:SCALe {time}')

    def annotate_screen(self, message: str):
        self.write(f':DISPlay:ANNotation:TEXT "{message}"')

    def set_math_scale(self, scale: float):
        self.write(f'FUNCtion:SCALe {scale}')

    def set_math_offset(self, offset: float):
        self.write(f'FUNCtion:OFFSet {offset}')

    def channel_state(self, channel: int, state: int):
        """
        Turn on or off a channel
        state: int (ON: 1 or OFF: 0)
        """
        self.write(f':CHANnel{channel}:DISPlay {state}')

    def set_channel_offset(self, channel: int, y_offset: float):
        self.write(f':CHANnel{channel}:OFFSet {y_offset}')

    def set_channel_scale(self, channel: int, y_scale: float):
        self.write(f':CHANnel{channel}:SCALe {y_scale}')
