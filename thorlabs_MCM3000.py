import time
import serial

class Controller:
    '''
    Basic device adaptor for thorlabs MCM3000 and MCM3001 3-axis controllers.
    Not implemented:
    - stop function (not useful?)
    - query motor status (not working? documentation error?)
    Test code runs and seems robust.
    '''
    def __init__(
        self, which_port, stages=3*(None,), reverse=3*(False,), verbose=True):
        self.verbose = verbose
        if self.verbose: print("Opening MCM3000 controller...", end='')
        try:
            self.port = serial.Serial(port=which_port, baudrate=460800)
        except serial.serialutil.SerialException:
            raise IOError(
                'No connection to MCM3000 controller on port %s'%which_port)
        if self.verbose: print(" done.")
        assert type(stages) == tuple and type(reverse) == tuple
        assert 0 < len(stages) <= 3, '1 to 3 stages allowed'
        assert len(reverse) == len(stages)
        self.stages = stages
        self.reverse = reverse
        self.channels = tuple(range(0, len(stages)))
        supported_stages = {
            None:(0,0), # 'type':(limit_um, conversion_um)
            'ZFM2020':(1e3 * 12.7, 0.2116667),
            'ZFM2030':(1e3 * 12.7, 0.2116667),
            }
        self.stage_limits_um = []
        self.stage_conversion_um = []
        for stage in stages:
            assert stage in supported_stages, (
                'stage \'%s\' not supported'%stage)
            self.stage_limits_um.append(supported_stages[stage][0])
            self.stage_conversion_um.append(supported_stages[stage][1])
        if self.verbose:
            print("stages:", self.stages)
            print("stage_channels:", self.channels)
            print("stage_limits_um:", self.stage_limits_um)
            print("stage_conversion_um:", self.stage_conversion_um)

    def _send(self, cmd, channel, response_bytes=None):
        assert channel in self.channels, 'channel \'%s\' not available'%channel
        assert self.stages[channel] is not None, (
            'for channel %s, stage = None (cannot send command)'%channel)
        self.port.write(cmd)
        if response_bytes is not None:
            response = self.port.read(response_bytes)
        else:
            response = None
        assert self.port.inWaiting() == 0
        return response

    def _get_encoder_value(self, channel, verbose=False):
        channel_byte = channel.to_bytes(1, byteorder='little')
        cmd = b'\x0a\x04' + channel_byte + b'\x00\x00\x00'
        response = self._send(cmd, channel, response_bytes=12)
        assert response[6:7] == channel_byte # channel = selected
        encoder_value = int.from_bytes(
            response[-4:], byteorder='little', signed=True)
        if self.reverse[channel]: encoder_value = -encoder_value
        if verbose:
            print('Channel %s stage encoder value: %i'%(channel, encoder_value))
        return encoder_value

    def _set_encoder_value_to_zero(self, channel):
        if self.verbose:
            encoder_value = self._get_encoder_value(channel)
        channel_byte = channel.to_bytes(2, byteorder='little')
        encoder_bytes = (0).to_bytes(4, 'little', signed=True) # set to zero
        cmd = b'\x09\x04\x06\x00\x00\x00' + channel_byte + encoder_bytes
        self._send(cmd, channel)
        if self.verbose:
            print('Waiting for channel %s encoder to acknowledge'%channel,
                  're-set to zero:')
        while True:
            if encoder_value == 0: break
            encoder_value = self._get_encoder_value(channel, verbose=True)
        if self.verbose:
            print('Done with encoder re-set')
        return None

    def _move_to_encoder_value(self, channel, encoder_value, verbose=False):
        current_encoder_value = self._get_encoder_value(channel)
        if self.reverse[channel]: encoder_value = -encoder_value
        encoder_bytes = encoder_value.to_bytes(4, 'little', signed=True)
        channel_bytes = channel.to_bytes(2, byteorder='little')
        cmd = b'\x53\x04\x06\x00\x00\x00' + channel_bytes + encoder_bytes
        self._send(cmd, channel)
        if verbose:
            print('Moving channel %s stage encoder value to: %i'%(
                channel, encoder_value), end='')
        return None

    def get_position_um(self, channel, verbose=True):
        encoder_value = self._get_encoder_value(channel)
        position_um = encoder_value * self.stage_conversion_um[channel]
        if verbose:
            print('Channel %s stage position_um: %0.2f'%(channel, position_um))
        return position_um

    def move_um(
        self, channel, move_um, relative=True, block=True, polling_wait_s=0.1):
        limit_um = self.stage_limits_um[channel]
        assert -limit_um < move_um < limit_um, ( # assumes symetric about zero
            'requested move_um (%0.2f) exceeds limit_um (%0.2f)'%(
                move_um, limit_um))
        current_encoder_value = self._get_encoder_value(channel)
        target_encoder_value = int(move_um / self.stage_conversion_um[channel])
        if relative:
            target_encoder_value = current_encoder_value + target_encoder_value
        position_um = target_encoder_value * self.stage_conversion_um[channel]
        if self.verbose:
            print('Moving channel %s to position_um: %0.2f...'%(
                channel, position_um), end='')
        self._move_to_encoder_value(channel, target_encoder_value)
        while block:
            if current_encoder_value == target_encoder_value:
                break
            if self.verbose:
                print('.', end='')
            time.sleep(polling_wait_s)
            current_encoder_value = self._get_encoder_value(channel)
        if self.verbose and block:
            print(' done')
            self.get_position_um(channel)
        return None

    def legalize_move_um(self, channel, move_um):
        encoder_value = int(move_um / self.stage_conversion_um[channel])
        legal_move_um = encoder_value * self.stage_conversion_um[channel]
        if self.verbose:
            print('Legalized move_um: %0.2f (%0.2f requested)'%(
                legal_move_um, move_um))
        return legal_move_um

    def close(self):
        if self.verbose: print("Closing MCM3000 controller...", end=' ')
        self.port.close()
        if self.verbose: print("done.")
        return None

if __name__ == '__main__':
    # init:
    stage_controller = Controller(
        'COM4', stages=(None, None, 'ZFM2030'), reverse=(False, False, True))
    channel = 2
    
    # get position:
    stage_controller.get_position_um(channel)

    # relative moves:
    print('Some relative moves:')
    for moves in range(3):
        stage_controller.move_um(channel, 10)
    for moves in range(3):
        stage_controller.move_um(channel, -10)

    # legalized moves:
    legal_move_um = stage_controller.legalize_move_um(channel, 10)
    stage_controller.move_um(channel, legal_move_um)

    # random absolute moves:
    print('Some random absolute moves:')
    from random import randrange
    for moves in range(3):
        random_move_um = randrange(-100, 100)
        stage_controller.move_um(channel, random_move_um, relative=False)

    # home:
    print('Home with non-blocking call:')
    stage_controller.move_um(channel, 0, relative=False, block=False)
    print('doing something else')

##    # re-set zero:
##    stage_controller.move_um(channel, 10)
##    stage_controller._set_encoder_value_to_zero(channel)
##    stage_controller.move_um(channel, 0)
    
    stage_controller.close()
