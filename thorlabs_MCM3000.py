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
    def __init__(self,
                 which_port,
                 stages=3*(None,),
                 reverse=3*(False,),
                 verbose=True,
                 very_verbose=False):
        self.verbose = verbose
        self.very_verbose = very_verbose
        if self.verbose: print("Opening MCM3000 controller...", end='')
        try:
            self.port = serial.Serial(port=which_port, baudrate=460800)
        except serial.serialutil.SerialException:
            raise IOError(
                'No connection to MCM3000 controller on port %s'%which_port)
        if self.verbose: print(" done.")
        assert type(stages) == tuple and type(reverse) == tuple
        assert len(stages) == 3 and len(reverse) == 3
        for element in reverse: assert type(element) == bool
        self.stages = stages
        self.channels = (0, 1, 2)
        self.reverse = reverse
        self._stage_limit_um = 3*[None]
        self._stage_conversion_um = 3*[None]
        self._current_encoder_value = 3*[None]
        self._pending_encoder_value = 3*[None]
        supported_stages = { # 'Type': (+-stage_limit_um, stage_conversion_um)
            'ZFM2020':(1e3 * 12.7, 0.2116667), 
            'ZFM2030':(1e3 * 12.7, 0.2116667),}
        for channel, stage in enumerate(stages):
            if stage is not None:
                assert stage in supported_stages, (
                    'stage \'%s\' not supported'%stage)
                self._stage_limit_um[channel] = supported_stages[stage][0]
                self._stage_conversion_um[channel] = supported_stages[stage][1]
                self._current_encoder_value[channel] = (
                    self._get_encoder_value(channel))
        if self.verbose:
            print("stages:", self.stages)
            print("channels:", self.channels)
            print("reverse:", self.reverse)
            print("stage_limit_um:", self._stage_limit_um)
            print("stage_conversion_um:", self._stage_conversion_um)
            print("current_encoder_value:", self._current_encoder_value)

    def _send(self, cmd, channel, response_bytes=None):
        assert channel in self.channels, 'Channel \'%s\' not available'%channel
        assert self.stages[channel] is not None, (
            'Channel %s: stage = None (cannot send command)'%channel)
        self.port.write(cmd)
        if response_bytes is not None:
            response = self.port.read(response_bytes)
        else:
            response = None
        assert self.port.inWaiting() == 0
        return response

    def _get_encoder_value(self, channel):
        channel_byte = channel.to_bytes(1, byteorder='little')
        cmd = b'\x0a\x04' + channel_byte + b'\x00\x00\x00'
        response = self._send(cmd, channel, response_bytes=12)
        assert response[6:7] == channel_byte # channel = selected
        encoder_value = int.from_bytes(
            response[-4:], byteorder='little', signed=True)
        if self.very_verbose:
            print('\nChannel %s: stage encoder value = %i'%(
                channel, encoder_value))
        return encoder_value

    def _set_encoder_value_to_zero(self, channel):
        # WARNING: this device adaptor assumes the stage encoder will be set
        # to zero at the centre of it's range for +- stage_limit_um checks
        if self.verbose:
            encoder_value = self._get_encoder_value(channel)
        channel_byte = channel.to_bytes(2, byteorder='little')
        encoder_bytes = (0).to_bytes(4, 'little', signed=True) # set to zero
        cmd = b'\x09\x04\x06\x00\x00\x00' + channel_byte + encoder_bytes
        self._send(cmd, channel)
        if self.verbose:
            print('Channel %s: waiting for encoder to acknowledge'%channel,
                  're-set to zero')
        while True:
            if encoder_value == 0: break
            encoder_value = self._get_encoder_value(channel)
        self._current_encoder_value[channel] = 0
        if self.verbose:
            print('Channel %s: done with encoder re-set'%channel)
        return None

    def _move_to_encoder_value(self, channel, encoder_value, block=True):
        if self._pending_encoder_value[channel] is not None:
            self._finish_move(channel)
        encoder_bytes = encoder_value.to_bytes(4, 'little', signed=True)
        channel_bytes = channel.to_bytes(2, byteorder='little')
        cmd = b'\x53\x04\x06\x00\x00\x00' + channel_bytes + encoder_bytes
        self._send(cmd, channel)
        self._pending_encoder_value[channel] = encoder_value
        if self.very_verbose:
            print('Channel %s: moving stage encoder to value = %i'%(
                channel, encoder_value))
        if block:
            self._finish_move(channel)
        return None

    def _finish_move(self, channel, polling_wait_s=0.1):
        if self._pending_encoder_value[channel] is None:
            return
        current_encoder_value = self._get_encoder_value(channel)
        while True:
            if current_encoder_value == self._pending_encoder_value[channel]:
                break
            if self.verbose: print('.', end='')
            time.sleep(polling_wait_s)
            current_encoder_value = self._get_encoder_value(channel)
        self._current_encoder_value[channel] = current_encoder_value
        current_position_um = self._um_from_encoder_value(
            channel, current_encoder_value)
        if self.verbose:
            print('\nChannel %s: finished moving to position_um = %0.2f'%(
                channel, current_position_um))
        self._pending_encoder_value[channel] = None
        return current_encoder_value, current_position_um

    def _um_from_encoder_value(self, channel, encoder_value):
        um = encoder_value * self._stage_conversion_um[channel]
        if self.reverse[channel]: um = -um
        return um + 0 # avoid -0.0

    def _encoder_value_from_um(self, channel, um):
        encoder_value = int(um / self._stage_conversion_um[channel])
        if self.reverse[channel]: encoder_value = -encoder_value
        return encoder_value + 0 # avoid -0.0

    def get_position_um(self, channel):
        encoder_value = self._get_encoder_value(channel)
        position_um = self._um_from_encoder_value(channel, encoder_value)
        if self.verbose:
            print('Channel %s: stage position_um = %0.2f'%(
                channel, position_um))
        return position_um

    def legalize_move_um(self, channel, move_um, relative=True, verbose=True):
        encoder_value = self._encoder_value_from_um(channel, move_um)
        if relative:
            if self._pending_encoder_value[channel] is not None:
                encoder_value += self._pending_encoder_value[channel]
            else:
                encoder_value += self._current_encoder_value[channel]
        target_move_um = self._um_from_encoder_value(channel, encoder_value)
        limit_um = self._stage_limit_um[channel]
        assert -limit_um < target_move_um < limit_um, (
            'requested move_um (%0.2f) exceeds limit_um (%0.2f)'%(
                target_move_um, limit_um))
        legal_move_um = target_move_um
        if verbose:
            print('Legalized move_um: %0.2f (%0.2f requested, relative=%s)'%(
                legal_move_um, move_um, relative))
        return legal_move_um

    def move_um(self, channel, move_um, relative=True, block=True):
        legal_move_um = self.legalize_move_um(
            channel, move_um, relative, verbose=False)
        if self.verbose:
            print('Channel %s: moving to position_um = %0.2f'%(
                channel, legal_move_um), end='')
        encoder_value = self._encoder_value_from_um(channel, legal_move_um)
        self._move_to_encoder_value(channel, encoder_value, block)
        if block:
            self._finish_move(channel)
        else:
            print('\n')
        return legal_move_um

    def close(self):
        if self.verbose: print("Closing MCM3000 controller...", end=' ')
        self.port.close()
        if self.verbose: print("done.")
        return None

if __name__ == '__main__':
    channel = 2
    stage_controller = Controller(
        'COM4',stages=(None, None, 'ZFM2030'),reverse=(False, False, True))

    print('\n# Get position:')
    stage_controller.get_position_um(channel)

    print('\n# Some relative moves:')
    for moves in range(3):
        move = stage_controller.move_um(channel, 10)
    for moves in range(3):
        move = stage_controller.move_um(channel, -10)

    print('\n# Legalized move:')
    legal_move_um = stage_controller.legalize_move_um(channel, 10)
    stage_controller.move_um(channel, legal_move_um)

    print('\n# Some random absolute moves:')
    from random import randrange
    for moves in range(3):
        random_move_um = randrange(-100, 100)
        move = stage_controller.move_um(channel, random_move_um, relative=False)

    print('\n# Non-blocking move:')
    stage_controller.move_um(channel, 200, block=False)
    stage_controller.move_um(channel, 100, block=False)
    print('(immediate follow up call forces finish on pending move)')
    print('doing something else')
    stage_controller._finish_move(channel)

    print('\n# Home:')
    stage_controller.move_um(channel, 0, relative=False)

    # re-set zero:
##    stage_controller.move_um(channel, 10)
##    stage_controller._set_encoder_value_to_zero(channel)
##    stage_controller.move_um(channel, 0)

    stage_controller.close()
