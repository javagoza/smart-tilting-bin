"""Hardware backends. The driver only talks to this small interface."""
import abc
import logging

log = logging.getLogger("stepper28byj")


class Backend(abc.ABC):
    @abc.abstractmethod
    def set_coils(self, pattern):
        """Drive the 4 ULN2003 inputs with a tuple of 4 bits."""

    @abc.abstractmethod
    def read_switches(self):
        """Return (home_active, end_active) as booleans (logical level, polarity applied)."""

    @abc.abstractmethod
    def release(self):
        """De-energise all coils."""

    @abc.abstractmethod
    def close(self):
        """Release the hardware."""


class GpiodBackend(Backend):
    """Raspberry Pi GPIO through libgpiod v2 (pip install gpiod).

    One line request holds the 4 outputs and the limit-switch inputs, so all coil
    pins change with a single call.
    """

    def __init__(self, cfg, _gpiod=None):
        if _gpiod is None:  # injectable for the unit tests
            import gpiod as _gpiod
            from gpiod.line import Bias, Direction, Value
        else:
            Bias, Direction, Value = _gpiod.line.Bias, _gpiod.line.Direction, _gpiod.line.Value
        self._Value = Value
        self._coil_pins = tuple(cfg.coil_pins)
        bias = {"pull_up": Bias.PULL_UP, "pull_down": Bias.PULL_DOWN, "disabled": Bias.DISABLED}
        lines = {
            pin: _gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)
            for pin in self._coil_pins
        }
        self._home = cfg.home_switch
        self._end = cfg.end_switch
        self._sw_pins = []
        for sw in (self._home, self._end):
            if sw is not None:
                lines[sw.pin] = _gpiod.LineSettings(
                    direction=Direction.INPUT, bias=bias[sw.bias], active_low=sw.active_low
                )
                self._sw_pins.append(sw.pin)
        self._req = _gpiod.request_lines(cfg.chip, consumer="stepper28byj", config=lines)
        self._cache = {}
        self._off = {pin: Value.INACTIVE for pin in self._coil_pins}

    def set_coils(self, pattern):
        values = self._cache.get(pattern)
        if values is None:
            values = {
                pin: (self._Value.ACTIVE if bit else self._Value.INACTIVE)
                for pin, bit in zip(self._coil_pins, pattern)
            }
            self._cache[pattern] = values
        self._req.set_values(values)

    def read_switches(self):
        if not self._sw_pins:
            return False, False
        vals = iter(self._req.get_values(self._sw_pins))
        home = (next(vals) == self._Value.ACTIVE) if self._home is not None else False
        end = (next(vals) == self._Value.ACTIVE) if self._end is not None else False
        return home, end

    def release(self):
        self._req.set_values(self._off)

    def close(self):
        try:
            self.release()
        finally:
            self._req.release()
