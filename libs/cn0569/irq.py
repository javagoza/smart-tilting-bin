"""Edge waiting for the ADPD1080 INT line (GPIO0 on the Pmod INT pin) with libgpiod v2.

    pip install gpiod        # Python bindings, API v2

Usage with the driver:

    irq = InterruptLine(25)                       # BCM GPIO number wired to Pmod INT
    dev.use_interrupt(irq.wait, packets=2)        # after dev.init(), before dev.start()
"""


class InterruptLine:
    def __init__(self, pin, chip="/dev/gpiochip0", active_low=False, _gpiod=None):
        """pin: BCM GPIO number. active_low must match the polarity configured in the
        chip (configure_interrupt_pin): active high -> rising edge, active low -> falling."""
        if _gpiod is None:  # injectable for the unit tests
            import gpiod as _gpiod
            from gpiod.line import Edge
        else:
            Edge = _gpiod.line.Edge
        edge = Edge.FALLING if active_low else Edge.RISING
        self.pin = pin
        self._req = _gpiod.request_lines(
            chip,
            consumer="adpd1080",
            config={pin: _gpiod.LineSettings(edge_detection=edge)},
        )

    def wait(self, timeout=None):
        """Block until an edge arrives (True) or the timeout in seconds expires (False).
        Queued edges are drained, so one call returns after any number of edges."""
        if self._req.wait_edge_events(timeout):
            self._req.read_edge_events()
            return True
        return False

    def close(self):
        self._req.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
