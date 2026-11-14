
from array import array

# Quadrature transition lookup table
# Forward: 00 -> 01 -> 11 -> 10 -> 00
_STEPS = (
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
)

# Waiting for Sanyam to confirm the actual pins
ENCODER_PINS = {
    1: (None, None),
    2: (None, None),
    3: (None, None)
}


class QuadratureIRQEncoder:
    def __init__(self, pin_a, pin_b, reverse=False,
                 pin_class=None, disable_irq=None,
                 enable_irq=None):

        if pin_a is None or pin_b is None or pin_a == pin_b:
            raise ValueError("Supply two distinct encoder GPIO pins")

        # Real Pico hardware, unless fake pins are provided
        if pin_class is None or disable_irq is None or enable_irq is None:
            import machine
            if pin_class is None:
                pin_class = machine.Pin
            if disable_irq is None:
                disable_irq = machine.disable_irq
            if enable_irq is None:
                enable_irq = machine.enable_irq

        self._disable_irq = disable_irq
        self._enable_irq = enable_irq

        self._a = pin_class(pin_a, pin_class.IN)
        self._b = pin_class(pin_b, pin_class.IN)
        self._sign = -1 if reverse else 1

        self._counts = array("i", [0])
        self._previous = (
            (self._a.value() << 1) | self._b.value()
        )

        # Detect rising and falling edges on both channels
        self._callback = self._on_edge
        edges = pin_class.IRQ_RISING | pin_class.IRQ_FALLING
        self._a.irq(trigger=edges, handler=self._callback)
        self._b.irq(trigger=edges, handler=self._callback)

    def _on_edge(self, pin):
        # Keep interrupt handling short
        state = (self._a.value() << 1) | self._b.value()

        change = _STEPS[
            (self._previous << 2) | state
        ]

        self._counts[0] += self._sign * change
        self._previous = state

    def read_count(self):
        # Read safely while interrupts are temporarily disabled
        irq_state = self._disable_irq()
        try:
            return self._counts[0]
        finally:
            self._enable_irq(irq_state)

    def reset(self):
        irq_state = self._disable_irq()
        try:
            self._counts[0] = 0
            self._previous = (
                (self._a.value() << 1) | self._b.value()
            )
        finally:
            self._enable_irq(irq_state)


# Test using simulated GPIO pins
if __name__ == "__main__":

    class FakePin:
        IN = 0
        IRQ_RISING = 1
        IRQ_FALLING = 2
        pins = {}

        def __init__(self, number, mode):
            self._value = 0
            self._handler = None
            self.pins[number] = self

        def value(self):
            return self._value

        def irq(self, trigger, handler):
            self._handler = handler

        def drive(self, value):
            if value != self._value:
                self._value = value
                self._handler(self)

    def make_encoder(a, b, reverse=False):
        return QuadratureIRQEncoder(
            a, b,
            reverse=reverse,
            pin_class=FakePin,
            disable_irq=lambda: 0,
            enable_irq=lambda state: None
        )

    def signals(a, b, sequence):
        for a_value, b_value in sequence:
            FakePin.pins[a].drive(a_value)
            FakePin.pins[b].drive(b_value)

    encoder = make_encoder(11, 12)

    # One forward quadrature cycle
    signals(11, 12, [
        (0, 1), (1, 1), (1, 0), (0, 0)
    ])

    assert encoder.read_count() == 4
    print("PASS: Forward rotation = +4 counts")

    # One backward cycle
    signals(11, 12, [
        (1, 0), (1, 1), (0, 1), (0, 0)
    ])

    assert encoder.read_count() == 0
    print("PASS: Backward rotation restores zero")

    encoder.reset()
    assert encoder.read_count() == 0
    print("PASS: Reset counts")

    # Test reversed encoder polarity
    reversed_encoder = make_encoder(13, 14, reverse=True)

    signals(13, 14, [
        (0, 1), (1, 1), (1, 0), (0, 0)
    ])

    assert reversed_encoder.read_count() == -4
    print("PASS: Reverse polarity = -4 counts")

    print("All simulated GPIO encoder tests passed!")
