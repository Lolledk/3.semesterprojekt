"""Simple LED controller with gpiozero when available, else a dummy fallback.

Usage:
    from tb3_remote_control.gpio_led import LEDController
    led = LEDController(pin=17)
    led.blink(on_time=0.2, off_time=0.2)  # non-blocking
    led.off()

This module keeps the package runnable on non-RPi/dev machines by falling back
to a no-op implementation that logs actions to stdout.
"""
import time


class _DummyLED:
    def __init__(self, pin=None, active_high=True):
        self.pin = pin
        self._state = False

    def on(self):
        self._state = True
        print(f"[DummyLED] ON (pin={self.pin})")

    def off(self):
        self._state = False
        print(f"[DummyLED] OFF (pin={self.pin})")

    def blink(self, on_time=0.5, off_time=0.5, n=None, background=True):
        print(f"[DummyLED] BLINK start (pin={self.pin}) on={on_time}s off={off_time}s n={n} bg={background}")

    def close(self):
        print(f"[DummyLED] close (pin={self.pin})")


class LEDController:
    def __init__(self, pin=17, active_high=True):
        self.pin = pin
        self.available = False
        self._led = None
        try:
            from gpiozero import LED
            # instantiate gpiozero LED (uses BCM pin numbering)
            self._led = LED(pin, active_high=active_high)
            self.available = True
        except Exception:
            # Fallback (prints). This keeps code testable on dev machines.
            self._led = _DummyLED(pin, active_high)
            self.available = False

    def on(self):
        try:
            self._led.on()
        except Exception:
            pass

    def off(self):
        try:
            self._led.off()
        except Exception:
            pass

    def blink(self, on_time=0.5, off_time=0.5, n=None, background=True):
        """Start blinking. For gpiozero this is non-blocking when background=True.

        - on_time/off_time: seconds LED is on/off
        - n: number of cycles (None means infinite)
        - background: if True, returns immediately (recommended)
        """
        try:
            # gpiozero LED and DummyLED both support blink signature used here
            self._led.blink(on_time=on_time, off_time=off_time, n=n, background=background)
        except Exception:
            pass

    def close(self):
        try:
            self._led.close()
        except Exception:
            pass
