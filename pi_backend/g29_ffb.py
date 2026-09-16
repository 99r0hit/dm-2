import evdev
from evdev import ecodes
import glob
import threading
import time

def find_wheel_device():
    for dev_path in sorted(glob.glob("/dev/input/event*")):
        try:
            dev = evdev.InputDevice(dev_path)
            if ecodes.EV_FF in dev.capabilities():
                print(f"✅ Found FFB Device: {dev.name} ({dev_path})")
                return dev
        except Exception:
            pass
    return None

class G29FFB:
    def __init__(self, dev=None):
        self.dev = dev if dev else find_wheel_device()
        if not self.dev:
            raise RuntimeError("❌ No FFB-capable device found on any event node!")
        self._last_autocenter = 0.0
        self._effect_playing = False
        self._idle_hum_active = False
        self._idle_hum_scale = 1.0

        # Pre-allocate effect slots for FF_PERIODIC
        self.hum_effect_id = self._upload_periodic(0x1000, 15)  # Soft high-freq hum
        self.rumble_effect_id = self._upload_periodic(0x7FFF, 30) # Hard mid-freq rumble
        self.terrain_effect_id = self._upload_periodic(0x5FFF, 80) # Bumpy low-freq terrain

    def _upload_periodic(self, magnitude, period):
        try:
            effect = evdev.ff.Effect(
                ecodes.FF_PERIODIC,
                -1,
                0,
                evdev.ff.Trigger(0, 0),
                evdev.ff.Replay(0xFFFF, 0),
                evdev.ff.EffectType(
                    ff_periodic_effect=evdev.ff.Periodic(
                        waveform=ecodes.FF_SINE,
                        period=period,
                        magnitude=magnitude,
                        offset=0,
                        phase=0,
                        envelope=evdev.ff.Envelope(0, 0, 0, 0)
                    )
                )
            )
            return self.dev.upload_effect(effect)
        except Exception as e:
            print(f"Failed to upload effect: {e}")
            return -1

    def _update_effect_magnitude(self, old_effect_id, period, magnitude):
        if old_effect_id < 0: return -1
        try:
            # The G29 driver doesn't support updating effects (Errno 38).
            # We must erase the old one and create a new one.
            self.dev.erase_effect(old_effect_id)
            return self._upload_periodic(magnitude, period)
        except Exception:
            return old_effect_id

    def disable_autocenter(self):
        try:
            self.dev.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, 0)
        except Exception as e:
            pass

    def set_hardware_autocenter(self, strength_pct, _internal=False):
        if not _internal:
            self._last_autocenter = float(strength_pct)
        try:
            magnitude = int(max(0.0, min(100.0, float(strength_pct))) * 655.35)
            self.dev.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, magnitude)
        except Exception as e:
            pass

    def set_autocenter(self, strength_pct):
        self.set_hardware_autocenter(strength_pct)

    def set_force(self, force_val):
        self.set_hardware_autocenter(abs(force_val) * 100.0)

    def stop(self):
        self.stop_idle_hum()
        try:
            self.set_hardware_autocenter(0)
            if self.rumble_effect_id >= 0:
                self.dev.write(ecodes.EV_FF, self.rumble_effect_id, 0)
            if self.terrain_effect_id >= 0:
                self.dev.write(ecodes.EV_FF, self.terrain_effect_id, 0)
        except Exception:
            pass

    def start_idle_hum(self, scale=1.0):
        self._idle_hum_scale = scale
        if getattr(self, '_idle_hum_active', False):
            return
        self._idle_hum_active = True
        
        def _hum():
            try:
                if self.hum_effect_id >= 0:
                    mag = min(0x3FFF, int(0x3FFF * self._idle_hum_scale))
                    self.hum_effect_id = self._update_effect_magnitude(self.hum_effect_id, 15, mag)
                    self.dev.write(ecodes.EV_FF, self.hum_effect_id, 1)
            except Exception:
                pass
            while getattr(self, '_idle_hum_active', False):
                time.sleep(0.1)
            try:
                if self.hum_effect_id >= 0:
                    self.dev.write(ecodes.EV_FF, self.hum_effect_id, 0)
            except Exception:
                pass
        threading.Thread(target=_hum, daemon=True).start()

    def stop_idle_hum(self):
        self._idle_hum_active = False

    def play_kick(self, strength, duration_ms=250):
        def _kick():
            self._effect_playing = True
            try:
                if self.rumble_effect_id >= 0:
                    mag = 0x7FFF # Max magnitude for kick
                    self.rumble_effect_id = self._update_effect_magnitude(self.rumble_effect_id, 40, mag)
                    self.dev.write(ecodes.EV_FF, self.rumble_effect_id, 1)
                time.sleep(duration_ms / 1000.0)
            finally:
                if self.rumble_effect_id >= 0:
                    try:
                        self.dev.write(ecodes.EV_FF, self.rumble_effect_id, 0)
                    except Exception: pass
                self._effect_playing = False
        threading.Thread(target=_kick, daemon=True).start()

    def play_rumble(self, duration_ms=1000, scale=1.0):
        def _play():
            self._effect_playing = True
            try:
                if self.rumble_effect_id >= 0:
                    mag = min(0x7FFF, int(0x7FFF * scale))
                    self.rumble_effect_id = self._update_effect_magnitude(self.rumble_effect_id, 30, mag)
                    self.dev.write(ecodes.EV_FF, self.rumble_effect_id, 1)
                time.sleep(duration_ms / 1000.0)
            finally:
                if self.rumble_effect_id >= 0:
                    try:
                        self.dev.write(ecodes.EV_FF, self.rumble_effect_id, 0)
                    except Exception: pass
                self._effect_playing = False
        threading.Thread(target=_play, daemon=True).start()

    def play_terrain(self, duration_ms=1000, scale=1.0):
        def _play():
            self._effect_playing = True
            try:
                if self.terrain_effect_id >= 0:
                    mag = min(0x5FFF, int(0x5FFF * scale))
                    self.terrain_effect_id = self._update_effect_magnitude(self.terrain_effect_id, 80, mag)
                    self.dev.write(ecodes.EV_FF, self.terrain_effect_id, 1)
                time.sleep(duration_ms / 1000.0)
            finally:
                if self.terrain_effect_id >= 0:
                    try:
                        self.dev.write(ecodes.EV_FF, self.terrain_effect_id, 0)
                    except Exception: pass
                self._effect_playing = False
        threading.Thread(target=_play, daemon=True).start()
