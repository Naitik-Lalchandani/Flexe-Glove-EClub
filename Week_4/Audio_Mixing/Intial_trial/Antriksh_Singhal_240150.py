import time
import signal
import rtmidi

# Create virtual MIDI port
midiout = rtmidi.MidiOut()
available = midiout.get_ports()
# Open matching VirMIDI port if found, else default
for i, name in enumerate(available):
    if 'VirMIDI 1-0' in name:
        midiout.open_port(i)
        print(f"Opened MIDI port: {name}")
        break
else:
    midiout.open_port(0)
    print(f"Opened default MIDI port: {available[0] if available else 'None'}")

# Graceful shutdown: send CC1=0 on exit
def cleanup(signum, frame):
    midiout.send_message([0xB0, 1, 0])  # CC1=0
    print("Sent CC1 -> 0 (cleanup)")
    midiout.close_port()
    exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

try:
    # Ramp CC#1 from 0 to 127
    for val in range(0, 128, 5):
        midiout.send_message([0xB0, 1, val])  # CC1
        print(f"Sent CC1 -> {val}")
        time.sleep(0.1)
    # Hold final level briefly
    time.sleep(1)
    # Fade back to 0
    for val in range(127, -1, -5):
        midiout.send_message([0xB0, 1, val])
        print(f"Sent CC1 -> {val}")
        time.sleep(0.1)
finally:
    # Ensure final reset
    midiout.send_message([0xB0, 1, 0])
    print("Sent CC1 -> 0 (final)")
    midiout.close_port()
    print("Closed MIDI port")
