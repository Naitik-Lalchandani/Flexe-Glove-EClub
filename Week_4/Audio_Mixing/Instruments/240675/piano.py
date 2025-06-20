from pynput import keyboard
import mido
from mido import Message

# MIDI output port
port_name = 'Virtual Raw MIDI 2-0:VirMIDI 2-0 24:0'
outport = mido.open_output(port_name)

# Key-to-note mapping (white keys)
key_map = {
    'a': 60,  # C4
    's': 62,  # D4
    'd': 64,  # E4
    'f': 65,  # F4
    'g': 67,  # G4
    'h': 69,  # A4
    'j': 71   # B4
}

# Add black keys
key_map.update({
    'w': 61,  # C#
    'e': 63,  # D#
    't': 66,  # F#
    'y': 68,  # G#
    'u': 70   # A#
})

# Track currently pressed keys to avoid repeats
pressed_keys = set()

def on_press(key):
    try:
        k = key.char.lower()
        if k in key_map and k not in pressed_keys:
            note = key_map[k]
            msg = Message('note_on', note=note, velocity=100)
            outport.send(msg)
            print(f"Note ON: {note}")
            pressed_keys.add(k)
    except AttributeError:
        pass

def on_release(key):
    try:
        k = key.char.lower()
        if k in key_map:
            note = key_map[k]
            msg = Message('note_off', note=note, velocity=0)
            outport.send(msg)
            print(f"Note OFF: {note}")
            pressed_keys.discard(k)
    except AttributeError:
        pass

# Listen forever
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
