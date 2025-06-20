import mido
import time

port_name = 'Virtual Raw MIDI 2-0:VirMIDI 2-0 24:0'

try:
    outport = mido.open_output(port_name)
    print(f"Connected to: {port_name}")
except Exception as e:
    print(f"Failed to open MIDI port: {e}")
    exit()

# Send a Control Change message: CC 7 (Volume), value 100, channel 0
cc_msg = mido.Message('control_change', control=7, value=100, channel=0)
outport.send(cc_msg)
print(f"Sent CC → control: {cc_msg.control}, value: {cc_msg.value}, channel: {cc_msg.channel}")

# Send a Note On then Note Off: Note 60 (Middle C), velocity 100
note_on = mido.Message('note_on', note=60, velocity=100, channel=0)
note_off = mido.Message('note_off', note=60, velocity=0, channel=0)

outport.send(note_on)
print(f"Sent Note ON: note {note_on.note}, velocity {note_on.velocity}")
time.sleep(0.5)
outport.send(note_off)
print(f"Sent Note OFF: note {note_off.note}, velocity {note_off.velocity}")

outport.close()
