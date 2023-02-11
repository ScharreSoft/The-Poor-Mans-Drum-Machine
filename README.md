# The-Poor-Man's-Drum Machine
A poor man's version of the legendary Roland TR909 Drum Machine

The PM-909 is basically a 16 step sequencer that mimicks as much as posible the TR909. The circuit can be used to control drum modules from the TR-909 drum machine. It can also be used to control any set of drum modules that accept 5 Volt triggers and CV (control voltages) from 0-5V for the accent input. 


Features:

·   Open Source

·   High / Low Accent (adjustable for each voice individually)

·   Flam on all voices  (if flam is set to maximal, 32th notes can be played)

·   Shuffle on all voices (if shuffle is set to maximal, all even notes are delayed a 32th note)

·   Create, play and edit drumtracks consisting of 48 different drum patterns. A drum track can have a length of 224 measures (patterns)

·   Create patterns using Pattern Write mode and Tap Write mode

.   Pattern Play mode: create tracks by playing different patterns on the fly

·   Voice Mute mode: mute and un-mute voices on the fly for extra variation and improvisation

·   Save up to 36 drum tracks on an EEPROM

·   Internal clock and external clockable (e.g. using LFO or MIDI)

·   External Reset input and Clock Out output

·   MIDI in  The module can be synced by an external MIDI clock (e.g. a DAW) to play a programmed rhythm

·   The module can play a MIDI drumtrack that was programmed in a DAW (with full velocity control). (only tested with Ableton Live)

·   MIDI out:  A drumtrack can be recorded in a DAW. (This is somewhat rudimentary)

·   Customizable through simple confirguration file

·   Create random rhythm patterns:   (some of the random patterns are quite OK )

·   Random improvisation on an existing pattern


By default, the firmware and the circuit are configured to control the 11 standard drum sounds that are found in the TR-909. Both firmware and hardware are easily adjusted to control upto 16 different drum modules of any kind, as long as they accept 5 volt triggers and 0-5 volt accent CV. If a higher level of CV is required (for instance, 15 volt CV  when using TR-808 modules) the opamp buffers at the CV output should be adjusted to have a gain of 3. 

