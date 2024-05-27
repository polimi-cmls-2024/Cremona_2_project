# Cremona_2_project

Our project consists of a glove with inertial sensors which detects the hand’s movements and collects data which get ent via Bluetooth, using MIDI messages,
to the pc where the sound will be genersted. SuperCollider is used to 
generate different kind of wavesapes that can be chosen trough the GUI. The waves are then filtered by the VST, created in JUCE, to obtain a sound similar 
to human voice. The graphical interface, other than choosing the type of wave, has some knobs needed to change some parameters. The Graphical Interphace 
and the JUCE plugin interface communicates both through OSC messages with SuperCollider.

## Processing

This Processing code creates an interactive graphical interface using the `controlP5` and `oscP5` libraries. The interface includes knob controls, a 
dropdown menu, and a movable circle representing the rotation of the hand and thus the frequencies of the formants. The 2 libraries can be installed 
trough the processing ide.

## SuperCollider

This SuperCollider script sets up the audio processing environment that includes MIDI and OSC (Open Sound Control) comunication, audio generation 
trough the SuperCollider oscilators, and integration with VST plugins for filtering.

### Running procedure

In order to run the SuperCollider code execute the lines outside the first round brackets pair, which are realate to the initialization of the server
and the midi interface. Then the first round bracket block can be executed defining the various synthdefs. The the JUCE plugin must be uploaded by
running line 171, followed by the second round bracket block.

## Arduino

The arduino project was developed on a lolin32 developement board with a dual core esp32 SoC mounted using a MPU6050 for inertial data retrival and a flex sensor
for the note selection. The libraries used `ESP-32-BLEMidi` and `Adafruit-MPU6050` which can be installed trough the arduino ide.

### Building procedure

After installing the 2 libraries and connecting the board to the desired usb port upload the code trough the arduino ide.

## Juce plugin

The filtering plugin responsable for the application of the formant filters was written trough the use of the JUCE framework, the compilation files available
in the project include Windows Linux and MacOSX
