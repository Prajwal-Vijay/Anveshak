# Functions and Flow control
## class IK_IROC
All functions belonging to this class have been assumed to have a parameter named self by default, unless explicitly mentioned

### __init__() - 
Defines the initial values of various variables.

### joyCallback(msg) - 
This is a callback from the 'joy_arm' topic. The msgs are of the type Joy, which is used to get data from the joystick.
- `joy.axes[0]` => refers to left stick horizontal
- Joy.axes[1] => refers to left stick vertical
- Joy.axes[2] => refers to right stick horizontal
- Joy.axes[3] => refers to right stick vertical

Using the input from joystick we move the goal to the required position. 
