# K9 System ROS 2 Nodes
These ROS 2 nodes work together to create a real robot K9 that can do everything from playing chessðŸ˜‰ to following you around the house.

## Back Lights and Side Screen
A node that:
* turns lights on or off (Trigger)
* turns K9's side screen on or off (Trigger)
* set patterns of back light activity (LightsControl)
* retrieves the status of K9's back panel switches (SwitchState)

## Ears
A node that controls the LIDAR ears on K9, specifically via a Trigger it can:
* stop the ears
* make them scan
* make them move quickly
* make them move as if he is thinking
* put them in follow mode
* put them in safe rotate mode

## Eyes
A node that controls the face panel on K9. It subscribes to the 'is_talking' topic to automatically temporarily brighten the lights when K9 is talking. It also responds to:
* set brightness (SetBrightness)
* get brightness (GetBrightness)
* turn on (Trigger)
* turn off (Trigger)

## Tail
A simple node that responds to Triggers that enables the tail to:
* Wag horizontally
* Wag vertically
* Cemtre the tail
* Raise the tail
* Lower the tail

## Voice
A complex node that enables K9 to speak on a FCFS via a Piper custom speech model. Subscribes to "tts_input" to get regular speech commands and places them in a queue. It publishes the 'is_talking' topic when the robot is talking.
It can also receive commands to:
* stop talking immediately (CancelSpeech)
* pre-emptively make an announcement (Speak)

## K9 Client
Provides a simple set of Python classes that wrap these ROS2 Nodes. The objects and interfaces are generally identical to
those used on the non-ROS version of K9 and can be used to write simple programs without knowledge of ROS.

## Context
This node aggregates information received from other nodes and publishes a context message.

## Ollama Wrap
This node wraps generative LLMs so that a message in English can be turned into a phrase that K9 might say.

