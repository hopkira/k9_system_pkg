# K9 System ROS 2 Nodes
These ROS 2 nodes work together to create a real robot K9 that can do everything from:
* playing chess
* following you around the house
* to telling you:
   * the best time to go for a walk
   * when your next Google Calendar appointment is
   * a list of tasks in the garden based on weather and month

```mermaid
flowchart LR
  %% =========================
  %% Layer 5 — Behaviours / Brain
  %% =========================
  subgraph L5["Layer 5 — Behaviours / Brain"]
    BEH[k9_behavior_orchestrator]
    CTX[k9_context_aggregator]
    LLM[k9_ollama_service]
    VOICE[k9_voice_node]
  end

  %% =========================
  %% Layer 4 — Navigation & SLAM
  %% =========================
  subgraph L4["Layer 4 — Navigation / SLAM"]
    BT[nav2_bt_navigator]
    PL[nav2_planner_server]
    CTRL[nav2_controller_server]
    CM[nav2_costmaps]
    SLAM[slam_toolbox]
  end

  %% =========================
  %% Layer 3 — Robot State & TF
  %% =========================
  subgraph L3["Layer 3 — Robot State / TF"]
    JS[/joint_states/]
    RSP[robot_state_publisher]
    TF[/tf/]
    MAP[/map/]
    ODOM[/odom/]
  end

  %% =========================
  %% Layer 2 — Command Arbitration
  %% =========================
  subgraph L2["Layer 2 — Command Arbitration"]
    MUX[twist_mux]
  end

  %% =========================
  %% Layer 1 — Drivers & Sensors
  %% =========================
  subgraph L1["Layer 1 — Drivers & Sensors"]
    BASE[roboclaw_driver<br/>(or Gazebo base plugin)]
    LD06[ld06_lidar_driver]
    EARS[ears_node]
    JOY[teleop_twist_joy]
  end

  %% =========================
  %% Key Topics (explicit)
  %% =========================
  CMDNAV[/cmd_vel_nav/]
  CMDJOY[/cmd_vel_joy/]
  CMDBEH[/cmd_vel_beh/]
  CMD[/cmd_vel/]
  SCAN[/scan/]
  EARSCAN[/ears/scan/]
  KCTX[/k9/context/]
  SAY[/k9/say/]
  NAVACT["Action: NavigateToPose"]

  %% =========================
  %% Behaviour & Speech Flow
  %% =========================
  BEH --> NAVACT --> BT
  BEH -->|context strings| KCTX --> CTX --> LLM --> SAY --> VOICE

  %% =========================
  %% Nav2 & SLAM Flow
  %% =========================
  PL --> BT
  CM --> CTRL
  CTRL -->|publishes| CMDNAV
  SCAN --> CM
  TF --> CM
  ODOM --> CTRL
  TF --> CTRL

  SCAN --> SLAM
  ODOM --> SLAM
  SLAM --> MAP
  SLAM --> TF

  %% =========================
  %% Twist Multiplexing
  %% =========================
  CMDNAV --> MUX
  CMDJOY --> MUX
  CMDBEH --> MUX
  MUX --> CMD

  %% =========================
  %% Base & Sensors
  %% =========================
  CMD --> BASE
  BASE --> ODOM
  BASE --> JS

  JS --> RSP --> TF

  LD06 --> SCAN
  EARS --> EARSCAN
  EARS --> JS

  JOY --> CMDJOY

  %% =========================
  %% Behaviour subscriptions
  %% =========================
  EARSCAN --> BEH
  SCAN --> BEH
  TF --> BEH


## Back Lights and Side Screen
A node that:
* turns lights on or off (back_lights_on; back_lights_off)
* turns K9's side screen on or off (Trigger)
* set custom patterns of back lights (LightsControl)
* retrieves the status of K9's back panel switches (SwitchState)

Use the `/back_lights_cmd` topic to send instructions to change the pattern of lights.  Current patterns include: original, colour, diagonal, two, three, four, six, red, green, blue, spiral, chase_v, chase_h, cols, rows, on, off. Speeds include fastest, fast, normal, slow, slowest.

```
ros2 service call /back_lights_on std_srvs/srv/Trigger`
ros2 topic pub --once /back_lights_cmd std_msgs/msg/String "{data: 'blue'}"
```

## Ears
A node that controls the LIDAR ears on K9, specifically via a Trigger it can:
* stop the ears (ears_stop)
* make them scan (ears_scan)
* make them move quickly (ears_fast)
* make them move as if he is thinking (ears_think)
* put them in follow mode (ears_follow_read)
* put them in safe rotate mode (ears_safe_rotate)

## Eyes and Tail
A node that controls the servo controller in K9; this means it controls both the eyes and the tail.
* For the face panel on K9. It subscribes to the 'is_talking' topic to automatically temporarily brighten the lights when K9 is talking. It also responds to:
    * set brightness (eyes_set_level)
    * get brightness (eyes_get_brightness)
    * turn on (tv_on)
    * turn off (tv_off)
* For the tail, it responds to Triggers that enables the tail to:
    * Wag horizontally (tail_wag_h)
    * Wag vertically (tail_wag v)
    * Cemtre the tail (tail_centre)
    * Raise the tail (tail_up)
    * Lower the tail (tail_down)

```
ros2 service call /tail_wag_v std_srvs/srv/Trigger
ros2 service call /eyes_on std_srvs/srv/Trigger
ros2 service call /eyes_set_level k9_interfaces_pkg/srv/SetBrightness "{level: 0.01}"
```

## Voice
A complex node that enables K9 to speak on a FCFS via a Piper custom speech model. Subscribes to "tts_input" to get regular speech commands and places them in a queue. It publishes the 'is_talking' topic when the robot is talking.
It can also receive commands to:
* stop talking immediately (CancelSpeech)
* pre-emptively make an announcement (Speak)

```
ros2 topic echo /is_talking
ros2 topic pub /tts_input std_msgs/String "{data: 'I am a very clever robot called K9'}" -1
ros2 service call /speak_now k9_interfaces_pkg/srv/Speak "{text: 'Hello'}"
ros2 service call /cancel_speech k9_interfaces_pkg/srv/CancelSpeech
```

## Hotword
A node that is activated by a service call and then listens for the "canine" hotword. When it hears that word, it closes itself down and publishes to the 'hotword_detected' topic.

## Calendar
A node that works with Google Calendar. It offers two services that announce the next appointment or the whole day's worth of appointments - or if there is an appointment in the next five minutes it will provide a reminder. Uses the topic subscribed to by the Voice node to make the announcements verbal.

## K9 Client
Provides a simple set of Python classes that wrap these ROS2 Nodes. The objects and interfaces are generally identical to
those used on the non-ROS version of K9 and can be used to write simple programs without knowledge of ROS.

## Context
This node aggregates information received from other nodes and publishes a context message.

## Ollama Wrap
This node wraps generative LLMs so that a message in English can be turned into a phrase that K9 might say.

