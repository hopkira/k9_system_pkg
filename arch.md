flowchart LR
  subgraph L7 [High Level Behaviours]
    B[k9_behavior_orchestrator]
    C[k9_context_aggregator]
    O[k9_ollama_service]
    V[k9_voice_node]
    NAVACT[Action: /navigate_to_pose]
    OLL[Service: /ollama_generate]
    SAY[/k9/say/]
    KCTX[/k9/context/]
    CMDBEH[/cmd_vel_beh/]
  end

  subgraph L1[Manual Controls]
    JOY[joy]   
    JOYTOP[/joy/]
    CMDJOY[/cmd_vel_joy/]
    TEL[teleop_twist_joy]
  end

  subgraph L6 [Navigation & SLAM]
    BT[nav2_bt_navigator]
    PL[nav2_planner_server]
    CT[nav2_controller_server]
    CM[nav2_costmaps]
    SL[slam_toolbox]
    MAP[/map/]
    CMDNAV[/cmd_vel_nav/]
  end

  subgraph L5 [Robot State / TF]
    RSP[robot_state_publisher]
    TF[/tf/]
    EARS[ears]
    LEAR[/l_ear_joint/]
    REAR[/r_ear_joint/]
  end

  subgraph L4 [Command Arbitration]
    MUX[twist_mux]
    CMD[/cmd_vel/]
  end

  subgraph L2 [Drivers & Sensors]
    LD[LD06 LIDAR]
    3D[Oak-D camera]
    LEFT[Left ear ToF sensor]
    RIGHT[Right ear ToF sensor]
    BASE[roboclaw]
    SCAN[/scan/]
    ODOM[/odom/]
  end


  B --> NAVACT --> BT
  PL --> BT
  CM --> CT
  CT --> CMDNAV --> MUX
  JOY --> JOYTOP --> TEL --> CMDJOY --> MUX
  B --> CMDBEH --> MUX
  MUX --> CMD --> BASE --> ODOM
  LD --> SCAN
  3D --> SCAN
  SCAN --> CM
  ODOM --> CT
  SCAN --> SL
  ODOM --> SL
  SL --> MAP
  SL --> TF
  LEFT --> SCAN
  RIGHT --> SCAN

  TF --> L6

  EARS --> LEAR --> RSP --> TF
  EARS --> REAR --> RSP

  B --> KCTX --> C --> OLL --> O --> SAY --> V
