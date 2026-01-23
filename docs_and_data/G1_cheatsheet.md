# Unitree G1 Robot - Isaac Sim 5.1.0 Complete Guide

A comprehensive cheatsheet for developing production-ready code for the Unitree G1 humanoid robot in NVIDIA Isaac Sim 5.1.0.

---

## Table of Contents

1. [Isaac Sim Fundamentals](#1-isaac-sim-fundamentals)
2. [G1 Robot Joint Analysis](#2-g1-robot-joint-analysis)
3. [Safety Considerations](#3-safety-considerations)
4. [Production-Ready Code Guide](#4-production-ready-code-guide)

---

## 1. Isaac Sim Fundamentals

### 1.1 Core Concepts

Isaac Sim uses a hierarchical structure based on USD (Universal Scene Description):

| Concept | Description |
|---------|-------------|
| **Stage** | The USD scene graph - logical/relational context for all prims |
| **Prim** | Any object in the scene (robots, lights, meshes, etc.) |
| **World** | Simulation context managing physics, rendering, and object handles |
| **Articulation** | A connected set of rigid bodies with joints (your robot) |
| **Scene** | Collection of objects managed by the World |

### 1.2 Key API Imports (Isaac Sim 5.1.0)

```python
# Core simulation
from isaacsim import SimulationApp
from isaacsim.core.api import World
from isaacsim.core.api.world import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.types import ArticulationAction

# Physics context
from isaacsim.core.api import SimulationContext

# Objects
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.objects.ground_plane import GroundPlane

# NumPy for data handling
import numpy as np
```

### 1.3 Basic Simulation Setup

```python
# STEP 1: Initialize the Application
simulation_app = SimulationApp({"headless": False})  # False = GUI enabled

# STEP 2: Create the World
world = World(stage_units_in_meters=1.0)

# STEP 3: Add Ground Plane
world.scene.add_default_ground_plane()

# STEP 4: Load Robot
robot_path = "/path/to/g1.usd"
prim_path = "/World/G1"
add_reference_to_stage(usd_path=robot_path, prim_path=prim_path)

# STEP 5: Create Articulation Handle
robot = Articulation(prim_paths_expr=prim_path, name="G1")

# STEP 6: Initialize World (CRITICAL - must be called before simulation)
world.reset()

# STEP 7: Main Loop
while simulation_app.is_running():
    world.step(render=True)
    
    # Read robot state
    joint_positions = robot.get_joint_positions()
    joint_velocities = robot.get_joint_velocities()
    
    # Control robot
    robot.set_joint_position_targets(target_positions)
    # OR
    robot.set_joint_velocity_targets(target_velocities)
    # OR
    robot.set_joint_effort_targets(target_torques)

simulation_app.close()
```

### 1.4 Physics Callbacks (For Real-Time Control)

```python
from isaacsim.core.api import SimulationContext

simulation_context = SimulationContext()

def control_callback(step_size):
    """Called every physics step"""
    positions = robot.get_joint_positions()
    # Your control logic here
    robot.set_joint_position_targets(new_targets)

# Register callback
simulation_context.add_physics_callback("robot_control", control_callback)

# Remove when done
simulation_context.remove_physics_callback("robot_control")
```

### 1.5 Joint Control Modes

| Mode | Method | Use Case |
|------|--------|----------|
| **Position** | `set_joint_position_targets()` | Precise positioning, uses internal PD controller |
| **Velocity** | `set_joint_velocity_targets()` | Continuous motion tasks |
| **Effort/Torque** | `set_joint_effort_targets()` | Direct torque control, requires stiffness=0, damping=0 |

### 1.6 PD Controller Equation

Isaac Sim uses implicit PD control:

```
τ = stiffness × (q_target - q) + damping × (q̇_target - q̇)
```

Where:
- `τ` = applied torque
- `q` = current joint position
- `q̇` = current joint velocity  
- `stiffness` = proportional gain (Kp)
- `damping` = derivative gain (Kd)

### 1.7 Gain Tuning Guidelines

| Scenario | Stiffness | Damping | Notes |
|----------|-----------|---------|-------|
| **Position Control** | High (1000+) | ~10% of stiffness | Start with damping = stiffness/10 |
| **Velocity Control** | 0 | Tune for convergence | Increase until joint reaches target velocity |
| **Effort Control** | 0 | 0 | Direct torque application |
| **Soft/Compliant** | Low (10-100) | Match stiffness | For safe interaction |

**Tuning Process:**
1. Start with stiffness = 0, tune damping until velocity converges
2. Add stiffness, keep damping ~10% of stiffness
3. Reduce damping for faster response (may overshoot)
4. For gravity compensation, disable gravity on rigid bodies in properties

---

## 2. G1 Robot Joint Analysis

### 2.1 Overview

Based on your CSV, the G1 has **61 joints** organized as follows:

| Body Part | Count | Joints |
|-----------|-------|--------|
| **Legs** | 12 (6 per leg) | Hip (pitch/roll/yaw), Knee, Ankle (pitch/roll) |
| **Torso** | 3 | Waist (yaw/roll/pitch) |
| **Arms** | 8 (4 per arm) | Shoulder (pitch/roll/yaw), Elbow |
| **Wrists** | 6 (3 per wrist) | Wrist (roll/pitch/yaw) |
| **Fingers** | 32 (16 per hand) | 5 fingers × multiple joints |

### 2.2 Complete Joint Map

#### Legs (Indices 0-18)

| Index | Joint Name | Side | Limits (rad) | Max Vel | Max Torque | Stiffness | Damping |
|-------|------------|------|--------------|---------|------------|-----------|---------|
| 0 | left_hip_pitch_joint | left | [-2.53, 2.88] | 20.0 | 139.0 | 1423.5 | 0.57 |
| 1 | right_hip_pitch_joint | right | [-2.53, 2.88] | 20.0 | 139.0 | 1423.5 | 0.57 |
| 3 | left_hip_roll_joint | left | [-0.52, 2.97] | 20.0 | 139.0 | 3560.4 | 1.42 |
| 4 | right_hip_roll_joint | right | [-2.97, 0.52] | 20.0 | 139.0 | 3560.4 | 1.42 |
| 6 | left_hip_yaw_joint | left | [-2.76, 2.76] | 32.0 | 88.0 | 8621.7 | 3.45 |
| 7 | right_hip_yaw_joint | right | [-2.76, 2.76] | 32.0 | 88.0 | 8621.7 | 3.45 |
| 9 | left_knee_joint | left | [-0.09, 2.88] | 20.0 | 139.0 | 13183.2 | 5.27 |
| 10 | right_knee_joint | right | [-0.09, 2.88] | 20.0 | 139.0 | 13182.7 | 5.27 |
| 13 | left_ankle_pitch_joint | left | [-0.87, 0.52] | 30.0 | 35.0 | 3720.6 | 1.49 |
| 14 | right_ankle_pitch_joint | right | [-0.87, 0.52] | 30.0 | 35.0 | 3720.6 | 1.49 |
| 17 | left_ankle_roll_joint | left | [-0.26, 0.26] | 30.0 | 35.0 | 131.9 | 0.05 |
| 18 | right_ankle_roll_joint | right | [-0.26, 0.26] | 30.0 | 35.0 | 131.9 | 0.05 |

#### Torso (Indices 2, 5, 8)

| Index | Joint Name | Limits (rad) | Max Vel | Max Torque | Stiffness | Damping |
|-------|------------|--------------|---------|------------|-----------|---------|
| 2 | waist_yaw_joint | [-2.62, 2.62] | 32.0 | 88.0 | 1589.9 | 0.64 |
| 5 | waist_roll_joint | [-0.52, 0.52] | 30.0 | 35.0 | 3099.7 | 1.24 |
| 8 | waist_pitch_joint | [-0.52, 0.52] | 30.0 | 35.0 | 3039.7 | 1.22 |

#### Arms (Indices 11-12, 15-16, 19-22)

| Index | Joint Name | Side | Limits (rad) | Max Vel | Max Torque | Stiffness | Damping |
|-------|------------|------|--------------|---------|------------|-----------|---------|
| 11 | left_shoulder_pitch_joint | left | [-3.09, 2.67] | 37.0 | 25.0 | 8584.7 | 3.43 |
| 12 | right_shoulder_pitch_joint | right | [-3.09, 2.67] | 37.0 | 25.0 | 8584.6 | 3.43 |
| 15 | left_shoulder_roll_joint | left | [-1.59, 2.25] | 37.0 | 25.0 | 7105.6 | 2.84 |
| 16 | right_shoulder_roll_joint | right | [-2.25, 1.59] | 37.0 | 25.0 | 7105.4 | 2.84 |
| 19 | left_shoulder_yaw_joint | left | [-2.62, 2.62] | 37.0 | 25.0 | 6562.3 | 2.62 |
| 20 | right_shoulder_yaw_joint | right | [-2.62, 2.62] | 37.0 | 25.0 | 6562.6 | 2.63 |
| 21 | left_elbow_joint | left | [-1.05, 2.09] | 37.0 | 25.0 | 3762.0 | 1.50 |
| 22 | right_elbow_joint | right | [-1.05, 2.09] | 37.0 | 25.0 | 3762.0 | 1.50 |

#### Wrists (Indices 23-28)

| Index | Joint Name | Side | Limits (rad) | Max Vel | Max Torque | Stiffness | Damping |
|-------|------------|------|--------------|---------|------------|-----------|---------|
| 23 | left_wrist_roll_joint | left | [-1.97, 1.97] | 37.0 | 25.0 | 2690.7 | 1.08 |
| 24 | right_wrist_roll_joint | right | [-1.97, 1.97] | 37.0 | 25.0 | 2690.9 | 1.08 |
| 25 | left_wrist_pitch_joint | left | [-1.61, 1.61] | 22.0 | 5.0 | 1002.5 | 0.40 |
| 26 | right_wrist_pitch_joint | right | [-1.61, 1.61] | 22.0 | 5.0 | 1002.8 | 0.40 |
| 27 | left_wrist_yaw_joint | left | [-1.61, 1.61] | 22.0 | 5.0 | 587.3 | 0.23 |
| 28 | right_wrist_yaw_joint | right | [-1.61, 1.61] | 22.0 | 5.0 | 587.6 | 0.24 |

#### Fingers (Indices 29-60)

**Important Notes:**
- Joints with `driveMode=0` are **passive/mimic joints** (they follow other joints)
- Joints with `hasLimits=False` and extreme velocities are **sensor/tip frames**
- Active finger joints have `driveMode=1`

| Type | Indices | Notes |
|------|---------|-------|
| **Proximal (Active)** | 29-34, 43, 48 | Primary control joints |
| **Distal (Passive)** | 39-42, 44-47, 53, 58 | driveMode=0, follow proximal |
| **Tip (Sensor)** | 49-52, 54-57, 59-60 | No limits, reference frames only |

### 2.3 Joint Groups for Control

```python
# Define joint groups for easier control
JOINT_GROUPS = {
    "left_leg": [0, 3, 6, 9, 13, 17],      # 6 joints
    "right_leg": [1, 4, 7, 10, 14, 18],    # 6 joints
    "torso": [2, 5, 8],                      # 3 joints
    "left_arm": [11, 15, 19, 21],           # 4 joints
    "right_arm": [12, 16, 20, 22],          # 4 joints
    "left_wrist": [23, 25, 27],             # 3 joints
    "right_wrist": [24, 26, 28],            # 3 joints
    "left_hand_active": [29, 30, 31, 32, 33, 43],  # Controllable finger joints
    "right_hand_active": [34, 35, 36, 37, 38, 48], # Controllable finger joints
}

# Joints that should NOT be directly controlled (passive/sensor)
PASSIVE_JOINTS = [39, 40, 41, 42, 44, 45, 46, 47, 53, 58]  # driveMode=0
SENSOR_JOINTS = [49, 50, 51, 52, 54, 55, 56, 57, 59, 60]   # No limits, tip frames
```

### 2.4 Critical Joint Observations

| Observation | Joints | Impact |
|-------------|--------|--------|
| **Highest Stiffness** | Knees (13183) | Very stiff, precise position control |
| **Lowest Stiffness** | Ankle roll (131.9) | Compliant, for balance |
| **Highest Torque** | Hips, Knees (139 Nm) | Load-bearing joints |
| **Lowest Torque** | Wrist pitch/yaw (5 Nm) | Delicate manipulation |
| **Mirror Limits** | Left/Right hip roll | Note asymmetric limits! |

---

## 3. Safety Considerations

### 3.1 Joint Limit Safety (CRITICAL)

**Always enforce software limits BEFORE hardware limits are reached:**

```python
class JointSafetyController:
    """Wrapper to enforce joint limits with safety margins"""
    
    def __init__(self, robot, safety_margin_rad=0.1):
        self.robot = robot
        self.safety_margin = safety_margin_rad
        
        # From your CSV - store as numpy arrays
        self.lower_limits = np.array([
            -2.5307, -2.5307, -2.6180, -0.5236, -2.9671, -0.5200,  # 0-5
            -2.7576, -2.7576, -0.5200, -0.0873, -0.0873, -3.0892,  # 6-11
            -3.0892, -0.8727, -0.8727, -1.5882, -2.2515, -0.2618,  # 12-17
            -0.2618, -2.6180, -2.6180, -1.0472, -1.0472, -1.9722,  # 18-23
            -1.9722, -1.6144, -1.6144, -1.6144, -1.6144,           # 24-28
            # ... add remaining limits
        ])
        
        self.upper_limits = np.array([
            2.8798, 2.8798, 2.6180, 2.9671, 0.5236, 0.5200,        # 0-5
            2.7576, 2.7576, 0.5200, 2.8798, 2.8798, 2.6704,        # 6-11
            2.6704, 0.5236, 0.5236, 2.2515, 1.5882, 0.2618,        # 12-17
            0.2618, 2.6180, 2.6180, 2.0944, 2.0944, 1.9722,        # 18-23
            1.9722, 1.6144, 1.6144, 1.6144, 1.6144,                # 24-28
            # ... add remaining limits
        ])
        
        # Apply safety margin
        self.safe_lower = self.lower_limits + self.safety_margin
        self.safe_upper = self.upper_limits - self.safety_margin
    
    def clamp_positions(self, target_positions):
        """Clamp target positions to safe range"""
        return np.clip(target_positions, self.safe_lower, self.safe_upper)
    
    def check_limits(self, positions):
        """Check if positions are within safe limits"""
        below_lower = positions < self.safe_lower
        above_upper = positions > self.safe_upper
        return not (np.any(below_lower) or np.any(above_upper))
```

### 3.2 Velocity Limits

```python
# Max velocities from CSV (rad/s)
MAX_VELOCITIES = {
    "leg_joints": 20.0,     # Hips, knees
    "torso_joints": 30.0,   # Waist
    "arm_joints": 37.0,     # Shoulders, elbows
    "wrist_roll": 37.0,
    "wrist_pitch_yaw": 22.0,
    "fingers": 2.27,        # Very slow for precision
}

# Apply velocity scaling for safety (start at 50%)
VELOCITY_SCALE = 0.5  # Production should ramp up gradually
```

### 3.3 Torque Limits

```python
# Max effort/torque from CSV (Nm)
MAX_TORQUES = {
    "hip_pitch": 139.0,
    "hip_roll": 139.0,
    "hip_yaw": 88.0,
    "knee": 139.0,
    "ankle_pitch": 35.0,
    "ankle_roll": 35.0,
    "waist_yaw": 88.0,
    "waist_roll_pitch": 35.0,
    "shoulder_all": 25.0,
    "elbow": 25.0,
    "wrist_roll": 25.0,
    "wrist_pitch_yaw": 5.0,  # Very low!
    "finger_proximal": 2.0,
    "finger_thumb": 0.5,     # Even lower
}
```

### 3.4 Self-Collision Prevention

Critical collision pairs to monitor:

| Pair | Risk | Mitigation |
|------|------|------------|
| Arms ↔ Torso | High | Limit shoulder roll range |
| Hands ↔ Legs | Medium | Monitor during sitting/bending |
| Knees ↔ Face | High | Limit knee flex when hip is flexed |
| Elbows ↔ Hips | Medium | Coordinate arm/torso motion |

```python
def check_self_collision_risk(joint_positions):
    """Heuristic checks for self-collision risk"""
    warnings = []
    
    # Check arm-torso collision risk
    left_shoulder_roll = joint_positions[15]   # idx 15
    right_shoulder_roll = joint_positions[16]  # idx 16
    
    if left_shoulder_roll > 2.0:  # Too far inward
        warnings.append("Left arm close to torso collision")
    if right_shoulder_roll < -2.0:
        warnings.append("Right arm close to torso collision")
    
    # Check knee-face collision (when hip flexed forward)
    left_hip_pitch = joint_positions[0]
    left_knee = joint_positions[9]
    
    if left_hip_pitch > 1.5 and left_knee > 2.5:
        warnings.append("Left leg may collide with upper body")
    
    return warnings
```

### 3.5 Emergency Stop Implementation

```python
class EmergencyStop:
    """Emergency stop controller with graceful shutdown"""
    
    def __init__(self, robot, world):
        self.robot = robot
        self.world = world
        self.estop_active = False
        
    def trigger_estop(self, reason="Unknown"):
        """Trigger emergency stop"""
        print(f"⚠️ E-STOP TRIGGERED: {reason}")
        self.estop_active = True
        
        # Set all joints to hold current position
        current_pos = self.robot.get_joint_positions()
        self.robot.set_joint_position_targets(current_pos)
        
        # Optionally pause simulation
        # self.world.pause()
        
    def reset_estop(self):
        """Reset emergency stop (requires explicit call)"""
        if self.estop_active:
            print("E-STOP RESET - Resuming normal operation")
            self.estop_active = False
            
    def is_active(self):
        return self.estop_active
```

### 3.6 Pre-Production Checklist

Before deploying to real hardware:

- [ ] **Verify joint limits** match physical robot spec sheet
- [ ] **Test all joints** individually at 10% speed
- [ ] **Verify sensor readings** match expected values
- [ ] **Check emergency stop** triggers correctly
- [ ] **Test self-collision** detection at boundaries
- [ ] **Verify torque limits** don't exceed motor specs
- [ ] **Run sim-to-sim** transfer test (PhysX ↔ Newton)
- [ ] **Check observation space** - only use real-sensor-available data
- [ ] **Test graceful degradation** when sensors fail

---

## 4. Production-Ready Code Guide

### 4.1 Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER                        │
│  (Task definitions, behaviors, state machines)              │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    CONTROL LAYER                            │
│  (Motion planning, trajectory generation, IK/FK)            │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    SAFETY LAYER                             │
│  (Limit checking, collision avoidance, E-stop)              │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    HARDWARE ABSTRACTION                     │
│  (Joint commands, sensor reading, robot interface)          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    ISAAC SIM / REAL ROBOT                   │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 Production Code Template

```python
"""
G1 Robot Controller - Production Template
==========================================
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, List, Dict
from enum import Enum, auto
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RobotState(Enum):
    """Robot operational states"""
    UNINITIALIZED = auto()
    INITIALIZING = auto()
    READY = auto()
    EXECUTING = auto()
    ERROR = auto()
    ESTOP = auto()


@dataclass
class JointLimits:
    """Joint limit configuration"""
    lower: np.ndarray
    upper: np.ndarray
    max_velocity: np.ndarray
    max_torque: np.ndarray
    
    def apply_safety_margin(self, margin_rad: float = 0.1) -> 'JointLimits':
        """Return new limits with safety margins applied"""
        return JointLimits(
            lower=self.lower + margin_rad,
            upper=self.upper - margin_rad,
            max_velocity=self.max_velocity * 0.9,  # 10% velocity margin
            max_torque=self.max_torque * 0.9       # 10% torque margin
        )


@dataclass
class RobotCommand:
    """Command structure for robot control"""
    joint_positions: Optional[np.ndarray] = None
    joint_velocities: Optional[np.ndarray] = None
    joint_torques: Optional[np.ndarray] = None
    timestamp: float = 0.0


@dataclass
class RobotObservation:
    """Observation structure from robot"""
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    joint_torques: np.ndarray
    imu_angular_velocity: np.ndarray  # Available on real robot
    imu_linear_acceleration: np.ndarray  # Available on real robot
    timestamp: float


class G1Controller:
    """
    Production-ready controller for Unitree G1 robot.
    
    Designed for easy sim-to-real transfer:
    - Uses only sensor-available observations
    - Implements all safety layers
    - Modular architecture for testing
    """
    
    def __init__(self, config: Dict):
        self.config = config
        self.state = RobotState.UNINITIALIZED
        self.robot = None
        self.world = None
        
        # Load joint limits (from CSV)
        self.limits = self._load_joint_limits()
        self.safe_limits = self.limits.apply_safety_margin()
        
        # State tracking
        self._last_command = None
        self._last_observation = None
        self._error_count = 0
        self._max_errors = 10
        
    def _load_joint_limits(self) -> JointLimits:
        """Load joint limits from configuration"""
        # In production: load from CSV or config file
        # These are extracted from your CSV
        return JointLimits(
            lower=np.array([...]),  # Fill from CSV
            upper=np.array([...]),
            max_velocity=np.array([...]),
            max_torque=np.array([...])
        )
    
    def initialize(self, simulation_app, robot_prim_path: str) -> bool:
        """
        Initialize the robot controller.
        
        Args:
            simulation_app: Isaac Sim application instance
            robot_prim_path: Path to robot in USD stage
            
        Returns:
            True if initialization successful
        """
        try:
            self.state = RobotState.INITIALIZING
            logger.info("Initializing G1 Controller...")
            
            # Import here to allow testing without Isaac Sim
            from isaacsim.core.api import World
            from isaacsim.core.prims import Articulation
            
            # Create world
            self.world = World(stage_units_in_meters=1.0)
            self.world.scene.add_default_ground_plane()
            
            # Initialize robot
            self.robot = Articulation(
                prim_paths_expr=robot_prim_path,
                name="G1"
            )
            
            # Reset world to initialize physics
            self.world.reset()
            
            # Verify joint count
            num_joints = self.robot.num_dof
            if num_joints != 61:
                logger.warning(f"Expected 61 joints, got {num_joints}")
            
            self.state = RobotState.READY
            logger.info("G1 Controller initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Initialization failed: {e}")
            self.state = RobotState.ERROR
            return False
    
    def get_observation(self) -> RobotObservation:
        """
        Get current robot state.
        
        NOTE: Only returns sensor-available data for sim-to-real compatibility.
        Base linear velocity is NOT included (not directly measurable on real robot).
        """
        return RobotObservation(
            joint_positions=self.robot.get_joint_positions().copy(),
            joint_velocities=self.robot.get_joint_velocities().copy(),
            joint_torques=self.robot.get_measured_joint_efforts().copy(),
            imu_angular_velocity=self._get_imu_angular_velocity(),
            imu_linear_acceleration=self._get_imu_linear_acceleration(),
            timestamp=self.world.current_time
        )
    
    def _get_imu_angular_velocity(self) -> np.ndarray:
        """Get angular velocity from IMU (available on real robot)"""
        # In simulation, get from root body
        # On real robot: read from IMU sensor
        return self.robot.get_angular_velocities()[0]
    
    def _get_imu_linear_acceleration(self) -> np.ndarray:
        """Get linear acceleration from IMU"""
        # Placeholder - implement based on your IMU interface
        return np.zeros(3)
    
    def send_command(self, command: RobotCommand) -> bool:
        """
        Send command to robot with safety checks.
        
        Args:
            command: Target positions/velocities/torques
            
        Returns:
            True if command was applied successfully
        """
        if self.state != RobotState.READY and self.state != RobotState.EXECUTING:
            logger.warning(f"Cannot send command in state: {self.state}")
            return False
        
        try:
            # SAFETY LAYER 1: Validate command
            if not self._validate_command(command):
                return False
            
            # SAFETY LAYER 2: Apply limits
            safe_command = self._apply_limits(command)
            
            # SAFETY LAYER 3: Check for self-collision risk
            if self._check_collision_risk(safe_command):
                logger.warning("Collision risk detected, command rejected")
                return False
            
            # Apply command to robot
            if safe_command.joint_positions is not None:
                self.robot.set_joint_position_targets(safe_command.joint_positions)
            if safe_command.joint_velocities is not None:
                self.robot.set_joint_velocity_targets(safe_command.joint_velocities)
            if safe_command.joint_torques is not None:
                self.robot.set_joint_effort_targets(safe_command.joint_torques)
            
            self._last_command = safe_command
            self.state = RobotState.EXECUTING
            return True
            
        except Exception as e:
            logger.error(f"Command execution failed: {e}")
            self._handle_error()
            return False
    
    def _validate_command(self, command: RobotCommand) -> bool:
        """Validate command structure and values"""
        if command.joint_positions is not None:
            if len(command.joint_positions) != 61:
                logger.error("Invalid position command length")
                return False
            if np.any(np.isnan(command.joint_positions)):
                logger.error("NaN in position command")
                return False
        return True
    
    def _apply_limits(self, command: RobotCommand) -> RobotCommand:
        """Apply safety limits to command"""
        result = RobotCommand(timestamp=command.timestamp)
        
        if command.joint_positions is not None:
            result.joint_positions = np.clip(
                command.joint_positions,
                self.safe_limits.lower,
                self.safe_limits.upper
            )
        
        if command.joint_velocities is not None:
            result.joint_velocities = np.clip(
                command.joint_velocities,
                -self.safe_limits.max_velocity,
                self.safe_limits.max_velocity
            )
        
        if command.joint_torques is not None:
            result.joint_torques = np.clip(
                command.joint_torques,
                -self.safe_limits.max_torque,
                self.safe_limits.max_torque
            )
        
        return result
    
    def _check_collision_risk(self, command: RobotCommand) -> bool:
        """Check for self-collision risk"""
        # Implement collision checking based on your robot's geometry
        # Return True if collision risk detected
        return False
    
    def _handle_error(self):
        """Handle errors with graceful degradation"""
        self._error_count += 1
        if self._error_count >= self._max_errors:
            self.emergency_stop("Too many consecutive errors")
    
    def emergency_stop(self, reason: str = "Unknown"):
        """Trigger emergency stop"""
        logger.critical(f"EMERGENCY STOP: {reason}")
        self.state = RobotState.ESTOP
        
        # Hold current position
        if self.robot is not None:
            current_pos = self.robot.get_joint_positions()
            self.robot.set_joint_position_targets(current_pos)
    
    def reset_estop(self) -> bool:
        """Reset emergency stop state"""
        if self.state == RobotState.ESTOP:
            self.state = RobotState.READY
            self._error_count = 0
            logger.info("Emergency stop reset")
            return True
        return False
    
    def step(self) -> bool:
        """
        Execute one simulation/control step.
        
        Returns:
            True if step executed successfully
        """
        if self.state == RobotState.ESTOP:
            return False
        
        try:
            self.world.step(render=True)
            self._last_observation = self.get_observation()
            return True
        except Exception as e:
            logger.error(f"Step failed: {e}")
            self._handle_error()
            return False
    
    def shutdown(self):
        """Clean shutdown"""
        logger.info("Shutting down G1 Controller...")
        self.state = RobotState.UNINITIALIZED
        if self.world is not None:
            self.world.clear()
```

### 4.3 Step-by-Step Production Integration

**Phase 1: Simulation Validation**
1. Load robot USD in Isaac Sim
2. Verify all 61 joints detected and named correctly
3. Test each joint group independently at 10% speed
4. Verify sensor readings (positions, velocities, torques)
5. Run automated limit testing

**Phase 2: Control Development**
1. Implement basic position control for each joint group
2. Add velocity and torque control modes
3. Implement safety layers (limits, collision detection)
4. Add emergency stop functionality
5. Test failure modes and recovery

**Phase 3: Sim-to-Sim Transfer**
1. Train/test policy in PhysX backend
2. Export policy to ONNX format
3. Test in Newton backend
4. Verify identical behavior
5. Document any discrepancies

**Phase 4: Sim-to-Real Preparation**
1. Remove privileged observations (base linear velocity)
2. Add sensor noise models to simulation
3. Implement domain randomization
4. Use teacher-student distillation if needed
5. Verify control frequency matches real robot

**Phase 5: Real Robot Deployment**
1. Start with robot suspended/supported
2. Test individual joints at minimum speed
3. Gradually increase to full operation
4. Monitor torque/current for anomalies
5. Always have E-stop ready

### 4.4 Key Differences: Simulation vs Real Robot

| Aspect | Simulation | Real Robot |
|--------|------------|------------|
| **Base Linear Velocity** | Available | NOT directly measurable |
| **Contact Forces** | Perfect | Noisy/estimated |
| **Joint Positions** | Exact | Encoder noise |
| **Control Frequency** | Variable | Fixed (typically 200-1000 Hz) |
| **Latency** | Minimal | Network/processing delays |
| **Gravity** | Can disable | Always present |

### 4.5 Domain Randomization for Sim-to-Real

```python
# Add these randomizations during training
DOMAIN_RANDOMIZATION = {
    "mass_scale": (0.8, 1.2),           # ±20% mass variation
    "friction": (0.5, 1.5),             # Ground friction
    "motor_strength": (0.9, 1.1),       # ±10% motor variation
    "joint_damping": (0.8, 1.2),        # Damping variation
    "sensor_noise_pos": 0.01,           # Position noise (rad)
    "sensor_noise_vel": 0.1,            # Velocity noise (rad/s)
    "action_delay": (0, 2),             # Control delay (timesteps)
    "gravity_variation": (9.6, 10.0),   # Gravity (m/s²)
}
```

---

## Quick Reference Card

### Essential Commands

```python
# Get joint state
positions = robot.get_joint_positions()
velocities = robot.get_joint_velocities()
torques = robot.get_measured_joint_efforts()

# Set targets
robot.set_joint_position_targets(target_pos)
robot.set_joint_velocity_targets(target_vel)
robot.set_joint_effort_targets(target_torque)

# Step simulation
world.step(render=True)

# Reset to initial state
world.reset()
```

### Critical Safety Values

| Parameter | Value | Note |
|-----------|-------|------|
| Max hip/knee torque | 139 Nm | Highest on robot |
| Max wrist torque | 5 Nm | Very delicate! |
| Safety position margin | 0.1 rad | ~5.7° from limits |
| Velocity scale (start) | 50% | Ramp up gradually |
| Max continuous errors | 10 | Then E-stop |

### Contact & Resources

- **Isaac Sim Docs**: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/
- **Isaac Lab**: https://isaac-sim.github.io/IsaacLab/
- **Unitree G1 Specs**: https://www.unitree.com/g1/

---

*Document generated for Unitree G1 (61-DOF model) with Isaac Sim 5.1.0*