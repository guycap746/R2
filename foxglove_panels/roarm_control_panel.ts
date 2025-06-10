// RoArm M3 Control Panel for Foxglove Studio
// Custom panel for intuitive robot arm control

import { PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import { useLayoutEffect, useEffect, useState, useCallback } from "react";
import ReactDOM from "react-dom";

type JointState = {
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
};

type PoseStamped = {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
};

function RoArmControlPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [topics, setTopics] = useState<readonly Topic[]>([]);
  const [messages, setMessages] = useState<readonly MessageEvent<unknown>[]>([]);
  const [currentJointStates, setCurrentJointStates] = useState<JointState | null>(null);
  const [targetPose, setTargetPose] = useState({ x: 0.2, y: 0.0, z: 0.2 });
  const [isConnected, setIsConnected] = useState(false);

  useLayoutEffect(() => {
    context.onRender = (renderState: RenderState, done) => {
      setTopics(renderState.topics ?? []);
      setMessages(renderState.currentFrame ?? []);
      done();
    };
    context.watch("topics");
    context.watch("currentFrame");
  }, [context]);

  // Subscribe to relevant topics
  useEffect(() => {
    context.subscribe([
      { topic: "/joint_states" },
      { topic: "/roarm_driver/status" },
      { topic: "/move_group/display_planned_path" },
    ]);
  }, [context]);

  // Process incoming messages
  useEffect(() => {
    for (const message of messages) {
      if (message.topic === "/joint_states") {
        setCurrentJointStates(message.message as JointState);
      }
      if (message.topic === "/roarm_driver/status") {
        setIsConnected(true);
      }
    }
  }, [messages]);

  // Joint control functions
  const moveToHomePosition = useCallback(() => {
    const homeMessage = {
      x: 0.2,
      y: 0.0,
      z: 0.2
    };
    
    context.callService?.("/move_point_cmd", homeMessage);
  }, [context]);

  const moveToPosition = useCallback(() => {
    context.callService?.("/move_point_cmd", targetPose);
  }, [context, targetPose]);

  const emergencyStop = useCallback(() => {
    // Publish stop command
    context.publish?.("/servo_node/delta_twist_cmds", {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "base_link" },
      twist: {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      }
    });
  }, [context]);

  const openGripper = useCallback(() => {
    context.publish?.("/gripper_cmd", { data: 0.0 });
  }, [context]);

  const closeGripper = useCallback(() => {
    context.publish?.("/gripper_cmd", { data: 1.57 }); // Max gripper closure
  }, [context]);

  const controlLED = useCallback((brightness: number) => {
    context.publish?.("/led_ctrl", { data: brightness });
  }, [context]);

  return (
    <div style={{ padding: "1rem", height: "100%", overflow: "auto" }}>
      <h3>🦾 RoArm M3 Control Panel</h3>
      
      {/* Connection Status */}
      <div style={{ marginBottom: "1rem" }}>
        <span style={{ 
          color: isConnected ? "green" : "red",
          fontWeight: "bold"
        }}>
          ● {isConnected ? "Connected" : "Disconnected"}
        </span>
      </div>

      {/* Current Joint States */}
      <div style={{ marginBottom: "1rem", padding: "0.5rem", border: "1px solid #ccc", borderRadius: "4px" }}>
        <h4>📊 Current Joint States</h4>
        {currentJointStates ? (
          <div style={{ fontSize: "0.8rem" }}>
            {currentJointStates.name.map((name, index) => (
              <div key={name}>
                <strong>{name}:</strong> {currentJointStates.position[index]?.toFixed(3) ?? "N/A"} rad
              </div>
            ))}
          </div>
        ) : (
          <div>No joint data available</div>
        )}
      </div>

      {/* Quick Actions */}
      <div style={{ marginBottom: "1rem" }}>
        <h4>⚡ Quick Actions</h4>
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0.5rem" }}>
          <button onClick={moveToHomePosition} style={buttonStyle}>
            🏠 Home Position
          </button>
          <button onClick={emergencyStop} style={{ ...buttonStyle, backgroundColor: "#ff4444" }}>
            🛑 Emergency Stop
          </button>
          <button onClick={openGripper} style={buttonStyle}>
            ✋ Open Gripper
          </button>
          <button onClick={closeGripper} style={buttonStyle}>
            👊 Close Gripper
          </button>
        </div>
      </div>

      {/* Position Control */}
      <div style={{ marginBottom: "1rem", padding: "0.5rem", border: "1px solid #ccc", borderRadius: "4px" }}>
        <h4>🎯 Position Control</h4>
        <div style={{ marginBottom: "0.5rem" }}>
          <label>X (forward/back): </label>
          <input
            type="range"
            min="-0.5"
            max="0.5"
            step="0.01"
            value={targetPose.x}
            onChange={(e) => setTargetPose(prev => ({ ...prev, x: parseFloat(e.target.value) }))}
            style={{ width: "100%" }}
          />
          <span>{targetPose.x.toFixed(2)}m</span>
        </div>
        
        <div style={{ marginBottom: "0.5rem" }}>
          <label>Y (left/right): </label>
          <input
            type="range"
            min="-0.3"
            max="0.3"
            step="0.01"
            value={targetPose.y}
            onChange={(e) => setTargetPose(prev => ({ ...prev, y: parseFloat(e.target.value) }))}
            style={{ width: "100%" }}
          />
          <span>{targetPose.y.toFixed(2)}m</span>
        </div>
        
        <div style={{ marginBottom: "0.5rem" }}>
          <label>Z (up/down): </label>
          <input
            type="range"
            min="0.05"
            max="0.4"
            step="0.01"
            value={targetPose.z}
            onChange={(e) => setTargetPose(prev => ({ ...prev, z: parseFloat(e.target.value) }))}
            style={{ width: "100%" }}
          />
          <span>{targetPose.z.toFixed(2)}m</span>
        </div>
        
        <button onClick={moveToPosition} style={{ ...buttonStyle, width: "100%" }}>
          🎯 Move to Position
        </button>
      </div>

      {/* LED Control */}
      <div style={{ marginBottom: "1rem", padding: "0.5rem", border: "1px solid #ccc", borderRadius: "4px" }}>
        <h4>💡 LED Control</h4>
        <input
          type="range"
          min="0"
          max="255"
          step="1"
          onChange={(e) => controlLED(parseFloat(e.target.value))}
          style={{ width: "100%" }}
        />
        <div style={{ textAlign: "center", fontSize: "0.8rem" }}>Brightness: 0-255</div>
      </div>

      {/* Preset Positions */}
      <div style={{ marginBottom: "1rem" }}>
        <h4>📍 Preset Positions</h4>
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0.5rem" }}>
          <button 
            onClick={() => {
              setTargetPose({ x: 0.2, y: 0.0, z: 0.3 });
              setTimeout(moveToPosition, 100);
            }}
            style={buttonStyle}
          >
            📦 Pick Position
          </button>
          <button 
            onClick={() => {
              setTargetPose({ x: -0.1, y: 0.2, z: 0.25 });
              setTimeout(moveToPosition, 100);
            }}
            style={buttonStyle}
          >
            📤 Place Position
          </button>
          <button 
            onClick={() => {
              setTargetPose({ x: 0.3, y: 0.0, z: 0.1 });
              setTimeout(moveToPosition, 100);
            }}
            style={buttonStyle}
          >
            👀 Observe Position
          </button>
          <button 
            onClick={() => {
              setTargetPose({ x: 0.0, y: 0.0, z: 0.4 });
              setTimeout(moveToPosition, 100);
            }}
            style={buttonStyle}
          >
            ⬆️ Safe Position
          </button>
        </div>
      </div>

      {/* Instructions */}
      <div style={{ fontSize: "0.8rem", color: "#666", marginTop: "1rem" }}>
        <strong>Instructions:</strong>
        <ul style={{ margin: "0.5rem 0", paddingLeft: "1rem" }}>
          <li>Use sliders to set target position</li>
          <li>Click preset buttons for common positions</li>
          <li>Monitor joint states for current position</li>
          <li>Use emergency stop in case of issues</li>
        </ul>
      </div>
    </div>
  );
}

const buttonStyle = {
  padding: "0.5rem",
  border: "1px solid #ccc",
  borderRadius: "4px",
  backgroundColor: "#f5f5f5",
  cursor: "pointer",
  fontSize: "0.8rem"
};

export function initRoArmControlPanel(context: PanelExtensionContext): void {
  ReactDOM.render(<RoArmControlPanel context={context} />, context.panelElement);
}

// Export for Foxglove Studio
if (typeof window !== "undefined") {
  (window as any).registerFoxglovePanel = (context: PanelExtensionContext) => {
    initRoArmControlPanel(context);
  };
}