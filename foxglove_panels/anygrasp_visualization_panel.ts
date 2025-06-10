// AnyGrasp Interactive Visualization Panel for Foxglove Studio
// Custom panel for visualizing, selecting, and executing grasp poses with user confirmation

import { PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import { useLayoutEffect, useEffect, useState, useCallback } from "react";
import ReactDOM from "react-dom";

type PoseArray = {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  poses: Array<{
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  }>;
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

type GraspCandidate = {
  pose: PoseStamped;
  confidence: number;
  width: number;
  quality: number;
  originalIndex: number;
};

type WorkflowState = "idle" | "detecting" | "selecting" | "confirming" | "executing" | "completed" | "failed";

function AnyGraspVisualizationPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  // State management
  const [workflowState, setWorkflowState] = useState<WorkflowState>("idle");
  const [graspCandidates, setGraspCandidates] = useState<GraspCandidate[]>([]);
  const [selectedGraspIndex, setSelectedGraspIndex] = useState<number>(-1);
  const [statusMessage, setStatusMessage] = useState<string>("Ready to start grasp workflow");
  const [detectionInProgress, setDetectionInProgress] = useState<boolean>(false);
  const [confirmationDialog, setConfirmationDialog] = useState<boolean>(false);
  
  // Settings
  const [workflowSettings, setWorkflowSettings] = useState({
    numCandidates: 5,
    minConfidence: 0.6,
    autoExecute: false,
    showConfirmation: true
  });

  useLayoutEffect(() => {
    context.onRender = (renderState: RenderState, done) => {
      const messages = renderState.currentFrame ?? [];
      
      for (const message of messages) {
        if (message.topic === "/anygrasp/grasp_poses") {
          setGraspPoses(message.message as PoseArray);
          setDetectionStatus("Grasps detected");
        }
        if (message.topic === "/anygrasp/grasp_metrics") {
          setGraspMetrics(message.message as GraspMetrics[]);
        }
        if (message.topic === "/anygrasp/status") {
          setDetectionStatus((message.message as any).data || "Processing...");
        }
      }
      
      done();
    };
    context.watch("currentFrame");
  }, [context]);

  // Subscribe to AnyGrasp topics
  useEffect(() => {
    context.subscribe([
      { topic: "/anygrasp/grasp_poses" },
      { topic: "/anygrasp/grasp_metrics" },
      { topic: "/anygrasp/status" },
      { topic: "/anygrasp/debug_markers" },
    ]);
  }, [context]);

  // Execute selected grasp
  const executeGrasp = useCallback((graspIndex: number) => {
    if (!graspPoses || graspIndex < 0 || graspIndex >= graspPoses.poses.length) {
      return;
    }

    const selectedPose = graspPoses.poses[graspIndex];
    
    // Convert pose to move command
    const moveCommand = {
      x: selectedPose.position.x,
      y: selectedPose.position.y,
      z: selectedPose.position.z + 0.1, // Approach from above
    };

    // Send move command
    context.callService?.("/move_point_cmd", moveCommand);
    
    // Plan grasp execution sequence
    setTimeout(() => {
      // Move to actual grasp position
      context.callService?.("/move_point_cmd", {
        x: selectedPose.position.x,
        y: selectedPose.position.y,
        z: selectedPose.position.z,
      });
    }, 2000);

    setTimeout(() => {
      // Close gripper
      context.publish?.("/gripper_cmd", { data: 1.2 });
    }, 4000);

    setTimeout(() => {
      // Lift object
      context.callService?.("/move_point_cmd", {
        x: selectedPose.position.x,
        y: selectedPose.position.y,
        z: selectedPose.position.z + 0.15,
      });
    }, 5000);

    setSelectedGraspIndex(graspIndex);
  }, [context, graspPoses]);

  // Trigger new grasp detection
  const triggerDetection = useCallback(() => {
    setDetectionStatus("Detecting grasps...");
    context.callService?.("/anygrasp/detect_grasps", {});
  }, [context]);

  // Clear all grasps
  const clearGrasps = useCallback(() => {
    setGraspPoses(null);
    setSelectedGraspIndex(-1);
    setDetectionStatus("Cleared");
    
    // Publish empty marker array to clear visualization
    context.publish?.("/anygrasp/debug_markers", {
      markers: []
    });
  }, [context]);

  // Filter grasps based on settings
  const filteredGrasps = graspPoses ? graspPoses.poses
    .map((pose, index) => ({ pose, index, metrics: graspMetrics[index] }))
    .filter(item => !item.metrics || item.metrics.confidence >= filterSettings.minConfidence)
    .sort((a, b) => {
      if (!filterSettings.sortByConfidence) return 0;
      const confA = a.metrics?.confidence ?? 0;
      const confB = b.metrics?.confidence ?? 0;
      return confB - confA;
    })
    .slice(0, filterSettings.maxGrasps) : [];

  return (
    <div style={{ padding: "1rem", height: "100%", overflow: "auto" }}>
      <h3>🤖 AnyGrasp Visualization</h3>
      
      {/* Status */}
      <div style={{ marginBottom: "1rem", padding: "0.5rem", border: "1px solid #ccc", borderRadius: "4px" }}>
        <strong>Status:</strong> <span style={{ color: detectionStatus.includes("detected") ? "green" : "orange" }}>
          {detectionStatus}
        </span>
        <div style={{ fontSize: "0.8rem", marginTop: "0.25rem" }}>
          {graspPoses ? `${graspPoses.poses.length} total grasps, ${filteredGrasps.length} filtered` : "No grasps"}
        </div>
      </div>

      {/* Control Buttons */}
      <div style={{ marginBottom: "1rem", display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0.5rem" }}>
        <button onClick={triggerDetection} style={buttonStyle}>
          🔍 Detect Grasps
        </button>
        <button onClick={clearGrasps} style={buttonStyle}>
          🗑️ Clear Grasps
        </button>
      </div>

      {/* Filter Settings */}
      <div style={{ marginBottom: "1rem", padding: "0.5rem", border: "1px solid #ccc", borderRadius: "4px" }}>
        <h4>⚙️ Filter Settings</h4>
        
        <div style={{ marginBottom: "0.5rem" }}>
          <label>Min Confidence: </label>
          <input
            type="range"
            min="0"
            max="1"
            step="0.1"
            value={filterSettings.minConfidence}
            onChange={(e) => setFilterSettings(prev => ({ 
              ...prev, 
              minConfidence: parseFloat(e.target.value) 
            }))}
            style={{ width: "70%" }}
          />
          <span>{filterSettings.minConfidence.toFixed(1)}</span>
        </div>
        
        <div style={{ marginBottom: "0.5rem" }}>
          <label>Max Grasps: </label>
          <input
            type="range"
            min="1"
            max="20"
            step="1"
            value={filterSettings.maxGrasps}
            onChange={(e) => setFilterSettings(prev => ({ 
              ...prev, 
              maxGrasps: parseInt(e.target.value) 
            }))}
            style={{ width: "70%" }}
          />
          <span>{filterSettings.maxGrasps}</span>
        </div>
        
        <div>
          <label>
            <input
              type="checkbox"
              checked={filterSettings.sortByConfidence}
              onChange={(e) => setFilterSettings(prev => ({ 
                ...prev, 
                sortByConfidence: e.target.checked 
              }))}
            />
            Sort by confidence
          </label>
        </div>
      </div>

      {/* Grasp List */}
      <div style={{ marginBottom: "1rem" }}>
        <h4>🎯 Available Grasps</h4>
        <div style={{ maxHeight: "300px", overflowY: "auto", border: "1px solid #ccc", borderRadius: "4px" }}>
          {filteredGrasps.length === 0 ? (
            <div style={{ padding: "1rem", textAlign: "center", color: "#666" }}>
              No grasps available. Click "Detect Grasps" to start.
            </div>
          ) : (
            filteredGrasps.map((item, displayIndex) => (
              <div
                key={item.index}
                style={{
                  padding: "0.5rem",
                  borderBottom: "1px solid #eee",
                  backgroundColor: selectedGraspIndex === item.index ? "#e6f3ff" : "transparent",
                  cursor: "pointer"
                }}
                onClick={() => setSelectedGraspIndex(item.index)}
              >
                <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                  <div>
                    <strong>Grasp #{displayIndex + 1}</strong>
                    {item.metrics && (
                      <div style={{ fontSize: "0.8rem", color: "#666" }}>
                        Confidence: {(item.metrics.confidence * 100).toFixed(1)}%
                      </div>
                    )}
                  </div>
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      executeGrasp(item.index);
                    }}
                    style={{ ...buttonStyle, fontSize: "0.7rem", padding: "0.25rem 0.5rem" }}
                  >
                    ▶️ Execute
                  </button>
                </div>
                
                <div style={{ fontSize: "0.7rem", marginTop: "0.25rem" }}>
                  Position: ({item.pose.position.x.toFixed(3)}, {item.pose.position.y.toFixed(3)}, {item.pose.position.z.toFixed(3)})
                </div>
                
                {item.metrics && (
                  <div style={{ fontSize: "0.7rem", color: "#666" }}>
                    Width: {item.metrics.width?.toFixed(3)}m | 
                    Quality: {(item.metrics.quality_score * 100).toFixed(1)}%
                  </div>
                )}
              </div>
            ))
          )}
        </div>
      </div>

      {/* Selected Grasp Details */}
      {selectedGraspIndex >= 0 && graspPoses && (
        <div style={{ padding: "0.5rem", border: "2px solid #007acc", borderRadius: "4px" }}>
          <h4>📌 Selected Grasp</h4>
          <div style={{ fontSize: "0.8rem" }}>
            <div><strong>Index:</strong> {selectedGraspIndex}</div>
            <div><strong>Position:</strong> 
              ({graspPoses.poses[selectedGraspIndex].position.x.toFixed(3)}, 
               {graspPoses.poses[selectedGraspIndex].position.y.toFixed(3)}, 
               {graspPoses.poses[selectedGraspIndex].position.z.toFixed(3)})
            </div>
            {graspMetrics[selectedGraspIndex] && (
              <>
                <div><strong>Confidence:</strong> {(graspMetrics[selectedGraspIndex].confidence * 100).toFixed(1)}%</div>
                <div><strong>Gripper Width:</strong> {graspMetrics[selectedGraspIndex].width?.toFixed(3)}m</div>
              </>
            )}
          </div>
          
          <button
            onClick={() => executeGrasp(selectedGraspIndex)}
            style={{ ...buttonStyle, marginTop: "0.5rem", width: "100%", backgroundColor: "#007acc", color: "white" }}
          >
            🚀 Execute Selected Grasp
          </button>
        </div>
      )}

      {/* Instructions */}
      <div style={{ fontSize: "0.8rem", color: "#666", marginTop: "1rem" }}>
        <strong>Instructions:</strong>
        <ul style={{ margin: "0.5rem 0", paddingLeft: "1rem" }}>
          <li>Click "Detect Grasps" to analyze current scene</li>
          <li>Adjust filters to refine grasp selection</li>
          <li>Click on grasps to select, then execute</li>
          <li>Monitor 3D view for grasp visualization</li>
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

export function initAnyGraspVisualizationPanel(context: PanelExtensionContext): void {
  ReactDOM.render(<AnyGraspVisualizationPanel context={context} />, context.panelElement);
}

// Export for Foxglove Studio
if (typeof window !== "undefined") {
  (window as any).registerFoxgloveAnyGraspPanel = (context: PanelExtensionContext) => {
    initAnyGraspVisualizationPanel(context);
  };
}