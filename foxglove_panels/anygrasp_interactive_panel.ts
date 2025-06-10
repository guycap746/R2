// AnyGrasp Interactive Workflow Panel for Foxglove Studio
// Complete user-guided grasp selection and execution interface

import { PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import { useLayoutEffect, useEffect, useState, useCallback } from "react";
import ReactDOM from "react-dom";

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

function AnyGraspInteractivePanel({ context }: { context: PanelExtensionContext }): JSX.Element {
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
        // Monitor workflow status
        if (message.topic === "/grasp_coordinator/status") {
          const statusData = (message.message as any).data;
          setStatusMessage(statusData);
          
          // Update workflow state based on status
          if (statusData.includes("[DETECTING]")) {
            setWorkflowState("detecting");
          } else if (statusData.includes("[WAITING_FOR_USER]")) {
            setWorkflowState("selecting");
          } else if (statusData.includes("[EXECUTING]")) {
            setWorkflowState("executing");
          } else if (statusData.includes("[COMPLETED]")) {
            setWorkflowState("completed");
          } else if (statusData.includes("[FAILED]")) {
            setWorkflowState("failed");
          } else if (statusData.includes("Ready")) {
            setWorkflowState("idle");
          }
        }
        
        // Monitor AnyGrasp status
        if (message.topic === "/anygrasp/status") {
          const anygraspStatus = (message.message as any).data;
          if (anygraspStatus.includes("Detecting")) {
            setDetectionInProgress(true);
          } else {
            setDetectionInProgress(false);
          }
        }
      }
      
      done();
    };
    context.watch("currentFrame");
  }, [context]);

  // Subscribe to relevant topics
  useEffect(() => {
    context.subscribe([
      { topic: "/grasp_coordinator/status" },
      { topic: "/anygrasp/status" },
      { topic: "/anygrasp/top_candidates" },
    ]);
  }, [context]);

  // Start grasp detection workflow
  const startGraspWorkflow = useCallback(async () => {
    setWorkflowState("detecting");
    setDetectionInProgress(true);
    setStatusMessage("Starting grasp detection...");
    setGraspCandidates([]);
    setSelectedGraspIndex(-1);

    try {
      // Call grasp coordinator to start workflow
      const response = await context.callService?.("/grasp_coordinator/start_grasp_workflow", {
        num_candidates: workflowSettings.numCandidates,
        min_confidence: workflowSettings.minConfidence,
      });

      if (response && response.grasp_poses && response.grasp_poses.length > 0) {
        // Convert response to grasp candidates
        const candidates: GraspCandidate[] = response.grasp_poses.map((pose: PoseStamped, index: number) => ({
          pose,
          confidence: response.confidence_scores?.[index] || 0.5,
          width: response.grasp_widths?.[index] || 0.05,
          quality: response.quality_scores?.[index] || 0.5,
          originalIndex: response.original_indices?.[index] || index,
        }));

        setGraspCandidates(candidates);
        setWorkflowState("selecting");
        setStatusMessage(`Found ${candidates.length} grasp candidates - please select one`);
      } else {
        setWorkflowState("failed");
        setStatusMessage("No grasp candidates detected");
      }
    } catch (error) {
      setWorkflowState("failed");
      setStatusMessage(`Detection failed: ${error}`);
    } finally {
      setDetectionInProgress(false);
    }
  }, [context, workflowSettings]);

  // Select a grasp candidate
  const selectGrasp = useCallback((index: number) => {
    if (index >= 0 && index < graspCandidates.length) {
      setSelectedGraspIndex(index);
      
      if (workflowSettings.showConfirmation) {
        setConfirmationDialog(true);
      } else if (workflowSettings.autoExecute) {
        executeSelectedGrasp(index);
      }
    }
  }, [graspCandidates, workflowSettings]);

  // Execute selected grasp
  const executeSelectedGrasp = useCallback(async (graspIndex?: number) => {
    const indexToUse = graspIndex !== undefined ? graspIndex : selectedGraspIndex;
    
    if (indexToUse < 0 || indexToUse >= graspCandidates.length) {
      setStatusMessage("Invalid grasp selection");
      return;
    }

    setWorkflowState("executing");
    setConfirmationDialog(false);
    setStatusMessage("Executing selected grasp...");

    try {
      const response = await context.callService?.("/grasp_coordinator/execute_selected", {
        selected_grasp_index: indexToUse,
        show_more_candidates: false,
      });

      if (response && response.success) {
        setWorkflowState("completed");
        setStatusMessage("Grasp executed successfully!");
        
        // Auto-reset after successful completion
        setTimeout(() => {
          resetWorkflow();
        }, 3000);
      } else {
        setWorkflowState("failed");
        setStatusMessage(response?.message || "Grasp execution failed");
      }
    } catch (error) {
      setWorkflowState("failed");
      setStatusMessage(`Execution failed: ${error}`);
    }
  }, [context, selectedGraspIndex, graspCandidates]);

  // Reset workflow
  const resetWorkflow = useCallback(() => {
    setWorkflowState("idle");
    setGraspCandidates([]);
    setSelectedGraspIndex(-1);
    setStatusMessage("Ready to start grasp workflow");
    setConfirmationDialog(false);
    setDetectionInProgress(false);
  }, []);

  // Emergency stop
  const emergencyStop = useCallback(() => {
    setWorkflowState("failed");
    setStatusMessage("EMERGENCY STOP ACTIVATED");
    setConfirmationDialog(false);
    
    // Call emergency stop service if available
    context.callService?.("/grasp_coordinator/emergency_stop", {});
    
    setTimeout(() => {
      resetWorkflow();
    }, 2000);
  }, [context, resetWorkflow]);

  // Get state color for UI
  const getStateColor = (state: WorkflowState): string => {
    switch (state) {
      case "idle": return "#6c757d";
      case "detecting": return "#ffc107";
      case "selecting": return "#17a2b8";
      case "confirming": return "#fd7e14";
      case "executing": return "#007bff";
      case "completed": return "#28a745";
      case "failed": return "#dc3545";
      default: return "#6c757d";
    }
  };

  return (
    <div style={{ padding: "1rem", height: "100%", overflow: "auto" }}>
      <h3>🤖 Interactive Grasp Workflow</h3>
      
      {/* Status Section */}
      <div style={{ 
        marginBottom: "1rem", 
        padding: "0.75rem", 
        border: "2px solid", 
        borderColor: getStateColor(workflowState),
        borderRadius: "8px",
        backgroundColor: workflowState === "failed" ? "#fff5f5" : "#f8f9fa"
      }}>
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
          <div>
            <strong style={{ color: getStateColor(workflowState) }}>
              {workflowState.toUpperCase()}
            </strong>
            <div style={{ fontSize: "0.9rem", marginTop: "0.25rem" }}>
              {statusMessage}
            </div>
          </div>
          {workflowState !== "idle" && workflowState !== "completed" && (
            <button onClick={emergencyStop} style={{
              ...buttonStyle,
              backgroundColor: "#dc3545",
              color: "white",
              fontWeight: "bold"
            }}>
              🛑 STOP
            </button>
          )}
        </div>
      </div>

      {/* Workflow Settings */}
      <div style={{ marginBottom: "1rem", padding: "0.5rem", border: "1px solid #ccc", borderRadius: "4px" }}>
        <h4>⚙️ Workflow Settings</h4>
        
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0.5rem", marginBottom: "0.5rem" }}>
          <div>
            <label>Max Candidates: </label>
            <input
              type="number"
              min="1"
              max="10"
              value={workflowSettings.numCandidates}
              onChange={(e) => setWorkflowSettings(prev => ({ 
                ...prev, 
                numCandidates: parseInt(e.target.value) || 5
              }))}
              style={{ width: "60px" }}
              disabled={workflowState !== "idle"}
            />
          </div>
          
          <div>
            <label>Min Confidence: </label>
            <input
              type="number"
              min="0"
              max="1"
              step="0.1"
              value={workflowSettings.minConfidence}
              onChange={(e) => setWorkflowSettings(prev => ({ 
                ...prev, 
                minConfidence: parseFloat(e.target.value) || 0.6
              }))}
              style={{ width: "60px" }}
              disabled={workflowState !== "idle"}
            />
          </div>
        </div>
        
        <div style={{ display: "flex", gap: "1rem", fontSize: "0.9rem" }}>
          <label>
            <input
              type="checkbox"
              checked={workflowSettings.showConfirmation}
              onChange={(e) => setWorkflowSettings(prev => ({ 
                ...prev, 
                showConfirmation: e.target.checked 
              }))}
              disabled={workflowState !== "idle"}
            />
            Show confirmation dialog
          </label>
          
          <label>
            <input
              type="checkbox"
              checked={workflowSettings.autoExecute}
              onChange={(e) => setWorkflowSettings(prev => ({ 
                ...prev, 
                autoExecute: e.target.checked 
              }))}
              disabled={workflowState !== "idle"}
            />
            Auto-execute selection
          </label>
        </div>
      </div>

      {/* Main Controls */}
      <div style={{ marginBottom: "1rem" }}>
        <h4>🎯 Workflow Control</h4>
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0.5rem" }}>
          <button 
            onClick={startGraspWorkflow} 
            disabled={workflowState !== "idle"}
            style={{
              ...buttonStyle,
              backgroundColor: workflowState === "idle" ? "#28a745" : "#6c757d",
              color: "white"
            }}
          >
            {detectionInProgress ? "🔄 Detecting..." : "🔍 Start Workflow"}
          </button>
          
          <button 
            onClick={resetWorkflow}
            disabled={workflowState === "detecting" || workflowState === "executing"}
            style={buttonStyle}
          >
            🔄 Reset
          </button>
        </div>
      </div>

      {/* Grasp Candidates Selection */}
      {graspCandidates.length > 0 && workflowState === "selecting" && (
        <div style={{ marginBottom: "1rem" }}>
          <h4>🎯 Select Grasp (Top {graspCandidates.length})</h4>
          <div style={{ 
            maxHeight: "300px", 
            overflowY: "auto", 
            border: "2px solid #17a2b8", 
            borderRadius: "8px" 
          }}>
            {graspCandidates.map((candidate, index) => (
              <div
                key={index}
                style={{
                  padding: "0.75rem",
                  borderBottom: index < graspCandidates.length - 1 ? "1px solid #eee" : "none",
                  backgroundColor: selectedGraspIndex === index ? "#e3f2fd" : "transparent",
                  cursor: "pointer",
                  transition: "background-color 0.2s"
                }}
                onClick={() => selectGrasp(index)}
                onMouseEnter={(e) => {
                  if (selectedGraspIndex !== index) {
                    e.currentTarget.style.backgroundColor = "#f5f5f5";
                  }
                }}
                onMouseLeave={(e) => {
                  if (selectedGraspIndex !== index) {
                    e.currentTarget.style.backgroundColor = "transparent";
                  }
                }}
              >
                <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
                  <div>
                    <strong>🎯 Grasp #{index + 1}</strong>
                    {selectedGraspIndex === index && <span style={{ color: "#007bff", marginLeft: "0.5rem" }}>✓ SELECTED</span>}
                    <div style={{ fontSize: "0.8rem", color: "#666", marginTop: "0.25rem" }}>
                      Confidence: <strong>{(candidate.confidence * 100).toFixed(1)}%</strong> | 
                      Width: <strong>{candidate.width.toFixed(3)}m</strong> | 
                      Quality: <strong>{(candidate.quality * 100).toFixed(1)}%</strong>
                    </div>
                  </div>
                  
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      selectGrasp(index);
                      if (!workflowSettings.showConfirmation) {
                        executeSelectedGrasp(index);
                      }
                    }}
                    style={{
                      ...buttonStyle,
                      fontSize: "0.8rem",
                      padding: "0.25rem 0.5rem",
                      backgroundColor: selectedGraspIndex === index ? "#007bff" : "#28a745",
                      color: "white"
                    }}
                  >
                    {selectedGraspIndex === index ? "✓ Selected" : "▶️ Select"}
                  </button>
                </div>
                
                <div style={{ fontSize: "0.75rem", marginTop: "0.25rem", color: "#555" }}>
                  Position: ({candidate.pose.pose.position.x.toFixed(3)}, {candidate.pose.pose.position.y.toFixed(3)}, {candidate.pose.pose.position.z.toFixed(3)})
                </div>
              </div>
            ))}
          </div>
          
          {selectedGraspIndex >= 0 && (
            <div style={{ marginTop: "0.5rem", textAlign: "center" }}>
              <button
                onClick={() => executeSelectedGrasp()}
                style={{
                  ...buttonStyle,
                  backgroundColor: "#007bff",
                  color: "white",
                  fontSize: "1rem",
                  padding: "0.75rem 1.5rem",
                  fontWeight: "bold"
                }}
              >
                🚀 Execute Selected Grasp
              </button>
            </div>
          )}
        </div>
      )}

      {/* Confirmation Dialog */}
      {confirmationDialog && selectedGraspIndex >= 0 && (
        <div style={{
          position: "fixed",
          top: "50%",
          left: "50%",
          transform: "translate(-50%, -50%)",
          backgroundColor: "white",
          border: "3px solid #007bff",
          borderRadius: "12px",
          padding: "1.5rem",
          boxShadow: "0 4px 20px rgba(0,0,0,0.3)",
          zIndex: 1000,
          minWidth: "300px"
        }}>
          <h4 style={{ color: "#007bff", marginBottom: "1rem" }}>🤖 Confirm Grasp Execution</h4>
          
          <div style={{ marginBottom: "1rem" }}>
            <strong>Selected Grasp #{selectedGraspIndex + 1}</strong>
            <div style={{ fontSize: "0.9rem", color: "#666", marginTop: "0.5rem" }}>
              Confidence: {(graspCandidates[selectedGraspIndex].confidence * 100).toFixed(1)}%<br/>
              Position: ({graspCandidates[selectedGraspIndex].pose.pose.position.x.toFixed(3)}, {graspCandidates[selectedGraspIndex].pose.pose.position.y.toFixed(3)}, {graspCandidates[selectedGraspIndex].pose.pose.position.z.toFixed(3)})
            </div>
          </div>
          
          <div style={{ display: "flex", gap: "0.5rem", justifyContent: "center" }}>
            <button
              onClick={() => executeSelectedGrasp()}
              style={{
                ...buttonStyle,
                backgroundColor: "#28a745",
                color: "white",
                fontWeight: "bold"
              }}
            >
              ✅ Execute
            </button>
            
            <button
              onClick={() => setConfirmationDialog(false)}
              style={{
                ...buttonStyle,
                backgroundColor: "#6c757d",
                color: "white"
              }}
            >
              ❌ Cancel
            </button>
          </div>
        </div>
      )}

      {/* Progress Indicators */}
      {(workflowState === "detecting" || workflowState === "executing") && (
        <div style={{ 
          textAlign: "center", 
          padding: "1rem",
          backgroundColor: "#f8f9fa",
          border: "1px solid #dee2e6",
          borderRadius: "8px"
        }}>
          <div style={{ fontSize: "1.2rem", marginBottom: "0.5rem" }}>
            {workflowState === "detecting" ? "🔍 Detecting Grasps..." : "🤖 Executing Grasp..."}
          </div>
          <div style={{ 
            width: "100%", 
            height: "4px", 
            backgroundColor: "#e9ecef",
            borderRadius: "2px",
            overflow: "hidden"
          }}>
            <div style={{
              width: "100%",
              height: "100%",
              backgroundColor: getStateColor(workflowState),
              animation: "pulse 1.5s ease-in-out infinite"
            }} />
          </div>
        </div>
      )}

      {/* Instructions */}
      <div style={{ fontSize: "0.8rem", color: "#666", marginTop: "1rem" }}>
        <strong>Workflow Instructions:</strong>
        <ol style={{ margin: "0.5rem 0", paddingLeft: "1.5rem" }}>
          <li>Configure settings and click "Start Workflow"</li>
          <li>Wait for grasp detection to complete</li>
          <li>Review and select from top 5 candidates</li>
          <li>Confirm execution (if enabled)</li>
          <li>Monitor execution progress</li>
          <li>Use "STOP" for emergency halt</li>
        </ol>
      </div>
    </div>
  );
}

const buttonStyle = {
  padding: "0.5rem 0.75rem",
  border: "1px solid #ccc",
  borderRadius: "6px",
  backgroundColor: "#f8f9fa",
  cursor: "pointer",
  fontSize: "0.9rem",
  transition: "all 0.2s"
};

export function initAnyGraspInteractivePanel(context: PanelExtensionContext): void {
  ReactDOM.render(<AnyGraspInteractivePanel context={context} />, context.panelElement);
}

// Export for Foxglove Studio
if (typeof window !== "undefined") {
  (window as any).registerFoxgloveInteractiveGraspPanel = (context: PanelExtensionContext) => {
    initAnyGraspInteractivePanel(context);
  };
}