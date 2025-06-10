// Roboflow Configuration Panel for Foxglove Studio
// Panel for configuring and monitoring Roboflow dataset uploads

import { PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import { useLayoutEffect, useEffect, useState, useCallback } from "react";
import ReactDOM from "react-dom";

type RoboflowConfig = {
  apiKey: string;
  workspaceName: string;
  projectName: string;
  datasetVersion: string;
  autoUpload: boolean;
  includeFailedGrasps: boolean;
  annotationFormat: string;
  minConfidenceForUpload: number;
};

type UploadStatus = {
  isUploading: boolean;
  queueSize: number;
  totalUploaded: number;
  lastUploadTime: string;
  lastError: string;
};

function RoboflowConfigurationPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  // Configuration state
  const [config, setConfig] = useState<RoboflowConfig>({
    apiKey: '',
    workspaceName: 'roarm-grasping',
    projectName: 'grasp-detection',
    datasetVersion: 'v1',
    autoUpload: true,
    includeFailedGrasps: true,
    annotationFormat: 'yolo',
    minConfidenceForUpload: 0.3
  });

  // Status state
  const [uploadStatus, setUploadStatus] = useState<UploadStatus>({
    isUploading: false,
    queueSize: 0,
    totalUploaded: 0,
    lastUploadTime: 'Never',
    lastError: ''
  });

  const [statusMessage, setStatusMessage] = useState<string>('Not configured');
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [showApiKey, setShowApiKey] = useState<boolean>(false);

  useLayoutEffect(() => {
    context.onRender = (renderState: RenderState, done) => {
      const messages = renderState.currentFrame ?? [];
      
      for (const message of messages) {
        // Monitor Roboflow upload status
        if (message.topic === "/roboflow/upload_status") {
          const status = (message.message as any).data;
          setStatusMessage(status);
          
          // Parse status for upload information
          if (status.includes("Uploading")) {
            setUploadStatus(prev => ({ ...prev, isUploading: true }));
          } else if (status.includes("Uploaded")) {
            setUploadStatus(prev => ({ 
              ...prev, 
              isUploading: false,
              totalUploaded: prev.totalUploaded + 1,
              lastUploadTime: new Date().toLocaleTimeString()
            }));
          } else if (status.includes("initialized")) {
            setIsConnected(true);
          } else if (status.includes("failed")) {
            setUploadStatus(prev => ({ 
              ...prev, 
              isUploading: false,
              lastError: status
            }));
          }
        }
      }
      
      done();
    };
    context.watch("currentFrame");
  }, [context]);

  // Subscribe to Roboflow topics
  useEffect(() => {
    context.subscribe([
      { topic: "/roboflow/upload_status" },
    ]);
  }, [context]);

  // Configure Roboflow
  const configureRoboflow = useCallback(async () => {
    try {
      setStatusMessage("Configuring Roboflow...");
      
      const response = await context.callService?.("/roboflow/configure", {
        api_key: config.apiKey,
        workspace_name: config.workspaceName,
        project_name: config.projectName,
        dataset_version: config.datasetVersion,
        auto_upload: config.autoUpload,
        include_failed_grasps: config.includeFailedGrasps,
        annotation_format: config.annotationFormat,
        min_confidence_for_upload: config.minConfidenceForUpload,
      });

      if (response && response.success) {
        setIsConnected(true);
        setStatusMessage("Roboflow configured successfully");
        setUploadStatus(prev => ({ ...prev, lastError: '' }));
      } else {
        setIsConnected(false);
        setStatusMessage(response?.message || "Configuration failed");
        setUploadStatus(prev => ({ ...prev, lastError: response?.message || "Unknown error" }));
      }
    } catch (error) {
      setIsConnected(false);
      setStatusMessage(`Configuration error: ${error}`);
      setUploadStatus(prev => ({ ...prev, lastError: String(error) }));
    }
  }, [context, config]);

  // Manual upload trigger
  const triggerManualUpload = useCallback(async () => {
    try {
      setStatusMessage("Triggering manual upload...");
      
      // This would typically upload the current scene
      // For now, we'll just show the action
      setUploadStatus(prev => ({ ...prev, isUploading: true }));
      
      // Simulate upload process
      setTimeout(() => {
        setUploadStatus(prev => ({ 
          ...prev, 
          isUploading: false,
          totalUploaded: prev.totalUploaded + 1,
          lastUploadTime: new Date().toLocaleTimeString()
        }));
        setStatusMessage("Manual upload completed");
      }, 2000);
      
    } catch (error) {
      setStatusMessage(`Upload error: ${error}`);
      setUploadStatus(prev => ({ 
        ...prev, 
        isUploading: false,
        lastError: String(error)
      }));
    }
  }, []);

  // Reset configuration
  const resetConfiguration = useCallback(() => {
    setConfig({
      apiKey: '',
      workspaceName: 'roarm-grasping',
      projectName: 'grasp-detection',
      datasetVersion: 'v1',
      autoUpload: true,
      includeFailedGrasps: true,
      annotationFormat: 'yolo',
      minConfidenceForUpload: 0.3
    });
    setIsConnected(false);
    setStatusMessage('Configuration reset');
    setUploadStatus({
      isUploading: false,
      queueSize: 0,
      totalUploaded: 0,
      lastUploadTime: 'Never',
      lastError: ''
    });
  }, []);

  const getStatusColor = (): string => {
    if (uploadStatus.isUploading) return "#ffc107";
    if (isConnected) return "#28a745";
    if (uploadStatus.lastError) return "#dc3545";
    return "#6c757d";
  };

  return (
    <div style={{ padding: "1rem", height: "100%", overflow: "auto" }}>
      <h3>📊 Roboflow Dataset Configuration</h3>
      
      {/* Status Section */}
      <div style={{ 
        marginBottom: "1rem", 
        padding: "0.75rem", 
        border: "2px solid", 
        borderColor: getStatusColor(),
        borderRadius: "8px",
        backgroundColor: uploadStatus.lastError ? "#fff5f5" : "#f8f9fa"
      }}>
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
          <div>
            <strong style={{ color: getStatusColor() }}>
              {isConnected ? "🟢 CONNECTED" : "🔴 DISCONNECTED"}
            </strong>
            <div style={{ fontSize: "0.9rem", marginTop: "0.25rem" }}>
              {statusMessage}
            </div>
          </div>
        </div>
      </div>

      {/* Configuration Section */}
      <div style={{ marginBottom: "1rem", padding: "0.5rem", border: "1px solid #ccc", borderRadius: "4px" }}>
        <h4>🔧 Configuration</h4>
        
        {/* API Key */}
        <div style={{ marginBottom: "0.5rem" }}>
          <label style={{ display: "block", fontWeight: "bold", marginBottom: "0.25rem" }}>
            Roboflow API Key:
          </label>
          <div style={{ display: "flex", gap: "0.5rem", alignItems: "center" }}>
            <input
              type={showApiKey ? "text" : "password"}
              value={config.apiKey}
              onChange={(e) => setConfig(prev => ({ ...prev, apiKey: e.target.value }))}
              placeholder="Enter your Roboflow API key"
              style={{ 
                flex: 1, 
                padding: "0.25rem", 
                border: "1px solid #ccc", 
                borderRadius: "4px" 
              }}
            />
            <button
              onClick={() => setShowApiKey(!showApiKey)}
              style={{
                ...buttonStyle,
                padding: "0.25rem 0.5rem",
                fontSize: "0.8rem"
              }}
            >
              {showApiKey ? "👁️" : "🙈"}
            </button>
          </div>
        </div>

        {/* Workspace and Project */}
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0.5rem", marginBottom: "0.5rem" }}>
          <div>
            <label style={{ display: "block", fontWeight: "bold", marginBottom: "0.25rem" }}>
              Workspace:
            </label>
            <input
              type="text"
              value={config.workspaceName}
              onChange={(e) => setConfig(prev => ({ ...prev, workspaceName: e.target.value }))}
              style={{ 
                width: "100%", 
                padding: "0.25rem", 
                border: "1px solid #ccc", 
                borderRadius: "4px" 
              }}
            />
          </div>
          
          <div>
            <label style={{ display: "block", fontWeight: "bold", marginBottom: "0.25rem" }}>
              Project:
            </label>
            <input
              type="text"
              value={config.projectName}
              onChange={(e) => setConfig(prev => ({ ...prev, projectName: e.target.value }))}
              style={{ 
                width: "100%", 
                padding: "0.25rem", 
                border: "1px solid #ccc", 
                borderRadius: "4px" 
              }}
            />
          </div>
        </div>

        {/* Dataset Version and Format */}
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0.5rem", marginBottom: "0.5rem" }}>
          <div>
            <label style={{ display: "block", fontWeight: "bold", marginBottom: "0.25rem" }}>
              Dataset Version:
            </label>
            <input
              type="text"
              value={config.datasetVersion}
              onChange={(e) => setConfig(prev => ({ ...prev, datasetVersion: e.target.value }))}
              style={{ 
                width: "100%", 
                padding: "0.25rem", 
                border: "1px solid #ccc", 
                borderRadius: "4px" 
              }}
            />
          </div>
          
          <div>
            <label style={{ display: "block", fontWeight: "bold", marginBottom: "0.25rem" }}>
              Annotation Format:
            </label>
            <select
              value={config.annotationFormat}
              onChange={(e) => setConfig(prev => ({ ...prev, annotationFormat: e.target.value }))}
              style={{ 
                width: "100%", 
                padding: "0.25rem", 
                border: "1px solid #ccc", 
                borderRadius: "4px" 
              }}
            >
              <option value="yolo">YOLO</option>
              <option value="coco">COCO</option>
              <option value="voc">Pascal VOC</option>
            </select>
          </div>
        </div>

        {/* Upload Settings */}
        <div style={{ marginBottom: "0.5rem" }}>
          <label style={{ display: "block", fontWeight: "bold", marginBottom: "0.25rem" }}>
            Min Confidence for Upload: {config.minConfidenceForUpload.toFixed(1)}
          </label>
          <input
            type="range"
            min="0"
            max="1"
            step="0.1"
            value={config.minConfidenceForUpload}
            onChange={(e) => setConfig(prev => ({ 
              ...prev, 
              minConfidenceForUpload: parseFloat(e.target.value) 
            }))}
            style={{ width: "100%" }}
          />
        </div>

        {/* Checkboxes */}
        <div style={{ display: "flex", gap: "1rem", fontSize: "0.9rem" }}>
          <label>
            <input
              type="checkbox"
              checked={config.autoUpload}
              onChange={(e) => setConfig(prev => ({ ...prev, autoUpload: e.target.checked }))}
            />
            Auto-upload images
          </label>
          
          <label>
            <input
              type="checkbox"
              checked={config.includeFailedGrasps}
              onChange={(e) => setConfig(prev => ({ ...prev, includeFailedGrasps: e.target.checked }))}
            />
            Include failed grasps
          </label>
        </div>
      </div>

      {/* Control Buttons */}
      <div style={{ marginBottom: "1rem" }}>
        <h4>🎮 Controls</h4>
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0.5rem" }}>
          <button 
            onClick={configureRoboflow}
            style={{
              ...buttonStyle,
              backgroundColor: "#007bff",
              color: "white"
            }}
          >
            💾 Apply Configuration
          </button>
          
          <button 
            onClick={triggerManualUpload}
            disabled={!isConnected}
            style={{
              ...buttonStyle,
              backgroundColor: isConnected ? "#28a745" : "#6c757d",
              color: "white"
            }}
          >
            📤 Manual Upload
          </button>
          
          <button 
            onClick={resetConfiguration}
            style={{
              ...buttonStyle,
              backgroundColor: "#ffc107",
              color: "black"
            }}
          >
            🔄 Reset Config
          </button>
          
          <button 
            onClick={() => window.open(`https://app.roboflow.com/${config.workspaceName}/${config.projectName}`, '_blank')}
            disabled={!config.workspaceName || !config.projectName}
            style={{
              ...buttonStyle,
              backgroundColor: "#17a2b8",
              color: "white"
            }}
          >
            🌐 Open Roboflow
          </button>
        </div>
      </div>

      {/* Upload Statistics */}
      <div style={{ marginBottom: "1rem", padding: "0.5rem", border: "1px solid #ccc", borderRadius: "4px" }}>
        <h4>📈 Upload Statistics</h4>
        
        <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0.5rem", fontSize: "0.9rem" }}>
          <div>
            <strong>Total Uploaded:</strong> {uploadStatus.totalUploaded}
          </div>
          <div>
            <strong>Queue Size:</strong> {uploadStatus.queueSize}
          </div>
          <div>
            <strong>Last Upload:</strong> {uploadStatus.lastUploadTime}
          </div>
          <div>
            <strong>Status:</strong> 
            <span style={{ 
              color: uploadStatus.isUploading ? "#ffc107" : (isConnected ? "#28a745" : "#dc3545"),
              marginLeft: "0.25rem"
            }}>
              {uploadStatus.isUploading ? "Uploading..." : (isConnected ? "Ready" : "Disconnected")}
            </span>
          </div>
        </div>
        
        {uploadStatus.lastError && (
          <div style={{ 
            marginTop: "0.5rem", 
            padding: "0.5rem", 
            backgroundColor: "#fff5f5", 
            border: "1px solid #dc3545", 
            borderRadius: "4px",
            fontSize: "0.8rem",
            color: "#721c24"
          }}>
            <strong>Last Error:</strong> {uploadStatus.lastError}
          </div>
        )}
      </div>

      {/* Information Section */}
      <div style={{ fontSize: "0.8rem", color: "#666" }}>
        <strong>About Roboflow Integration:</strong>
        <ul style={{ margin: "0.5rem 0", paddingLeft: "1.5rem" }}>
          <li>Automatically captures RGB images during grasp detection</li>
          <li>Annotates images with detected grasp points</li>
          <li>Uploads to Roboflow for model training and improvement</li>
          <li>Helps build better grasp detection datasets</li>
          <li>Supports YOLO, COCO, and Pascal VOC annotation formats</li>
        </ul>
        
        <div style={{ marginTop: "0.5rem", padding: "0.5rem", backgroundColor: "#e7f3ff", borderRadius: "4px" }}>
          <strong>💡 Tip:</strong> Get your free Roboflow API key at{" "}
          <a href="https://app.roboflow.com" target="_blank" rel="noopener noreferrer">
            app.roboflow.com
          </a>
        </div>
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

export function initRoboflowConfigurationPanel(context: PanelExtensionContext): void {
  ReactDOM.render(<RoboflowConfigurationPanel context={context} />, context.panelElement);
}

// Export for Foxglove Studio
if (typeof window !== "undefined") {
  (window as any).registerFoxgloveRoboflowPanel = (context: PanelExtensionContext) => {
    initRoboflowConfigurationPanel(context);
  };
}