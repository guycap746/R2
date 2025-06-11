import { PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import { useEffect, useLayoutEffect, useState, useCallback } from "react";
import ReactDOM from "react-dom";

// Define message types for training data
type TrainingStatus = {
  is_training: boolean;
  current_epoch: number;
  total_epochs: number;
  current_batch: number;
  total_batches: number;
  learning_rate: number;
  estimated_time_remaining: number;
  model_name: string;
  training_mode: string;
};

type TrainingMetrics = {
  epoch: number;
  batch: number;
  loss: number;
  accuracy: number;
  validation_loss: number;
  validation_accuracy: number;
  timestamp: number;
};

type PerformanceMetrics = {
  success_rate: number;
  average_execution_time: number;
  grasp_confidence: number;
  improvement_trend: string;
  recommendation: string;
};

function TrainingDashboardPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [trainingStatus, setTrainingStatus] = useState<TrainingStatus | null>(null);
  const [trainingMetrics, setTrainingMetrics] = useState<TrainingMetrics[]>([]);
  const [performanceMetrics, setPerformanceMetrics] = useState<PerformanceMetrics | null>(null);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Subscribe to training topics
  useLayoutEffect(() => {
    context.subscribe([
      { topic: "/lerobot/trainer_status" },
      { topic: "/lerobot/training_stats" },
      { topic: "/lerobot/performance_analytics" },
    ]);
  }, [context]);

  // Handle incoming messages
  useEffect(() => {
    context.onRender = (renderState: RenderState, done) => {
      setRenderDone(() => done);

      if (renderState.currentFrame) {
        for (const messageEvent of renderState.currentFrame) {
          const msg = messageEvent as MessageEvent<any>;
          
          switch (msg.topic) {
            case "/lerobot/trainer_status":
              setTrainingStatus(msg.message);
              break;
            case "/lerobot/training_stats":
              setTrainingMetrics(prev => {
                const newMetrics = [...prev, msg.message];
                // Keep only last 1000 data points for performance
                return newMetrics.slice(-1000);
              });
              break;
            case "/lerobot/performance_analytics":
              setPerformanceMetrics(msg.message);
              break;
          }
        }
      }
    };
  }, [context]);

  // Training control functions
  const startTraining = useCallback(() => {
    context.callService?.("/lerobot/start_training", {});
  }, [context]);

  const pauseTraining = useCallback(() => {
    context.callService?.("/lerobot/pause_training", {});
  }, [context]);

  const stopTraining = useCallback(() => {
    context.callService?.("/lerobot/stop_training", {});
  }, [context]);

  // Calculate training progress
  const getTrainingProgress = () => {
    if (!trainingStatus) return 0;
    const epochProgress = trainingStatus.current_epoch / trainingStatus.total_epochs;
    const batchProgress = trainingStatus.current_batch / trainingStatus.total_batches;
    return (epochProgress + batchProgress / trainingStatus.total_epochs) * 100;
  };

  // Format time remaining
  const formatTimeRemaining = (seconds: number) => {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = Math.floor(seconds % 60);
    return `${hours}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  // Render loss chart (simplified SVG visualization)
  const renderLossChart = () => {
    if (trainingMetrics.length < 2) return null;

    const width = 400;
    const height = 200;
    const margin = { top: 20, right: 30, bottom: 30, left: 50 };
    const chartWidth = width - margin.left - margin.right;
    const chartHeight = height - margin.top - margin.bottom;

    const maxLoss = Math.max(...trainingMetrics.map(m => Math.max(m.loss, m.validation_loss || 0)));
    const minLoss = Math.min(...trainingMetrics.map(m => Math.min(m.loss, m.validation_loss || 0)));

    const points = trainingMetrics.map((metric, index) => {
      const x = margin.left + (index / (trainingMetrics.length - 1)) * chartWidth;
      const y = margin.top + (1 - (metric.loss - minLoss) / (maxLoss - minLoss)) * chartHeight;
      return `${x},${y}`;
    }).join(' ');

    const valPoints = trainingMetrics.filter(m => m.validation_loss != null).map((metric, index) => {
      const x = margin.left + (index / (trainingMetrics.length - 1)) * chartWidth;
      const y = margin.top + (1 - (metric.validation_loss! - minLoss) / (maxLoss - minLoss)) * chartHeight;
      return `${x},${y}`;
    }).join(' ');

    return (
      <svg width={width} height={height} style={{ backgroundColor: '#f8f9fa', border: '1px solid #ddd' }}>
        {/* Training Loss Line */}
        <polyline
          points={points}
          fill="none"
          stroke="#007bff"
          strokeWidth="2"
        />
        {/* Validation Loss Line */}
        {valPoints && (
          <polyline
            points={valPoints}
            fill="none"
            stroke="#ff6b6b"
            strokeWidth="2"
            strokeDasharray="5,5"
          />
        )}
        {/* Y-axis */}
        <line x1={margin.left} y1={margin.top} x2={margin.left} y2={height - margin.bottom} stroke="#333" />
        {/* X-axis */}
        <line x1={margin.left} y1={height - margin.bottom} x2={width - margin.right} y2={height - margin.bottom} stroke="#333" />
        {/* Labels */}
        <text x={10} y={15} fontSize="12" fill="#333">Loss</text>
        <text x={width - 50} y={height - 5} fontSize="12" fill="#333">Epoch</text>
      </svg>
    );
  };

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div style={{ padding: '16px', fontFamily: 'Arial, sans-serif' }}>
      <h2 style={{ marginBottom: '20px', color: '#333' }}>🤖 Training Dashboard</h2>
      
      {/* Training Status Section */}
      <div style={{ marginBottom: '20px', padding: '16px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
        <h3 style={{ marginBottom: '12px' }}>Training Status</h3>
        {trainingStatus ? (
          <div>
            <div style={{ marginBottom: '8px' }}>
              <strong>Status:</strong> {trainingStatus.is_training ? '🟢 Training' : '🔴 Stopped'}
            </div>
            <div style={{ marginBottom: '8px' }}>
              <strong>Model:</strong> {trainingStatus.model_name} ({trainingStatus.training_mode})
            </div>
            <div style={{ marginBottom: '8px' }}>
              <strong>Progress:</strong> Epoch {trainingStatus.current_epoch}/{trainingStatus.total_epochs}, 
              Batch {trainingStatus.current_batch}/{trainingStatus.total_batches}
            </div>
            <div style={{ marginBottom: '12px' }}>
              <div style={{ 
                width: '100%', 
                height: '20px', 
                backgroundColor: '#e9ecef', 
                borderRadius: '10px',
                overflow: 'hidden'
              }}>
                <div style={{
                  width: `${getTrainingProgress()}%`,
                  height: '100%',
                  backgroundColor: '#28a745',
                  transition: 'width 0.3s ease'
                }} />
              </div>
              <small>{getTrainingProgress().toFixed(1)}% complete</small>
            </div>
            <div style={{ marginBottom: '8px' }}>
              <strong>Learning Rate:</strong> {trainingStatus.learning_rate.toExponential(2)}
            </div>
            <div style={{ marginBottom: '12px' }}>
              <strong>Time Remaining:</strong> {formatTimeRemaining(trainingStatus.estimated_time_remaining)}
            </div>
          </div>
        ) : (
          <div style={{ color: '#6c757d' }}>No training status available</div>
        )}

        {/* Training Controls */}
        <div style={{ display: 'flex', gap: '8px', marginTop: '12px' }}>
          <button 
            onClick={startTraining}
            style={{ 
              padding: '8px 16px', 
              backgroundColor: '#28a745', 
              color: 'white', 
              border: 'none', 
              borderRadius: '4px',
              cursor: 'pointer'
            }}
            disabled={trainingStatus?.is_training}
          >
            ▶️ Start
          </button>
          <button 
            onClick={pauseTraining}
            style={{ 
              padding: '8px 16px', 
              backgroundColor: '#ffc107', 
              color: 'white', 
              border: 'none', 
              borderRadius: '4px',
              cursor: 'pointer'
            }}
            disabled={!trainingStatus?.is_training}
          >
            ⏸️ Pause
          </button>
          <button 
            onClick={stopTraining}
            style={{ 
              padding: '8px 16px', 
              backgroundColor: '#dc3545', 
              color: 'white', 
              border: 'none', 
              borderRadius: '4px',
              cursor: 'pointer'
            }}
          >
            ⏹️ Stop
          </button>
        </div>
      </div>

      {/* Training Metrics Section */}
      <div style={{ marginBottom: '20px', padding: '16px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
        <h3 style={{ marginBottom: '12px' }}>Training Metrics</h3>
        {trainingMetrics.length > 0 ? (
          <div>
            <div style={{ marginBottom: '16px' }}>
              {renderLossChart()}
            </div>
            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '16px' }}>
              <div>
                <strong>Latest Loss:</strong> {trainingMetrics[trainingMetrics.length - 1]?.loss.toFixed(4)}
              </div>
              <div>
                <strong>Latest Accuracy:</strong> {(trainingMetrics[trainingMetrics.length - 1]?.accuracy * 100).toFixed(2)}%
              </div>
              {trainingMetrics[trainingMetrics.length - 1]?.validation_loss && (
                <>
                  <div>
                    <strong>Validation Loss:</strong> {trainingMetrics[trainingMetrics.length - 1]?.validation_loss.toFixed(4)}
                  </div>
                  <div>
                    <strong>Validation Accuracy:</strong> {(trainingMetrics[trainingMetrics.length - 1]?.validation_accuracy * 100).toFixed(2)}%
                  </div>
                </>
              )}
            </div>
          </div>
        ) : (
          <div style={{ color: '#6c757d' }}>No training metrics available</div>
        )}
      </div>

      {/* Performance Analytics Section */}
      <div style={{ marginBottom: '20px', padding: '16px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
        <h3 style={{ marginBottom: '12px' }}>Performance Analytics</h3>
        {performanceMetrics ? (
          <div>
            <div style={{ marginBottom: '8px' }}>
              <strong>Success Rate:</strong> {(performanceMetrics.success_rate * 100).toFixed(2)}%
            </div>
            <div style={{ marginBottom: '8px' }}>
              <strong>Avg Execution Time:</strong> {performanceMetrics.average_execution_time.toFixed(2)}s
            </div>
            <div style={{ marginBottom: '8px' }}>
              <strong>Grasp Confidence:</strong> {(performanceMetrics.grasp_confidence * 100).toFixed(2)}%
            </div>
            <div style={{ marginBottom: '8px' }}>
              <strong>Trend:</strong> {performanceMetrics.improvement_trend}
            </div>
            {performanceMetrics.recommendation && (
              <div style={{ 
                marginTop: '12px', 
                padding: '8px', 
                backgroundColor: '#d1ecf1', 
                borderRadius: '4px',
                borderLeft: '4px solid #bee5eb'
              }}>
                <strong>💡 Recommendation:</strong> {performanceMetrics.recommendation}
              </div>
            )}
          </div>
        ) : (
          <div style={{ color: '#6c757d' }}>No performance analytics available</div>
        )}
      </div>
    </div>
  );
}

export function initTrainingDashboardPanel(context: PanelExtensionContext): void {
  ReactDOM.render(<TrainingDashboardPanel context={context} />, context.panelElement);
}