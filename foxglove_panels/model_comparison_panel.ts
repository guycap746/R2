import { PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import { useEffect, useLayoutEffect, useState, useCallback } from "react";
import ReactDOM from "react-dom";

// Define message types for model comparison
type ModelInfo = {
  name: string;
  version: string;
  training_date: string;
  architecture: string;
  dataset_used: string;
  total_parameters: number;
  training_epochs: number;
  best_validation_loss: number;
};

type ModelPerformance = {
  model_name: string;
  model_version: string;
  success_rate: number;
  average_execution_time: number;
  grasp_confidence: number;
  episodes_tested: number;
  timestamp: number;
  test_conditions: string[];
};

type ABTestStatus = {
  active: boolean;
  model_a: string;
  model_b: string;
  duration_hours: number;
  episodes_completed: number;
  target_episodes: number;
  current_leader: string;
  confidence_interval: number;
};

type ComparisonMetrics = {
  model_name: string;
  metrics: {
    success_rate: number;
    avg_execution_time: number;
    consistency_score: number;
    robustness_score: number;
    improvement_over_baseline: number;
  };
  statistical_significance: boolean;
  sample_size: number;
};

function ModelComparisonPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [availableModels, setAvailableModels] = useState<ModelInfo[]>([]);
  const [selectedModels, setSelectedModels] = useState<string[]>([]);
  const [performanceData, setPerformanceData] = useState<ModelPerformance[]>([]);
  const [comparisonResults, setComparisonResults] = useState<ComparisonMetrics[]>([]);
  const [abTestStatus, setABTestStatus] = useState<ABTestStatus | null>(null);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Subscribe to model comparison topics
  useLayoutEffect(() => {
    context.subscribe([
      { topic: "/model_comparison/available_models" },
      { topic: "/model_comparison/performance_data" },
      { topic: "/model_comparison/comparison_results" },
      { topic: "/model_comparison/ab_test_status" },
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
            case "/model_comparison/available_models":
              setAvailableModels(msg.message.models || []);
              break;
            case "/model_comparison/performance_data":
              setPerformanceData(prev => {
                const newData = [...prev, msg.message];
                // Keep only recent data (last 1000 entries)
                return newData.slice(-1000);
              });
              break;
            case "/model_comparison/comparison_results":
              setComparisonResults(msg.message.results || []);
              break;
            case "/model_comparison/ab_test_status":
              setABTestStatus(msg.message);
              break;
          }
        }
      }
    };
  }, [context]);

  // Model selection functions
  const toggleModelSelection = useCallback((modelName: string) => {
    setSelectedModels(prev => {
      if (prev.includes(modelName)) {
        return prev.filter(name => name !== modelName);
      } else if (prev.length < 4) { // Limit to 4 models for comparison
        return [...prev, modelName];
      }
      return prev;
    });
  }, []);

  // Comparison control functions
  const startComparison = useCallback(() => {
    if (selectedModels.length >= 2) {
      context.callService?.("/model_comparison/start_comparison", {
        models: selectedModels,
        episodes_per_model: 50,
        test_scenarios: ["standard", "challenging", "edge_cases"]
      });
    }
  }, [context, selectedModels]);

  const startABTest = useCallback(() => {
    if (selectedModels.length === 2) {
      context.callService?.("/model_comparison/start_ab_test", {
        model_a: selectedModels[0],
        model_b: selectedModels[1],
        duration_hours: 24,
        target_episodes: 100
      });
    }
  }, [context, selectedModels]);

  const stopABTest = useCallback(() => {
    context.callService?.("/model_comparison/stop_ab_test", {});
  }, [context]);

  const deployWinner = useCallback((modelName: string) => {
    context.callService?.("/model_comparison/deploy_model", {
      model_name: modelName,
      replace_current: true
    });
  }, [context]);

  // Get performance summary for a model
  const getModelSummary = (modelName: string) => {
    const modelData = performanceData.filter(d => d.model_name === modelName);
    if (modelData.length === 0) return null;

    const recent = modelData.slice(-20); // Last 20 episodes
    const avgSuccessRate = recent.reduce((sum, d) => sum + d.success_rate, 0) / recent.length;
    const avgExecutionTime = recent.reduce((sum, d) => sum + d.average_execution_time, 0) / recent.length;
    const avgConfidence = recent.reduce((sum, d) => sum + d.grasp_confidence, 0) / recent.length;

    return {
      success_rate: avgSuccessRate,
      execution_time: avgExecutionTime,
      confidence: avgConfidence,
      episodes: recent.length
    };
  };

  // Render performance chart comparison
  const renderPerformanceChart = () => {
    if (selectedModels.length < 2) return null;

    const width = 600;
    const height = 300;
    const margin = { top: 20, right: 100, bottom: 40, left: 60 };
    const chartWidth = width - margin.left - margin.right;
    const chartHeight = height - margin.top - margin.bottom;

    const colors = ['#007bff', '#28a745', '#dc3545', '#ffc107'];
    
    // Get recent performance data for selected models
    const modelLines = selectedModels.map((modelName, index) => {
      const modelData = performanceData
        .filter(d => d.model_name === modelName)
        .slice(-50); // Last 50 episodes

      if (modelData.length < 2) return null;

      const points = modelData.map((data, i) => {
        const x = margin.left + (i / (modelData.length - 1)) * chartWidth;
        const y = margin.top + (1 - data.success_rate) * chartHeight;
        return `${x},${y}`;
      }).join(' ');

      return (
        <g key={modelName}>
          <polyline
            points={points}
            fill="none"
            stroke={colors[index]}
            strokeWidth="2"
          />
          <text
            x={width - margin.right + 5}
            y={margin.top + index * 20}
            fontSize="12"
            fill={colors[index]}
          >
            {modelName}
          </text>
        </g>
      );
    }).filter(Boolean);

    return (
      <svg width={width} height={height} style={{ backgroundColor: '#f8f9fa', border: '1px solid #ddd' }}>
        {/* Chart area */}
        <rect
          x={margin.left}
          y={margin.top}
          width={chartWidth}
          height={chartHeight}
          fill="white"
          stroke="#ddd"
        />
        
        {/* Model lines */}
        {modelLines}
        
        {/* Y-axis */}
        <line x1={margin.left} y1={margin.top} x2={margin.left} y2={height - margin.bottom} stroke="#333" />
        {/* X-axis */}
        <line x1={margin.left} y1={height - margin.bottom} x2={width - margin.right} y2={height - margin.bottom} stroke="#333" />
        
        {/* Labels */}
        <text x={20} y={height / 2} fontSize="12" fill="#333" transform={`rotate(-90, 20, ${height / 2})`}>
          Success Rate
        </text>
        <text x={width / 2} y={height - 5} fontSize="12" fill="#333" textAnchor="middle">
          Episodes
        </text>
        <text x={width / 2} y={15} fontSize="14" fill="#333" textAnchor="middle" fontWeight="bold">
          Model Performance Comparison
        </text>
      </svg>
    );
  };

  // Performance indicator component
  const PerformanceIndicator = ({ value, label, unit = "", color = "#007bff" }: { 
    value: number; 
    label: string; 
    unit?: string; 
    color?: string; 
  }) => (
    <div style={{ textAlign: 'center', padding: '8px' }}>
      <div style={{ fontSize: '24px', fontWeight: 'bold', color }}>{value.toFixed(2)}{unit}</div>
      <div style={{ fontSize: '12px', color: '#6c757d' }}>{label}</div>
    </div>
  );

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div style={{ padding: '16px', fontFamily: 'Arial, sans-serif', height: '100%', overflow: 'auto' }}>
      <h2 style={{ marginBottom: '20px', color: '#333' }}>🤖 Model Comparison & A/B Testing</h2>

      {/* Model Selection */}
      <div style={{ marginBottom: '20px', padding: '16px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
        <h3 style={{ marginBottom: '12px' }}>Available Models</h3>
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fill, minmax(300px, 1fr))', gap: '12px' }}>
          {availableModels.map((model) => (
            <div
              key={`${model.name}_${model.version}`}
              onClick={() => toggleModelSelection(`${model.name}_${model.version}`)}
              style={{
                padding: '12px',
                backgroundColor: selectedModels.includes(`${model.name}_${model.version}`) ? '#e3f2fd' : 'white',
                border: `2px solid ${selectedModels.includes(`${model.name}_${model.version}`) ? '#2196f3' : '#ddd'}`,
                borderRadius: '6px',
                cursor: 'pointer',
                transition: 'all 0.2s ease'
              }}
            >
              <div style={{ fontWeight: 'bold', marginBottom: '4px' }}>
                {model.name} v{model.version}
              </div>
              <div style={{ fontSize: '12px', color: '#6c757d' }}>
                {model.architecture} • {model.total_parameters.toLocaleString()} params
              </div>
              <div style={{ fontSize: '12px', color: '#6c757d' }}>
                Trained: {model.training_date} • Loss: {model.best_validation_loss.toFixed(4)}
              </div>
            </div>
          ))}
        </div>
        
        <div style={{ marginTop: '12px', fontSize: '14px', color: '#6c757d' }}>
          Selected: {selectedModels.length}/4 models
        </div>
      </div>

      {/* Comparison Controls */}
      {selectedModels.length >= 2 && (
        <div style={{ marginBottom: '20px', padding: '16px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
          <h3 style={{ marginBottom: '12px' }}>Comparison Controls</h3>
          <div style={{ display: 'flex', gap: '12px', marginBottom: '16px' }}>
            <button
              onClick={startComparison}
              style={{
                padding: '10px 20px',
                backgroundColor: '#007bff',
                color: 'white',
                border: 'none',
                borderRadius: '6px',
                cursor: 'pointer',
                fontWeight: 'bold'
              }}
            >
              🔬 Start Detailed Comparison
            </button>
            
            {selectedModels.length === 2 && (
              <button
                onClick={startABTest}
                style={{
                  padding: '10px 20px',
                  backgroundColor: '#28a745',
                  color: 'white',
                  border: 'none',
                  borderRadius: '6px',
                  cursor: 'pointer',
                  fontWeight: 'bold'
                }}
                disabled={abTestStatus?.active}
              >
                🧪 Start A/B Test
              </button>
            )}
          </div>
        </div>
      )}

      {/* A/B Test Status */}
      {abTestStatus?.active && (
        <div style={{ marginBottom: '20px', padding: '16px', backgroundColor: '#fff3cd', borderRadius: '8px', border: '1px solid #ffeaa7' }}>
          <h3 style={{ marginBottom: '12px', color: '#856404' }}>🧪 A/B Test in Progress</h3>
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '16px', marginBottom: '12px' }}>
            <div>
              <strong>Model A:</strong> {abTestStatus.model_a}<br />
              <strong>Model B:</strong> {abTestStatus.model_b}
            </div>
            <div>
              <strong>Progress:</strong> {abTestStatus.episodes_completed}/{abTestStatus.target_episodes} episodes<br />
              <strong>Current Leader:</strong> {abTestStatus.current_leader}
            </div>
          </div>
          
          <div style={{ marginBottom: '12px' }}>
            <div style={{
              width: '100%',
              height: '8px',
              backgroundColor: '#e9ecef',
              borderRadius: '4px',
              overflow: 'hidden'
            }}>
              <div style={{
                width: `${(abTestStatus.episodes_completed / abTestStatus.target_episodes) * 100}%`,
                height: '100%',
                backgroundColor: '#28a745'
              }} />
            </div>
            <small>{((abTestStatus.episodes_completed / abTestStatus.target_episodes) * 100).toFixed(1)}% complete</small>
          </div>

          <div style={{ display: 'flex', gap: '8px' }}>
            <button
              onClick={stopABTest}
              style={{
                padding: '8px 16px',
                backgroundColor: '#dc3545',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer'
              }}
            >
              Stop A/B Test
            </button>
            {abTestStatus.current_leader && (
              <button
                onClick={() => deployWinner(abTestStatus.current_leader)}
                style={{
                  padding: '8px 16px',
                  backgroundColor: '#ffc107',
                  color: 'black',
                  border: 'none',
                  borderRadius: '4px',
                  cursor: 'pointer'
                }}
              >
                Deploy Winner
              </button>
            )}
          </div>
        </div>
      )}

      {/* Performance Visualization */}
      {selectedModels.length >= 2 && (
        <div style={{ marginBottom: '20px', padding: '16px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
          <h3 style={{ marginBottom: '12px' }}>Performance Comparison</h3>
          {renderPerformanceChart()}
        </div>
      )}

      {/* Model Performance Summary */}
      {selectedModels.length > 0 && (
        <div style={{ marginBottom: '20px', padding: '16px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
          <h3 style={{ marginBottom: '12px' }}>Performance Summary</h3>
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))', gap: '16px' }}>
            {selectedModels.map((modelName) => {
              const summary = getModelSummary(modelName);
              if (!summary) return null;

              return (
                <div
                  key={modelName}
                  style={{
                    padding: '16px',
                    backgroundColor: 'white',
                    borderRadius: '6px',
                    border: '1px solid #ddd'
                  }}
                >
                  <h4 style={{ marginBottom: '12px', color: '#333' }}>{modelName}</h4>
                  <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '8px' }}>
                    <PerformanceIndicator
                      value={summary.success_rate * 100}
                      label="Success Rate"
                      unit="%"
                      color="#28a745"
                    />
                    <PerformanceIndicator
                      value={summary.execution_time}
                      label="Avg Time"
                      unit="s"
                      color="#007bff"
                    />
                    <PerformanceIndicator
                      value={summary.confidence * 100}
                      label="Confidence"
                      unit="%"
                      color="#ffc107"
                    />
                    <PerformanceIndicator
                      value={summary.episodes}
                      label="Episodes"
                      color="#6c757d"
                    />
                  </div>
                </div>
              );
            })}
          </div>
        </div>
      )}

      {/* Detailed Comparison Results */}
      {comparisonResults.length > 0 && (
        <div style={{ marginBottom: '20px', padding: '16px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
          <h3 style={{ marginBottom: '12px' }}>Detailed Comparison Results</h3>
          <div style={{ overflow: 'auto' }}>
            <table style={{ width: '100%', borderCollapse: 'collapse' }}>
              <thead>
                <tr style={{ backgroundColor: '#e9ecef' }}>
                  <th style={{ padding: '12px', textAlign: 'left', border: '1px solid #ddd' }}>Model</th>
                  <th style={{ padding: '12px', textAlign: 'left', border: '1px solid #ddd' }}>Success Rate</th>
                  <th style={{ padding: '12px', textAlign: 'left', border: '1px solid #ddd' }}>Avg Time</th>
                  <th style={{ padding: '12px', textAlign: 'left', border: '1px solid #ddd' }}>Consistency</th>
                  <th style={{ padding: '12px', textAlign: 'left', border: '1px solid #ddd' }}>Improvement</th>
                  <th style={{ padding: '12px', textAlign: 'left', border: '1px solid #ddd' }}>Significance</th>
                </tr>
              </thead>
              <tbody>
                {comparisonResults.map((result) => (
                  <tr key={result.model_name}>
                    <td style={{ padding: '12px', border: '1px solid #ddd', fontWeight: 'bold' }}>
                      {result.model_name}
                    </td>
                    <td style={{ padding: '12px', border: '1px solid #ddd' }}>
                      {(result.metrics.success_rate * 100).toFixed(1)}%
                    </td>
                    <td style={{ padding: '12px', border: '1px solid #ddd' }}>
                      {result.metrics.avg_execution_time.toFixed(2)}s
                    </td>
                    <td style={{ padding: '12px', border: '1px solid #ddd' }}>
                      {(result.metrics.consistency_score * 100).toFixed(1)}%
                    </td>
                    <td style={{ padding: '12px', border: '1px solid #ddd' }}>
                      {result.metrics.improvement_over_baseline > 0 ? '+' : ''}
                      {(result.metrics.improvement_over_baseline * 100).toFixed(1)}%
                    </td>
                    <td style={{ padding: '12px', border: '1px solid #ddd' }}>
                      {result.statistical_significance ? '✅ Yes' : '❌ No'}
                      <br />
                      <small>n={result.sample_size}</small>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </div>
      )}
    </div>
  );
}

export function initModelComparisonPanel(context: PanelExtensionContext): void {
  ReactDOM.render(<ModelComparisonPanel context={context} />, context.panelElement);
}