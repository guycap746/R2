import { PanelExtensionContext, RenderState, Topic, MessageEvent } from "@foxglove/studio";
import { useEffect, useLayoutEffect, useState, useCallback } from "react";
import ReactDOM from "react-dom";

// Define message types for dataset data
type DatasetInfo = {
  name: string;
  total_episodes: number;
  total_frames: number;
  duration_hours: number;
  data_quality_score: number;
  creation_date: string;
  storage_size_mb: number;
};

type EpisodeInfo = {
  episode_id: number;
  duration_seconds: number;
  frame_count: number;
  success: boolean;
  quality_score: number;
  grasp_attempts: number;
  camera_data_available: boolean;
  joint_data_available: boolean;
};

type FrameData = {
  episode_id: number;
  frame_id: number;
  timestamp: number;
  joint_positions: number[];
  rgb_image_path: string;
  depth_image_path: string;
  grasp_pose: {
    position: number[];
    orientation: number[];
    confidence: number;
  } | null;
};

function DatasetBrowserPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [datasets, setDatasets] = useState<DatasetInfo[]>([]);
  const [selectedDataset, setSelectedDataset] = useState<string | null>(null);
  const [episodes, setEpisodes] = useState<EpisodeInfo[]>([]);
  const [selectedEpisode, setSelectedEpisode] = useState<number | null>(null);
  const [currentFrame, setCurrentFrame] = useState<FrameData | null>(null);
  const [isPlaying, setIsPlaying] = useState(false);
  const [playbackSpeed, setPlaybackSpeed] = useState(1);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Subscribe to dataset topics
  useLayoutEffect(() => {
    context.subscribe([
      { topic: "/lerobot/dataset_list" },
      { topic: "/lerobot/episode_list" },
      { topic: "/lerobot/frame_data" },
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
            case "/lerobot/dataset_list":
              setDatasets(msg.message.datasets || []);
              break;
            case "/lerobot/episode_list":
              if (msg.message.dataset_name === selectedDataset) {
                setEpisodes(msg.message.episodes || []);
              }
              break;
            case "/lerobot/frame_data":
              if (msg.message.episode_id === selectedEpisode) {
                setCurrentFrame(msg.message);
              }
              break;
          }
        }
      }
    };
  }, [context, selectedDataset, selectedEpisode]);

  // Dataset management functions
  const loadDataset = useCallback((datasetName: string) => {
    context.callService?.("/lerobot/load_dataset", { dataset_name: datasetName });
    setSelectedDataset(datasetName);
    setSelectedEpisode(null);
    setCurrentFrame(null);
  }, [context]);

  const loadEpisode = useCallback((episodeId: number) => {
    context.callService?.("/lerobot/load_episode", { 
      dataset_name: selectedDataset,
      episode_id: episodeId 
    });
    setSelectedEpisode(episodeId);
    setCurrentFrame(null);
  }, [context, selectedDataset]);

  const seekToFrame = useCallback((frameId: number) => {
    context.callService?.("/lerobot/seek_frame", { 
      episode_id: selectedEpisode,
      frame_id: frameId 
    });
  }, [context, selectedEpisode]);

  const togglePlayback = useCallback(() => {
    if (isPlaying) {
      context.callService?.("/lerobot/pause_playback", {});
    } else {
      context.callService?.("/lerobot/start_playback", { 
        speed: playbackSpeed 
      });
    }
    setIsPlaying(!isPlaying);
  }, [context, isPlaying, playbackSpeed]);

  const exportDataset = useCallback((format: string) => {
    context.callService?.("/lerobot/export_dataset", { 
      dataset_name: selectedDataset,
      format: format 
    });
  }, [context, selectedDataset]);

  // Quality indicator component
  const QualityIndicator = ({ score }: { score: number }) => {
    const getColor = (score: number) => {
      if (score >= 0.8) return '#28a745';
      if (score >= 0.6) return '#ffc107';
      return '#dc3545';
    };

    return (
      <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
        <div style={{
          width: '60px',
          height: '8px',
          backgroundColor: '#e9ecef',
          borderRadius: '4px',
          overflow: 'hidden'
        }}>
          <div style={{
            width: `${score * 100}%`,
            height: '100%',
            backgroundColor: getColor(score)
          }} />
        </div>
        <span style={{ fontSize: '12px', color: '#6c757d' }}>
          {(score * 100).toFixed(1)}%
        </span>
      </div>
    );
  };

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div style={{ padding: '16px', fontFamily: 'Arial, sans-serif', height: '100%', overflow: 'auto' }}>
      <h2 style={{ marginBottom: '20px', color: '#333' }}>📊 Dataset Browser</h2>

      {/* Dataset Selection */}
      <div style={{ marginBottom: '20px', padding: '16px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
        <h3 style={{ marginBottom: '12px' }}>Available Datasets</h3>
        {datasets.length > 0 ? (
          <div style={{ display: 'grid', gap: '12px' }}>
            {datasets.map((dataset) => (
              <div 
                key={dataset.name}
                onClick={() => loadDataset(dataset.name)}
                style={{
                  padding: '12px',
                  backgroundColor: selectedDataset === dataset.name ? '#e3f2fd' : 'white',
                  border: `2px solid ${selectedDataset === dataset.name ? '#2196f3' : '#ddd'}`,
                  borderRadius: '6px',
                  cursor: 'pointer',
                  transition: 'all 0.2s ease'
                }}
              >
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                  <div>
                    <div style={{ fontWeight: 'bold', marginBottom: '4px' }}>{dataset.name}</div>
                    <div style={{ fontSize: '14px', color: '#6c757d' }}>
                      {dataset.total_episodes} episodes • {dataset.total_frames} frames • {dataset.duration_hours.toFixed(1)}h
                    </div>
                    <div style={{ fontSize: '12px', color: '#6c757d', marginTop: '4px' }}>
                      Created: {dataset.creation_date} • {dataset.storage_size_mb.toFixed(1)} MB
                    </div>
                  </div>
                  <div>
                    <QualityIndicator score={dataset.data_quality_score} />
                  </div>
                </div>
              </div>
            ))}
          </div>
        ) : (
          <div style={{ color: '#6c757d' }}>No datasets available</div>
        )}
      </div>

      {/* Episode Browser */}
      {selectedDataset && (
        <div style={{ marginBottom: '20px', padding: '16px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
          <h3 style={{ marginBottom: '12px' }}>Episodes in {selectedDataset}</h3>
          {episodes.length > 0 ? (
            <div style={{ maxHeight: '200px', overflow: 'auto' }}>
              <table style={{ width: '100%', borderCollapse: 'collapse' }}>
                <thead>
                  <tr style={{ backgroundColor: '#e9ecef' }}>
                    <th style={{ padding: '8px', textAlign: 'left', border: '1px solid #ddd' }}>Episode</th>
                    <th style={{ padding: '8px', textAlign: 'left', border: '1px solid #ddd' }}>Duration</th>
                    <th style={{ padding: '8px', textAlign: 'left', border: '1px solid #ddd' }}>Frames</th>
                    <th style={{ padding: '8px', textAlign: 'left', border: '1px solid #ddd' }}>Success</th>
                    <th style={{ padding: '8px', textAlign: 'left', border: '1px solid #ddd' }}>Quality</th>
                    <th style={{ padding: '8px', textAlign: 'left', border: '1px solid #ddd' }}>Grasps</th>
                  </tr>
                </thead>
                <tbody>
                  {episodes.map((episode) => (
                    <tr 
                      key={episode.episode_id}
                      onClick={() => loadEpisode(episode.episode_id)}
                      style={{
                        backgroundColor: selectedEpisode === episode.episode_id ? '#e3f2fd' : 'white',
                        cursor: 'pointer'
                      }}
                    >
                      <td style={{ padding: '8px', border: '1px solid #ddd' }}>{episode.episode_id}</td>
                      <td style={{ padding: '8px', border: '1px solid #ddd' }}>{episode.duration_seconds.toFixed(1)}s</td>
                      <td style={{ padding: '8px', border: '1px solid #ddd' }}>{episode.frame_count}</td>
                      <td style={{ padding: '8px', border: '1px solid #ddd' }}>
                        {episode.success ? '✅' : '❌'}
                      </td>
                      <td style={{ padding: '8px', border: '1px solid #ddd' }}>
                        <QualityIndicator score={episode.quality_score} />
                      </td>
                      <td style={{ padding: '8px', border: '1px solid #ddd' }}>{episode.grasp_attempts}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          ) : (
            <div style={{ color: '#6c757d' }}>No episodes available</div>
          )}
        </div>
      )}

      {/* Playback Controls */}
      {selectedEpisode && (
        <div style={{ marginBottom: '20px', padding: '16px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
          <h3 style={{ marginBottom: '12px' }}>Playback Controls</h3>
          <div style={{ display: 'flex', alignItems: 'center', gap: '12px', marginBottom: '16px' }}>
            <button 
              onClick={togglePlayback}
              style={{ 
                padding: '8px 16px', 
                backgroundColor: isPlaying ? '#dc3545' : '#28a745', 
                color: 'white', 
                border: 'none', 
                borderRadius: '4px',
                cursor: 'pointer'
              }}
            >
              {isPlaying ? '⏸️ Pause' : '▶️ Play'}
            </button>
            
            <label style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
              Speed:
              <select 
                value={playbackSpeed} 
                onChange={(e) => setPlaybackSpeed(Number(e.target.value))}
                style={{ padding: '4px', borderRadius: '4px', border: '1px solid #ddd' }}
              >
                <option value={0.25}>0.25x</option>
                <option value={0.5}>0.5x</option>
                <option value={1}>1x</option>
                <option value={2}>2x</option>
                <option value={4}>4x</option>
              </select>
            </label>
          </div>

          {currentFrame && (
            <div style={{ marginBottom: '16px' }}>
              <div style={{ marginBottom: '8px' }}>
                <strong>Frame:</strong> {currentFrame.frame_id} / {episodes.find(e => e.episode_id === selectedEpisode)?.frame_count || 0}
              </div>
              <div style={{ marginBottom: '8px' }}>
                <strong>Timestamp:</strong> {(currentFrame.timestamp / 1000).toFixed(3)}s
              </div>
              {currentFrame.grasp_pose && (
                <div style={{ marginBottom: '8px' }}>
                  <strong>Grasp Confidence:</strong> {(currentFrame.grasp_pose.confidence * 100).toFixed(1)}%
                </div>
              )}
            </div>
          )}
        </div>
      )}

      {/* Export Controls */}
      {selectedDataset && (
        <div style={{ padding: '16px', backgroundColor: '#f8f9fa', borderRadius: '8px' }}>
          <h3 style={{ marginBottom: '12px' }}>Export Dataset</h3>
          <div style={{ display: 'flex', gap: '8px' }}>
            <button 
              onClick={() => exportDataset('lerobot')}
              style={{ 
                padding: '8px 16px', 
                backgroundColor: '#007bff', 
                color: 'white', 
                border: 'none', 
                borderRadius: '4px',
                cursor: 'pointer'
              }}
            >
              📦 LeRobot Format
            </button>
            <button 
              onClick={() => exportDataset('hdf5')}
              style={{ 
                padding: '8px 16px', 
                backgroundColor: '#17a2b8', 
                color: 'white', 
                border: 'none', 
                borderRadius: '4px',
                cursor: 'pointer'
              }}
            >
              📊 HDF5 Format
            </button>
            <button 
              onClick={() => exportDataset('rosbag')}
              style={{ 
                padding: '8px 16px', 
                backgroundColor: '#6f42c1', 
                color: 'white', 
                border: 'none', 
                borderRadius: '4px',
                cursor: 'pointer'
              }}
            >
              🎒 ROS Bag
            </button>
          </div>
        </div>
      )}
    </div>
  );
}

export function initDatasetBrowserPanel(context: PanelExtensionContext): void {
  ReactDOM.render(<DatasetBrowserPanel context={context} />, context.panelElement);
}