#!/usr/bin/env python3
"""
LeRobot Hybrid Visualizer

This module provides real-time visualization of the LeRobot-AnyGrasp hybrid system,
including grasp candidates, confidence scores, and performance metrics.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import json
from typing import Dict, List, Optional, Any, Tuple
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from threading import Thread, Lock
import queue

# ROS2 message types
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

class GraspVisualization:
    """Grasp candidate visualization"""
    
    def __init__(self):
        self.grasp_candidates = []
        self.selected_grasp = None
        self.confidence_scores = []
        self.hybrid_scores = []
        
    def update_candidates(self, candidates: List[Dict]):
        """Update grasp candidates"""
        self.grasp_candidates = candidates
        
    def update_scores(self, confidence_scores: List[float], hybrid_scores: List[float]):
        """Update scoring information"""
        self.confidence_scores = confidence_scores
        self.hybrid_scores = hybrid_scores
        
    def set_selected_grasp(self, grasp_idx: int):
        """Set selected grasp"""
        if 0 <= grasp_idx < len(self.grasp_candidates):
            self.selected_grasp = grasp_idx

class PerformanceVisualization:
    """Performance metrics visualization"""
    
    def __init__(self, max_points: int = 100):
        self.max_points = max_points
        self.timestamps = []
        self.success_rates = []
        self.execution_times = []
        self.confidence_scores = []
        
    def add_data_point(self, timestamp: float, success_rate: float, 
                      execution_time: float, confidence: float):
        """Add performance data point"""
        self.timestamps.append(timestamp)
        self.success_rates.append(success_rate)
        self.execution_times.append(execution_time)
        self.confidence_scores.append(confidence)
        
        # Keep only recent points
        if len(self.timestamps) > self.max_points:
            self.timestamps = self.timestamps[-self.max_points:]
            self.success_rates = self.success_rates[-self.max_points:]
            self.execution_times = self.execution_times[-self.max_points:]
            self.confidence_scores = self.confidence_scores[-self.max_points:]

class LeRobotHybridVisualizer(Node):
    """Real-time visualization for hybrid system"""
    
    def __init__(self):
        super().__init__('lerobot_hybrid_visualizer')
        
        # Parameters
        self.declare_parameter('show_grasp_candidates', True)
        self.declare_parameter('show_confidence_scores', True)
        self.declare_parameter('show_hybrid_scores', True)
        self.declare_parameter('show_performance_plots', True)
        self.declare_parameter('visualization_fps', 10)
        self.declare_parameter('image_scale', 1.0)
        
        # Get parameters
        self.show_grasp_candidates = self.get_parameter('show_grasp_candidates').get_parameter_value().bool_value
        self.show_confidence_scores = self.get_parameter('show_confidence_scores').get_parameter_value().bool_value
        self.show_hybrid_scores = self.get_parameter('show_hybrid_scores').get_parameter_value().bool_value
        self.show_performance_plots = self.get_parameter('show_performance_plots').get_parameter_value().bool_value
        self.visualization_fps = self.get_parameter('visualization_fps').get_parameter_value().integer_value
        self.image_scale = self.get_parameter('image_scale').get_parameter_value().double_value
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        self.data_lock = Lock()
        
        # Visualization data
        self.current_image = None
        self.grasp_viz = GraspVisualization()
        self.performance_viz = PerformanceVisualization()
        
        # UI state
        self.display_mode = 'grasp_candidates'  # grasp_candidates, performance, hybrid_scores
        self.visualization_active = True
        
        # Publishers
        self.viz_image_pub = self.create_publisher(Image, '/lerobot/hybrid_visualization', 10)
        self.status_pub = self.create_publisher(String, '/lerobot/visualizer_status', 10)
        
        # Subscribers
        self.rgb_image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.grasp_candidates_sub = self.create_subscription(
            String, '/lerobot/grasp_candidates_viz', self.grasp_candidates_callback, 10)
        self.performance_sub = self.create_subscription(
            String, '/lerobot/performance_analytics', self.performance_callback, 10)
        self.hybrid_status_sub = self.create_subscription(
            String, '/lerobot/hybrid_status', self.hybrid_status_callback, 10)
        
        # Control subscribers
        self.viz_control_sub = self.create_subscription(
            String, '/lerobot/visualization_control', self.visualization_control_callback, 10)
        
        # Timers
        self.visualization_timer = self.create_timer(1.0 / self.visualization_fps, self.update_visualization)
        
        # Performance plotting (matplotlib)
        if self.show_performance_plots:
            self.setup_performance_plots()
        
        self.get_logger().info('LeRobot Hybrid Visualizer initialized')
        self.publish_status('Visualizer ready')
    
    def image_callback(self, msg):
        """Handle camera image updates"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.data_lock:
                self.current_image = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')
    
    def grasp_candidates_callback(self, msg):
        """Handle grasp candidates visualization data"""
        try:
            data = json.loads(msg.data)
            candidates = data.get('candidates', [])
            confidence_scores = data.get('confidence_scores', [])
            hybrid_scores = data.get('hybrid_scores', [])
            selected_idx = data.get('selected_grasp', -1)
            
            with self.data_lock:
                self.grasp_viz.update_candidates(candidates)
                self.grasp_viz.update_scores(confidence_scores, hybrid_scores)
                if selected_idx >= 0:
                    self.grasp_viz.set_selected_grasp(selected_idx)
                    
        except Exception as e:
            self.get_logger().error(f'Failed to process grasp candidates: {e}')
    
    def performance_callback(self, msg):
        """Handle performance analytics data"""
        try:
            data = json.loads(msg.data)
            current_stats = data.get('current_statistics', {})
            
            if current_stats:
                import time
                timestamp = time.time()
                success_rate = current_stats.get('success_rate', 0.0)
                avg_execution_time = current_stats.get('avg_execution_time', 0.0)
                avg_confidence = current_stats.get('avg_confidence', 0.5)
                
                with self.data_lock:
                    self.performance_viz.add_data_point(
                        timestamp, success_rate, avg_execution_time, avg_confidence)
                    
        except Exception as e:
            self.get_logger().error(f'Failed to process performance data: {e}')
    
    def hybrid_status_callback(self, msg):
        """Handle hybrid system status updates"""
        status = msg.data
        # Could be used to show system state in visualization
        pass
    
    def visualization_control_callback(self, msg):
        """Handle visualization control commands"""
        try:
            command = json.loads(msg.data)
            
            if 'display_mode' in command:
                self.display_mode = command['display_mode']
                
            if 'active' in command:
                self.visualization_active = command['active']
                
        except Exception as e:
            self.get_logger().error(f'Failed to process visualization control: {e}')
    
    def update_visualization(self):
        """Update and publish visualization"""
        if not self.visualization_active:
            return
        
        try:
            with self.data_lock:
                if self.current_image is None:
                    return
                
                # Create visualization based on current mode
                if self.display_mode == 'grasp_candidates':
                    viz_image = self.create_grasp_candidates_visualization()
                elif self.display_mode == 'performance':
                    viz_image = self.create_performance_visualization()
                elif self.display_mode == 'hybrid_scores':
                    viz_image = self.create_hybrid_scores_visualization()
                else:
                    viz_image = self.current_image.copy()
                
                # Add UI overlay
                viz_image = self.add_ui_overlay(viz_image)
                
                # Publish visualization
                self.publish_visualization_image(viz_image)
                
        except Exception as e:
            self.get_logger().error(f'Visualization update failed: {e}')
    
    def create_grasp_candidates_visualization(self) -> np.ndarray:
        """Create grasp candidates visualization"""
        viz_image = self.current_image.copy()
        
        if not self.show_grasp_candidates:
            return viz_image
        
        # Draw grasp candidates
        for i, candidate in enumerate(self.grasp_viz.grasp_candidates):
            # Extract pose information
            pose = candidate.get('pose', {})
            position = pose.get('position', {})
            x = int(position.get('x', 0) * viz_image.shape[1])
            y = int(position.get('y', 0) * viz_image.shape[0])
            
            # Color based on selection and scores
            if i == self.grasp_viz.selected_grasp:
                color = (0, 255, 0)  # Green for selected
                thickness = 3
            else:
                color = (0, 0, 255)  # Red for candidates
                thickness = 2
            
            # Draw grasp point
            cv2.circle(viz_image, (x, y), 8, color, thickness)
            
            # Draw approach vector
            approach = candidate.get('approach_vector', [0, 0, -1])
            end_x = int(x + approach[0] * 30)
            end_y = int(y + approach[1] * 30)
            cv2.arrowedLine(viz_image, (x, y), (end_x, end_y), color, thickness)
            
            # Add score text
            if i < len(self.grasp_viz.confidence_scores):
                confidence = self.grasp_viz.confidence_scores[i]
                text = f'{confidence:.2f}'
                cv2.putText(viz_image, text, (x + 10, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return viz_image
    
    def create_performance_visualization(self) -> np.ndarray:
        """Create performance metrics visualization"""
        viz_image = self.current_image.copy()
        
        # Create performance plot overlay
        if len(self.performance_viz.success_rates) > 5:
            # Create small performance plot
            plot_height = 150
            plot_width = 300
            plot_img = np.zeros((plot_height, plot_width, 3), dtype=np.uint8)
            
            # Plot success rate
            success_rates = self.performance_viz.success_rates[-20:]  # Last 20 points
            if len(success_rates) > 1:
                points = []
                for i, rate in enumerate(success_rates):
                    x = int((i / (len(success_rates) - 1)) * (plot_width - 20)) + 10
                    y = int((1 - rate) * (plot_height - 20)) + 10
                    points.append((x, y))
                
                # Draw line plot
                for i in range(len(points) - 1):
                    cv2.line(plot_img, points[i], points[i + 1], (0, 255, 0), 2)
            
            # Add plot to image
            overlay_y = 50
            overlay_x = viz_image.shape[1] - plot_width - 50
            viz_image[overlay_y:overlay_y + plot_height, 
                     overlay_x:overlay_x + plot_width] = plot_img
        
        return viz_image
    
    def create_hybrid_scores_visualization(self) -> np.ndarray:
        """Create hybrid scoring visualization"""
        viz_image = self.current_image.copy()
        
        # Add hybrid scoring information
        if self.grasp_viz.hybrid_scores:
            y_offset = 30
            for i, (conf, hybrid) in enumerate(zip(
                self.grasp_viz.confidence_scores, self.grasp_viz.hybrid_scores)):
                
                text = f'Candidate {i}: Conf={conf:.2f}, Hybrid={hybrid:.2f}'
                color = (0, 255, 0) if i == self.grasp_viz.selected_grasp else (255, 255, 255)
                
                cv2.putText(viz_image, text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                y_offset += 25
        
        return viz_image
    
    def add_ui_overlay(self, image: np.ndarray) -> np.ndarray:
        """Add UI overlay with controls and information"""
        overlay = image.copy()
        
        # Add display mode indicator
        mode_text = f'Mode: {self.display_mode}'
        cv2.putText(overlay, mode_text, (10, image.shape[0] - 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Add instructions
        instructions = [
            'Press: G=Grasps, P=Performance, H=Hybrid Scores',
            'ESC=Exit, SPACE=Toggle Active'
        ]
        
        for i, instruction in enumerate(instructions):
            cv2.putText(overlay, instruction, (10, image.shape[0] - 30 + i * 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # Add frame counter and FPS
        import time
        fps_text = f'FPS: {self.visualization_fps}'
        cv2.putText(overlay, fps_text, (image.shape[1] - 100, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return overlay
    
    def publish_visualization_image(self, image: np.ndarray):
        """Publish visualization image"""
        try:
            # Scale image if needed
            if self.image_scale != 1.0:
                new_width = int(image.shape[1] * self.image_scale)
                new_height = int(image.shape[0] * self.image_scale)
                image = cv2.resize(image, (new_width, new_height))
            
            # Convert to ROS image message
            ros_image = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            
            self.viz_image_pub.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish visualization: {e}')
    
    def setup_performance_plots(self):
        """Setup matplotlib performance plots"""
        try:
            import matplotlib
            matplotlib.use('Agg')  # Non-interactive backend
            
            self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
            self.fig.suptitle('LeRobot-AnyGrasp Hybrid Performance')
            
            # Setup plot formatting
            self.axes[0, 0].set_title('Success Rate')
            self.axes[0, 0].set_ylabel('Success Rate')
            
            self.axes[0, 1].set_title('Execution Time')
            self.axes[0, 1].set_ylabel('Time (s)')
            
            self.axes[1, 0].set_title('Confidence Scores')
            self.axes[1, 0].set_ylabel('Confidence')
            
            self.axes[1, 1].set_title('System Status')
            
            plt.tight_layout()
            
        except ImportError:
            self.get_logger().warning('Matplotlib not available for performance plots')
            self.show_performance_plots = False
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[LEROBOT_VISUALIZER] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        visualizer = LeRobotHybridVisualizer()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Hybrid visualizer error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()