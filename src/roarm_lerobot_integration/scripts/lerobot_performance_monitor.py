#!/usr/bin/env python3
"""
LeRobot Performance Monitor

This module monitors and analyzes performance of LeRobot-AnyGrasp hybrid system,
providing real-time analytics and adaptive parameter tuning.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import json
import time
from typing import Dict, List, Optional, Any
from pathlib import Path
import matplotlib.pyplot as plt
from datetime import datetime, timedelta
from collections import deque

# ROS2 message types
from std_msgs.msg import String, Bool, Float32

class PerformanceMetrics:
    """Performance metrics tracker"""
    
    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        self.execution_times = deque(maxlen=window_size)
        self.success_rates = deque(maxlen=window_size)
        self.confidence_scores = deque(maxlen=window_size)
        self.hybrid_scores = deque(maxlen=window_size)
        self.method_usage = {
            'lerobot_only': 0,
            'anygrasp_only': 0,
            'hybrid': 0
        }
        
    def add_execution(self, execution_time: float, success: bool, 
                     confidence: float, hybrid_score: float, method: str):
        """Add execution data"""
        self.execution_times.append(execution_time)
        self.success_rates.append(1.0 if success else 0.0)
        self.confidence_scores.append(confidence)
        self.hybrid_scores.append(hybrid_score)
        
        if method in self.method_usage:
            self.method_usage[method] += 1
    
    def get_statistics(self) -> Dict:
        """Get current performance statistics"""
        if not self.execution_times:
            return {}
        
        return {
            'avg_execution_time': np.mean(self.execution_times),
            'std_execution_time': np.std(self.execution_times),
            'success_rate': np.mean(self.success_rates),
            'avg_confidence': np.mean(self.confidence_scores),
            'avg_hybrid_score': np.mean(self.hybrid_scores),
            'total_executions': len(self.execution_times),
            'method_distribution': self.method_usage.copy()
        }

class LeRobotPerformanceMonitor(Node):
    """Real-time performance monitoring for hybrid system"""
    
    def __init__(self):
        super().__init__('lerobot_performance_monitor')
        
        # Parameters
        self.declare_parameter('monitor_hybrid', True)
        self.declare_parameter('log_performance', True)
        self.declare_parameter('adaptive_tuning', True)
        self.declare_parameter('performance_window', 100)
        self.declare_parameter('tuning_interval', 300.0)  # 5 minutes
        self.declare_parameter('log_dir', '/tmp/lerobot_performance_logs')
        
        # Get parameters
        self.monitor_hybrid = self.get_parameter('monitor_hybrid').get_parameter_value().bool_value
        self.log_performance = self.get_parameter('log_performance').get_parameter_value().bool_value
        self.adaptive_tuning = self.get_parameter('adaptive_tuning').get_parameter_value().bool_value
        self.performance_window = self.get_parameter('performance_window').get_parameter_value().integer_value
        self.tuning_interval = self.get_parameter('tuning_interval').get_parameter_value().double_value
        self.log_dir = Path(self.get_parameter('log_dir').get_parameter_value().string_value)
        
        # Create log directory
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # Performance tracking
        self.metrics = PerformanceMetrics(self.performance_window)
        self.current_parameters = {
            'confidence_threshold': 0.7,
            'anygrasp_weight': 0.4,
            'lerobot_weight': 0.6,
            'hybrid_mode': 'hybrid_validate'
        }
        
        # Real-time monitoring data
        self.recent_executions = []
        self.performance_trends = {
            'timestamps': deque(maxlen=1000),
            'success_rates': deque(maxlen=1000),
            'execution_times': deque(maxlen=1000)
        }
        
        # Publishers
        self.analytics_pub = self.create_publisher(String, '/lerobot/performance_analytics', 10)
        self.recommendations_pub = self.create_publisher(String, '/lerobot/performance_recommendations', 10)
        self.tuning_pub = self.create_publisher(String, '/lerobot/adaptive_tuning', 10)
        
        # Subscribers
        self.hybrid_performance_sub = self.create_subscription(
            String, '/lerobot/hybrid_performance', self.hybrid_performance_callback, 10)
        self.execution_result_sub = self.create_subscription(
            String, '/lerobot/execution_result', self.execution_result_callback, 10)
        self.system_status_sub = self.create_subscription(
            String, '/lerobot/hybrid_status', self.system_status_callback, 10)
        
        # Timers
        self.analytics_timer = self.create_timer(30.0, self.publish_analytics)
        if self.adaptive_tuning:
            self.tuning_timer = self.create_timer(self.tuning_interval, self.adaptive_parameter_tuning)
        
        # Performance log file
        self.performance_log_file = None
        if self.log_performance:
            log_filename = f"performance_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            self.performance_log_file = self.log_dir / log_filename
        
        self.get_logger().info('LeRobot Performance Monitor initialized')
    
    def hybrid_performance_callback(self, msg):
        """Handle hybrid performance updates"""
        try:
            performance_data = json.loads(msg.data)
            self.process_performance_data(performance_data)
        except Exception as e:
            self.get_logger().error(f'Failed to process hybrid performance data: {e}')
    
    def execution_result_callback(self, msg):
        """Handle individual execution results"""
        try:
            result_data = json.loads(msg.data)
            self.process_execution_result(result_data)
        except Exception as e:
            self.get_logger().error(f'Failed to process execution result: {e}')
    
    def system_status_callback(self, msg):
        """Handle system status updates"""
        # Monitor system health and performance
        status = msg.data
        if 'failed' in status.lower() or 'error' in status.lower():
            self.record_system_issue(status)
    
    def process_performance_data(self, data: Dict):
        """Process hybrid performance data"""
        try:
            performance_by_method = data.get('performance_by_method', {})
            
            # Update method usage statistics
            for method, stats in performance_by_method.items():
                if method in self.metrics.method_usage:
                    self.metrics.method_usage[method] = stats.get('attempts', 0)
            
            # Add to trends
            timestamp = time.time()
            self.performance_trends['timestamps'].append(timestamp)
            
            # Calculate overall success rate
            total_successes = sum(stats.get('successes', 0) for stats in performance_by_method.values())
            total_attempts = sum(stats.get('attempts', 0) for stats in performance_by_method.values())
            success_rate = total_successes / total_attempts if total_attempts > 0 else 0.0
            
            self.performance_trends['success_rates'].append(success_rate)
            
        except Exception as e:
            self.get_logger().error(f'Error processing performance data: {e}')
    
    def process_execution_result(self, result: Dict):
        """Process individual execution result"""
        try:
            execution_time = result.get('execution_time', 0.0)
            success = result.get('success', False)
            confidence = result.get('confidence', 0.5)
            hybrid_score = result.get('hybrid_score', 0.5)
            method = result.get('method', 'unknown')
            
            # Add to metrics
            self.metrics.add_execution(execution_time, success, confidence, hybrid_score, method)
            
            # Add to recent executions
            self.recent_executions.append({
                'timestamp': time.time(),
                'execution_time': execution_time,
                'success': success,
                'confidence': confidence,
                'hybrid_score': hybrid_score,
                'method': method
            })
            
            # Keep only recent executions
            if len(self.recent_executions) > 200:
                self.recent_executions = self.recent_executions[-200:]
            
            # Log to file if enabled
            if self.log_performance and self.performance_log_file:
                self.log_execution_to_file(result)
                
        except Exception as e:
            self.get_logger().error(f'Error processing execution result: {e}')
    
    def log_execution_to_file(self, result: Dict):
        """Log execution result to file"""
        try:
            log_entry = {
                'timestamp': datetime.now().isoformat(),
                'execution_data': result,
                'system_parameters': self.current_parameters.copy()
            }
            
            # Append to log file
            with open(self.performance_log_file, 'a') as f:
                f.write(json.dumps(log_entry) + '\n')
                
        except Exception as e:
            self.get_logger().error(f'Failed to log to file: {e}')
    
    def publish_analytics(self):
        """Publish performance analytics"""
        try:
            # Get current statistics
            stats = self.metrics.get_statistics()
            
            if not stats:
                return
            
            # Add trend analysis
            analytics = {
                'current_statistics': stats,
                'trend_analysis': self.analyze_trends(),
                'recommendations': self.generate_recommendations(stats),
                'timestamp': datetime.now().isoformat()
            }
            
            # Publish analytics
            analytics_msg = String()
            analytics_msg.data = json.dumps(analytics, indent=2)
            self.analytics_pub.publish(analytics_msg)
            
            # Publish recommendations separately
            if analytics['recommendations']:
                rec_msg = String()
                rec_msg.data = json.dumps(analytics['recommendations'], indent=2)
                self.recommendations_pub.publish(rec_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish analytics: {e}')
    
    def analyze_trends(self) -> Dict:
        """Analyze performance trends"""
        try:
            if len(self.performance_trends['success_rates']) < 10:
                return {'status': 'insufficient_data'}
            
            success_rates = list(self.performance_trends['success_rates'])
            
            # Calculate trend direction
            recent_rates = success_rates[-10:]
            older_rates = success_rates[-20:-10] if len(success_rates) >= 20 else success_rates[:-10]
            
            recent_avg = np.mean(recent_rates)
            older_avg = np.mean(older_rates) if older_rates else recent_avg
            
            trend_direction = 'improving' if recent_avg > older_avg else 'declining' if recent_avg < older_avg else 'stable'
            trend_magnitude = abs(recent_avg - older_avg)
            
            return {
                'trend_direction': trend_direction,
                'trend_magnitude': trend_magnitude,
                'recent_success_rate': recent_avg,
                'previous_success_rate': older_avg,
                'volatility': np.std(recent_rates)
            }
            
        except Exception as e:
            self.get_logger().error(f'Trend analysis failed: {e}')
            return {'status': 'analysis_failed'}
    
    def generate_recommendations(self, stats: Dict) -> List[Dict]:
        """Generate performance recommendations"""
        recommendations = []
        
        try:
            success_rate = stats.get('success_rate', 0.0)
            avg_confidence = stats.get('avg_confidence', 0.5)
            method_dist = stats.get('method_distribution', {})
            
            # Success rate recommendations
            if success_rate < 0.6:
                recommendations.append({
                    'type': 'performance',
                    'priority': 'high',
                    'message': f'Low success rate ({success_rate:.2f}). Consider adjusting confidence threshold or hybrid weights.',
                    'suggested_action': 'lower_confidence_threshold'
                })
            
            # Confidence recommendations
            if avg_confidence < 0.5:
                recommendations.append({
                    'type': 'confidence',
                    'priority': 'medium',
                    'message': f'Low average confidence ({avg_confidence:.2f}). Consider retraining policy or adjusting hybrid mode.',
                    'suggested_action': 'retrain_policy'
                })
            
            # Method usage recommendations
            total_executions = sum(method_dist.values())
            if total_executions > 0:
                lerobot_ratio = method_dist.get('lerobot_only', 0) / total_executions
                anygrasp_ratio = method_dist.get('anygrasp_only', 0) / total_executions
                
                if lerobot_ratio > 0.8:
                    recommendations.append({
                        'type': 'method_balance',
                        'priority': 'low',
                        'message': 'Heavy reliance on LeRobot. Consider increasing AnyGrasp weight for better balance.',
                        'suggested_action': 'increase_anygrasp_weight'
                    })
                elif anygrasp_ratio > 0.8:
                    recommendations.append({
                        'type': 'method_balance',
                        'priority': 'low',
                        'message': 'Heavy reliance on AnyGrasp. Consider increasing LeRobot weight or improving policy.',
                        'suggested_action': 'increase_lerobot_weight'
                    })
            
            return recommendations
            
        except Exception as e:
            self.get_logger().error(f'Failed to generate recommendations: {e}')
            return []
    
    def adaptive_parameter_tuning(self):
        """Perform adaptive parameter tuning based on performance"""
        if not self.adaptive_tuning:
            return
        
        try:
            stats = self.metrics.get_statistics()
            if not stats or stats.get('total_executions', 0) < 20:
                return  # Need more data for tuning
            
            success_rate = stats.get('success_rate', 0.0)
            avg_confidence = stats.get('avg_confidence', 0.5)
            
            tuning_actions = []
            
            # Adjust confidence threshold based on performance
            if success_rate < 0.5 and self.current_parameters['confidence_threshold'] > 0.3:
                new_threshold = max(0.3, self.current_parameters['confidence_threshold'] - 0.05)
                self.current_parameters['confidence_threshold'] = new_threshold
                tuning_actions.append(f'Lowered confidence threshold to {new_threshold:.2f}')
            
            elif success_rate > 0.8 and avg_confidence > 0.7 and self.current_parameters['confidence_threshold'] < 0.9:
                new_threshold = min(0.9, self.current_parameters['confidence_threshold'] + 0.02)
                self.current_parameters['confidence_threshold'] = new_threshold
                tuning_actions.append(f'Raised confidence threshold to {new_threshold:.2f}')
            
            # Adjust hybrid weights based on method performance
            method_dist = stats.get('method_distribution', {})
            total = sum(method_dist.values())
            
            if total > 0:
                lerobot_success = method_dist.get('lerobot_only', 0) / max(1, total)
                anygrasp_success = method_dist.get('anygrasp_only', 0) / max(1, total)
                
                if lerobot_success > anygrasp_success * 1.5:
                    # LeRobot performing better, increase its weight
                    new_lerobot_weight = min(0.8, self.current_parameters['lerobot_weight'] + 0.05)
                    new_anygrasp_weight = 1.0 - new_lerobot_weight
                    self.current_parameters['lerobot_weight'] = new_lerobot_weight
                    self.current_parameters['anygrasp_weight'] = new_anygrasp_weight
                    tuning_actions.append(f'Increased LeRobot weight to {new_lerobot_weight:.2f}')
                
                elif anygrasp_success > lerobot_success * 1.5:
                    # AnyGrasp performing better, increase its weight
                    new_anygrasp_weight = min(0.8, self.current_parameters['anygrasp_weight'] + 0.05)
                    new_lerobot_weight = 1.0 - new_anygrasp_weight
                    self.current_parameters['anygrasp_weight'] = new_anygrasp_weight
                    self.current_parameters['lerobot_weight'] = new_lerobot_weight
                    tuning_actions.append(f'Increased AnyGrasp weight to {new_anygrasp_weight:.2f}')
            
            # Publish tuning actions
            if tuning_actions:
                tuning_data = {
                    'timestamp': datetime.now().isoformat(),
                    'actions': tuning_actions,
                    'new_parameters': self.current_parameters.copy(),
                    'performance_trigger': {
                        'success_rate': success_rate,
                        'avg_confidence': avg_confidence
                    }
                }
                
                tuning_msg = String()
                tuning_msg.data = json.dumps(tuning_data, indent=2)
                self.tuning_pub.publish(tuning_msg)
                
                self.get_logger().info(f'Adaptive tuning applied: {tuning_actions}')
            
        except Exception as e:
            self.get_logger().error(f'Adaptive tuning failed: {e}')
    
    def record_system_issue(self, status: str):
        """Record system issues for analysis"""
        issue_record = {
            'timestamp': datetime.now().isoformat(),
            'status': status,
            'current_parameters': self.current_parameters.copy()
        }
        
        # Log issue
        self.get_logger().warning(f'System issue recorded: {status}')


def main():
    rclpy.init()
    
    try:
        monitor = LeRobotPerformanceMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Performance monitor error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()