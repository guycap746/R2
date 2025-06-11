#!/usr/bin/env python3

"""
Training Workflow Manager for RoArm M3 + LeRobot Integration

This node manages automated training workflows, including:
- Data collection triggers
- Training pipeline automation
- Model evaluation and deployment
- Performance monitoring and adaptation
- A/B testing coordination
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import json
import time
import threading
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
from enum import Enum

# ROS2 message imports
from std_msgs.msg import String, Float32, Int32, Bool
from std_srvs.srv import Empty, SetBool
from geometry_msgs.msg import PoseStamped

# Custom message imports (these would need to be defined)
try:
    from roarm_lerobot_integration.srv import (
        StartTraining, StopTraining, LoadDataset, EvaluateModel,
        StartDataCollection, DeployModel
    )
    from roarm_lerobot_integration.msg import (
        TrainingStatus, TrainingMetrics, DatasetInfo, 
        WorkflowStatus, ModelPerformance, DataCollectionStatus
    )
    from roarm_anygrasp_integration.msg import GraspResult
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    print("Warning: Custom messages not available, using standard messages")


class WorkflowState(Enum):
    IDLE = "idle"
    DATA_COLLECTION = "data_collection"
    TRAINING = "training"
    EVALUATION = "evaluation"
    DEPLOYMENT = "deployment"
    A_B_TESTING = "a_b_testing"
    ERROR = "error"


@dataclass
class WorkflowConfig:
    """Configuration for training workflows"""
    # Data collection settings
    min_episodes_before_training: int = 50
    max_episodes_per_dataset: int = 1000
    data_quality_threshold: float = 0.7
    
    # Training settings
    auto_train_enabled: bool = True
    training_trigger_interval_hours: int = 24
    max_training_time_hours: int = 8
    early_stopping_patience: int = 10
    
    # Model evaluation settings
    evaluation_episodes: int = 20
    success_rate_threshold: float = 0.8
    performance_improvement_threshold: float = 0.05
    
    # Deployment settings
    auto_deploy_enabled: bool = False
    a_b_test_duration_hours: int = 48
    a_b_test_episode_threshold: int = 100
    
    # Performance monitoring
    performance_monitoring_interval_minutes: int = 30
    performance_degradation_threshold: float = 0.1


@dataclass
class ModelMetrics:
    """Model performance metrics"""
    success_rate: float = 0.0
    average_execution_time: float = 0.0
    grasp_confidence: float = 0.0
    episodes_tested: int = 0
    timestamp: datetime = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()


class TrainingWorkflowManager(Node):
    """Main training workflow management node"""
    
    def __init__(self):
        super().__init__('training_workflow_manager')
        
        # Initialize configuration
        self.config = WorkflowConfig()
        self.load_config()
        
        # Current state
        self.current_state = WorkflowState.IDLE
        self.active_workflows: Dict[str, Dict] = {}
        self.model_performance_history: Dict[str, List[ModelMetrics]] = {}
        
        # Workflow tracking
        self.last_training_time = datetime.now() - timedelta(days=1)
        self.current_training_id = None
        self.current_evaluation_id = None
        self.a_b_test_active = False
        
        # Threading
        self.workflow_thread = None
        self.monitoring_thread = None
        self.shutdown_event = threading.Event()
        
        self.get_logger().info("Initializing Training Workflow Manager...")
        
        # Initialize ROS2 interfaces
        self.setup_publishers_subscribers()
        self.setup_service_clients()
        self.setup_services()
        
        # Start background threads
        self.start_background_threads()
        
        self.get_logger().info("Training Workflow Manager initialized successfully")

    def load_config(self):
        """Load configuration from parameters"""
        self.declare_parameters(namespace='', parameters=[
            ('workflow.min_episodes_before_training', 50),
            ('workflow.auto_train_enabled', True),
            ('workflow.training_trigger_interval_hours', 24),
            ('workflow.auto_deploy_enabled', False),
            ('workflow.performance_monitoring_interval_minutes', 30),
        ])
        
        # Update config from parameters
        self.config.min_episodes_before_training = self.get_parameter(
            'workflow.min_episodes_before_training').value
        self.config.auto_train_enabled = self.get_parameter(
            'workflow.auto_train_enabled').value
        self.config.training_trigger_interval_hours = self.get_parameter(
            'workflow.training_trigger_interval_hours').value
        self.config.auto_deploy_enabled = self.get_parameter(
            'workflow.auto_deploy_enabled').value
        self.config.performance_monitoring_interval_minutes = self.get_parameter(
            'workflow.performance_monitoring_interval_minutes').value

    def setup_publishers_subscribers(self):
        """Set up ROS2 publishers and subscribers"""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Publishers
        self.workflow_status_pub = self.create_publisher(
            String, '/training_workflow/status', qos_profile)
        self.workflow_metrics_pub = self.create_publisher(
            String, '/training_workflow/metrics', qos_profile)
        self.workflow_recommendations_pub = self.create_publisher(
            String, '/training_workflow/recommendations', qos_profile)
        
        # Subscribers
        if CUSTOM_MSGS_AVAILABLE:
            self.training_status_sub = self.create_subscription(
                TrainingStatus, '/lerobot/trainer_status', 
                self.training_status_callback, 10)
            self.grasp_result_sub = self.create_subscription(
                GraspResult, '/anygrasp/grasp_result',
                self.grasp_result_callback, 10)
            self.data_collection_sub = self.create_subscription(
                DataCollectionStatus, '/lerobot/data_collection_status',
                self.data_collection_callback, 10)
        else:
            # Fallback to standard messages
            self.training_status_sub = self.create_subscription(
                String, '/lerobot/trainer_status', 
                self.training_status_callback_fallback, 10)

    def setup_service_clients(self):
        """Set up service clients for training operations"""
        if CUSTOM_MSGS_AVAILABLE:
            self.start_training_client = self.create_client(
                StartTraining, '/lerobot/start_training')
            self.stop_training_client = self.create_client(
                StopTraining, '/lerobot/stop_training')
            self.start_data_collection_client = self.create_client(
                StartDataCollection, '/lerobot/start_data_collection')
            self.evaluate_model_client = self.create_client(
                EvaluateModel, '/lerobot/evaluate_model')
            self.deploy_model_client = self.create_client(
                DeployModel, '/lerobot/deploy_model')
        else:
            # Fallback to standard services
            self.start_training_client = self.create_client(
                SetBool, '/lerobot/start_training')
            self.stop_training_client = self.create_client(
                SetBool, '/lerobot/stop_training')

    def setup_services(self):
        """Set up service servers for workflow control"""
        self.start_workflow_service = self.create_service(
            SetBool, '/training_workflow/start_auto_workflow', 
            self.start_auto_workflow_callback)
        self.stop_workflow_service = self.create_service(
            SetBool, '/training_workflow/stop_workflow',
            self.stop_workflow_callback)
        self.trigger_training_service = self.create_service(
            Empty, '/training_workflow/trigger_training',
            self.trigger_training_callback)
        self.get_status_service = self.create_service(
            Empty, '/training_workflow/get_status',
            self.get_status_callback)

    def start_background_threads(self):
        """Start background monitoring threads"""
        self.workflow_thread = threading.Thread(
            target=self.workflow_monitor_loop, daemon=True)
        self.monitoring_thread = threading.Thread(
            target=self.performance_monitor_loop, daemon=True)
        
        self.workflow_thread.start()
        self.monitoring_thread.start()

    # =============================================
    # CALLBACK METHODS
    # =============================================
    
    def training_status_callback(self, msg):
        """Handle training status updates"""
        if hasattr(msg, 'is_training'):
            if msg.is_training and self.current_state != WorkflowState.TRAINING:
                self.current_state = WorkflowState.TRAINING
                self.current_training_id = f"training_{int(time.time())}"
                self.get_logger().info("Training started - workflow state updated")
            elif not msg.is_training and self.current_state == WorkflowState.TRAINING:
                self.current_state = WorkflowState.IDLE
                self.last_training_time = datetime.now()
                self.get_logger().info("Training completed - workflow state updated")
                
                # Trigger evaluation if configured
                if self.config.auto_deploy_enabled:
                    self.schedule_model_evaluation()

    def training_status_callback_fallback(self, msg):
        """Fallback training status callback for standard messages"""
        try:
            data = json.loads(msg.data)
            if data.get('is_training', False):
                self.current_state = WorkflowState.TRAINING
            else:
                self.current_state = WorkflowState.IDLE
        except json.JSONDecodeError:
            pass

    def grasp_result_callback(self, msg):
        """Handle grasp execution results for performance monitoring"""
        if hasattr(msg, 'success') and hasattr(msg, 'execution_time'):
            # Update performance metrics
            model_name = getattr(msg, 'model_name', 'current_model')
            
            if model_name not in self.model_performance_history:
                self.model_performance_history[model_name] = []
            
            # Calculate rolling metrics
            recent_results = self.model_performance_history[model_name][-50:]  # Last 50 results
            success_rate = sum(1 for r in recent_results if r.success_rate > 0) / max(len(recent_results), 1)
            avg_time = sum(r.average_execution_time for r in recent_results) / max(len(recent_results), 1)
            
            metrics = ModelMetrics(
                success_rate=float(msg.success),
                average_execution_time=msg.execution_time,
                grasp_confidence=getattr(msg, 'confidence', 0.0),
                episodes_tested=1
            )
            
            self.model_performance_history[model_name].append(metrics)
            
            # Check for performance degradation
            self.check_performance_degradation(model_name)

    def data_collection_callback(self, msg):
        """Handle data collection status updates"""
        if hasattr(msg, 'episodes_collected'):
            episodes = msg.episodes_collected
            
            # Check if we have enough data to trigger training
            if (episodes >= self.config.min_episodes_before_training and 
                self.config.auto_train_enabled and
                self.should_trigger_training()):
                
                self.schedule_training()

    # =============================================
    # WORKFLOW ORCHESTRATION
    # =============================================
    
    def workflow_monitor_loop(self):
        """Main workflow monitoring loop"""
        while not self.shutdown_event.is_set():
            try:
                self.process_workflow_triggers()
                self.update_workflow_status()
                time.sleep(60)  # Check every minute
                
            except Exception as e:
                self.get_logger().error(f"Error in workflow monitor loop: {e}")
                time.sleep(10)

    def performance_monitor_loop(self):
        """Performance monitoring loop"""
        while not self.shutdown_event.is_set():
            try:
                self.analyze_performance_trends()
                self.generate_recommendations()
                time.sleep(self.config.performance_monitoring_interval_minutes * 60)
                
            except Exception as e:
                self.get_logger().error(f"Error in performance monitor loop: {e}")
                time.sleep(60)

    def process_workflow_triggers(self):
        """Process automatic workflow triggers"""
        current_time = datetime.now()
        
        # Check for scheduled training
        if (self.config.auto_train_enabled and 
            self.should_trigger_training() and
            self.current_state == WorkflowState.IDLE):
            
            self.get_logger().info("Triggering scheduled training")
            self.start_training_workflow()
        
        # Check for scheduled evaluations
        if self.current_evaluation_id and self.current_state == WorkflowState.EVALUATION:
            # Check if evaluation is complete (this would need actual evaluation status)
            self.check_evaluation_status()

    def should_trigger_training(self) -> bool:
        """Check if training should be triggered"""
        time_since_last_training = datetime.now() - self.last_training_time
        hours_since_training = time_since_last_training.total_seconds() / 3600
        
        return (hours_since_training >= self.config.training_trigger_interval_hours and
                self.current_state == WorkflowState.IDLE)

    def start_training_workflow(self):
        """Start automated training workflow"""
        try:
            self.current_state = WorkflowState.TRAINING
            self.current_training_id = f"auto_training_{int(time.time())}"
            
            # Prepare training request
            if CUSTOM_MSGS_AVAILABLE and self.start_training_client.service_is_ready():
                request = StartTraining.Request()
                request.model_name = "ACT_Policy"  # Default model
                request.dataset_name = "latest_dataset"
                request.epochs = 100
                request.learning_rate = 0.001
                
                future = self.start_training_client.call_async(request)
                future.add_done_callback(self.training_start_callback)
            else:
                # Fallback
                request = SetBool.Request()
                request.data = True
                future = self.start_training_client.call_async(request)
            
            self.get_logger().info(f"Started training workflow: {self.current_training_id}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to start training workflow: {e}")
            self.current_state = WorkflowState.ERROR

    def training_start_callback(self, future):
        """Callback for training start service call"""
        try:
            response = future.result()
            if hasattr(response, 'success') and response.success:
                self.get_logger().info("Training started successfully")
            else:
                self.get_logger().error("Training failed to start")
                self.current_state = WorkflowState.ERROR
        except Exception as e:
            self.get_logger().error(f"Training start callback error: {e}")
            self.current_state = WorkflowState.ERROR

    def schedule_training(self):
        """Schedule training to start"""
        self.get_logger().info("Scheduling training due to sufficient data collection")
        self.start_training_workflow()

    def schedule_model_evaluation(self):
        """Schedule model evaluation after training"""
        self.current_state = WorkflowState.EVALUATION
        self.current_evaluation_id = f"eval_{int(time.time())}"
        
        if CUSTOM_MSGS_AVAILABLE and self.evaluate_model_client.service_is_ready():
            request = EvaluateModel.Request()
            request.model_name = "latest_trained_model"
            request.num_episodes = self.config.evaluation_episodes
            
            future = self.evaluate_model_client.call_async(request)
            future.add_done_callback(self.evaluation_callback)
            
            self.get_logger().info(f"Started model evaluation: {self.current_evaluation_id}")

    def evaluation_callback(self, future):
        """Callback for model evaluation"""
        try:
            response = future.result()
            if hasattr(response, 'success') and response.success:
                success_rate = getattr(response, 'success_rate', 0.0)
                
                if success_rate >= self.config.success_rate_threshold:
                    self.get_logger().info(
                        f"Model evaluation successful (success rate: {success_rate:.2f})")
                    
                    if self.config.auto_deploy_enabled:
                        self.deploy_model()
                else:
                    self.get_logger().warning(
                        f"Model evaluation below threshold (success rate: {success_rate:.2f})")
            
            self.current_state = WorkflowState.IDLE
            
        except Exception as e:
            self.get_logger().error(f"Evaluation callback error: {e}")
            self.current_state = WorkflowState.ERROR

    def deploy_model(self):
        """Deploy evaluated model"""
        self.current_state = WorkflowState.DEPLOYMENT
        
        if CUSTOM_MSGS_AVAILABLE and self.deploy_model_client.service_is_ready():
            request = DeployModel.Request()
            request.model_name = "latest_trained_model"
            request.enable_a_b_testing = True
            
            future = self.deploy_model_client.call_async(request)
            future.add_done_callback(self.deployment_callback)
            
            self.get_logger().info("Starting model deployment")

    def deployment_callback(self, future):
        """Callback for model deployment"""
        try:
            response = future.result()
            if hasattr(response, 'success') and response.success:
                self.get_logger().info("Model deployed successfully")
                
                if getattr(response, 'a_b_testing_enabled', False):
                    self.start_a_b_testing()
                else:
                    self.current_state = WorkflowState.IDLE
            else:
                self.get_logger().error("Model deployment failed")
                self.current_state = WorkflowState.ERROR
                
        except Exception as e:
            self.get_logger().error(f"Deployment callback error: {e}")
            self.current_state = WorkflowState.ERROR

    def start_a_b_testing(self):
        """Start A/B testing between models"""
        self.current_state = WorkflowState.A_B_TESTING
        self.a_b_test_active = True
        
        self.get_logger().info("Starting A/B testing")
        
        # Schedule A/B test evaluation
        threading.Timer(
            self.config.a_b_test_duration_hours * 3600,
            self.evaluate_a_b_test
        ).start()

    def evaluate_a_b_test(self):
        """Evaluate A/B test results"""
        self.get_logger().info("Evaluating A/B test results")
        
        # This would analyze the performance of both models
        # and determine which one to keep active
        
        self.a_b_test_active = False
        self.current_state = WorkflowState.IDLE

    # =============================================
    # PERFORMANCE ANALYSIS
    # =============================================
    
    def check_performance_degradation(self, model_name: str):
        """Check for performance degradation"""
        if model_name not in self.model_performance_history:
            return
        
        history = self.model_performance_history[model_name]
        if len(history) < 10:  # Need sufficient data
            return
        
        # Compare recent performance to historical average
        recent_metrics = history[-10:]
        historical_metrics = history[-50:-10] if len(history) >= 50 else history[:-10]
        
        if not historical_metrics:
            return
        
        recent_success_rate = sum(m.success_rate for m in recent_metrics) / len(recent_metrics)
        historical_success_rate = sum(m.success_rate for m in historical_metrics) / len(historical_metrics)
        
        degradation = historical_success_rate - recent_success_rate
        
        if degradation > self.config.performance_degradation_threshold:
            self.get_logger().warning(
                f"Performance degradation detected for {model_name}: "
                f"{degradation:.3f} drop in success rate")
            
            # Trigger retraining or investigation
            if self.config.auto_train_enabled:
                self.get_logger().info("Triggering retraining due to performance degradation")
                self.schedule_training()

    def analyze_performance_trends(self):
        """Analyze performance trends across all models"""
        trends = {}
        
        for model_name, history in self.model_performance_history.items():
            if len(history) < 5:
                continue
            
            recent_metrics = history[-10:]
            success_rates = [m.success_rate for m in recent_metrics]
            
            # Simple trend analysis
            if len(success_rates) >= 5:
                early_avg = sum(success_rates[:5]) / 5
                late_avg = sum(success_rates[-5:]) / 5
                trend = "improving" if late_avg > early_avg else "declining" if late_avg < early_avg else "stable"
                
                trends[model_name] = {
                    'trend': trend,
                    'current_success_rate': late_avg,
                    'change': late_avg - early_avg
                }
        
        # Publish trends
        self.publish_performance_trends(trends)

    def generate_recommendations(self):
        """Generate recommendations based on performance analysis"""
        recommendations = []
        
        # Check overall system performance
        all_recent_success_rates = []
        for history in self.model_performance_history.values():
            if len(history) >= 5:
                recent_rates = [m.success_rate for m in history[-5:]]
                all_recent_success_rates.extend(recent_rates)
        
        if all_recent_success_rates:
            avg_success_rate = sum(all_recent_success_rates) / len(all_recent_success_rates)
            
            if avg_success_rate < 0.7:
                recommendations.append(
                    "Consider collecting more diverse training data - success rate below 70%")
            
            if avg_success_rate > 0.9:
                recommendations.append(
                    "Excellent performance! Consider deploying to production")
        
        # Check training frequency
        time_since_training = (datetime.now() - self.last_training_time).total_seconds() / 3600
        if time_since_training > self.config.training_trigger_interval_hours * 2:
            recommendations.append(
                f"Consider triggering training - {time_since_training:.1f}h since last training")
        
        # Publish recommendations
        if recommendations:
            self.publish_recommendations(recommendations)

    # =============================================
    # SERVICE CALLBACKS
    # =============================================
    
    def start_auto_workflow_callback(self, request, response):
        """Start/stop automatic workflow management"""
        try:
            self.config.auto_train_enabled = request.data
            
            if request.data:
                self.get_logger().info("Automatic workflow management enabled")
                response.success = True
                response.message = "Automatic workflow enabled"
            else:
                self.get_logger().info("Automatic workflow management disabled")
                response.success = True
                response.message = "Automatic workflow disabled"
                
        except Exception as e:
            self.get_logger().error(f"Error in start auto workflow: {e}")
            response.success = False
            response.message = str(e)
        
        return response

    def stop_workflow_callback(self, request, response):
        """Stop current workflow"""
        try:
            if self.current_state != WorkflowState.IDLE:
                # Stop current operations
                if self.current_state == WorkflowState.TRAINING and self.stop_training_client.service_is_ready():
                    stop_request = StopTraining.Request() if CUSTOM_MSGS_AVAILABLE else SetBool.Request()
                    if not CUSTOM_MSGS_AVAILABLE:
                        stop_request.data = False
                    
                    future = self.stop_training_client.call_async(stop_request)
                
                self.current_state = WorkflowState.IDLE
                self.get_logger().info("Workflow stopped")
                
            response.success = True
            response.message = "Workflow stopped"
            
        except Exception as e:
            self.get_logger().error(f"Error stopping workflow: {e}")
            response.success = False
            response.message = str(e)
        
        return response

    def trigger_training_callback(self, request, response):
        """Manually trigger training"""
        try:
            if self.current_state == WorkflowState.IDLE:
                self.start_training_workflow()
                self.get_logger().info("Training manually triggered")
            else:
                self.get_logger().warning(f"Cannot trigger training - current state: {self.current_state}")
                
        except Exception as e:
            self.get_logger().error(f"Error triggering training: {e}")
        
        return response

    def get_status_callback(self, request, response):
        """Get current workflow status"""
        return response

    # =============================================
    # UTILITY METHODS
    # =============================================
    
    def update_workflow_status(self):
        """Update and publish workflow status"""
        status = {
            'current_state': self.current_state.value,
            'auto_train_enabled': self.config.auto_train_enabled,
            'last_training_time': self.last_training_time.isoformat(),
            'active_workflows': list(self.active_workflows.keys()),
            'a_b_test_active': self.a_b_test_active,
            'timestamp': datetime.now().isoformat()
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.workflow_status_pub.publish(msg)

    def publish_performance_trends(self, trends: Dict):
        """Publish performance trend analysis"""
        msg = String()
        msg.data = json.dumps({
            'trends': trends,
            'timestamp': datetime.now().isoformat()
        })
        self.workflow_metrics_pub.publish(msg)

    def publish_recommendations(self, recommendations: List[str]):
        """Publish workflow recommendations"""
        msg = String()
        msg.data = json.dumps({
            'recommendations': recommendations,
            'timestamp': datetime.now().isoformat()
        })
        self.workflow_recommendations_pub.publish(msg)

    def check_evaluation_status(self):
        """Check if evaluation is complete"""
        # This would check the actual evaluation status
        # For now, we'll simulate completion after some time
        pass

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down Training Workflow Manager")
        self.shutdown_event.set()
        
        if self.workflow_thread and self.workflow_thread.is_alive():
            self.workflow_thread.join(timeout=5)
        if self.monitoring_thread and self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=5)
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        workflow_manager = TrainingWorkflowManager()
        rclpy.spin(workflow_manager)
    except KeyboardInterrupt:
        pass
    finally:
        if 'workflow_manager' in locals():
            workflow_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()