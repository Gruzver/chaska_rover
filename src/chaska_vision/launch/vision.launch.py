from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    yolo_detector = Node(
        package='chaska_vision',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model': 'yolov8n.pt',
            'confidence': 0.4,
            'input_topic': '/depth_camera/image',
            'output_topic': '/depth_camera/image_detected',
        }],
    )

    return LaunchDescription([
        yolo_detector,
    ])
