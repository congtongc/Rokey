from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rokey_pjt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일들 추가
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # config, params, models 등이 있다면 여기에 추가
        # 예시: (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_xyz = rokey_pjt.object_xyz:main',
            'capture_image = rokey_pjt.tb4_capture_image:main',
            'cont_cap_image = rokey_pjt.tb4_cont_capture_image:main',
            'det_obj = rokey_pjt.tb4_yolov8_obj_det:main',
            'det_obj_thread = rokey_pjt.tb4_yolov8_obj_det_thread:main',
            'det_obj_track = rokey_pjt.tb4_yolov8_obj_det_track:main',
            'depth_checker = rokey_pjt.depth_checker:main',
            'depth_checker_click = rokey_pjt.depth_checker_click:main',
            'yolo_depth_checker = rokey_pjt.tb4_yolo_bbox_depth_checker:main',
            'yolo_depth_checker1 = rokey_pjt.yolo_depth_checker:main',
            'tf_trans = rokey_pjt.tb4_tf_transform:main',
            'tf_point_transform = rokey_pjt.tf_point_transform:main',
            'parking_gui = rokey_pjt.parking_gui:main',
            'sc_follow_waypoints = rokey_pjt.sc_follow_waypoints:main',
            'detect_car_info = rokey_pjt.detect_car_info:main',
            'detect_car_info2 = rokey_pjt.detect_car_info2:main',


            'yolo_detect = rokey_pjt.yolo_detect:main',
            'detect_ps_map = rokey_pjt.detect_ps_map:main',
            'beep = rokey_pjt.beep:main',
            'sc_follow_waypoints2_1 = rokey_pjt.sc_follow_waypoints2_1:main',
            'sc_follow_waypoints2 = rokey_pjt.sc_follow_waypoints2:main',
            'detect_ps_front = rokey_pjt.detect_ps_front:main',
            'rotation_test = rokey_pjt.rotation_test:main',            

        ],
    },
)
