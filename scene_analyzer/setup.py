from setuptools import find_packages, setup

package_name = 'scene_analyzer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'volcengine-python-sdk[ark]'],
    zip_safe=True,
    maintainer='Team 2',
    maintainer_email='team2@medical-robot.dev',
    description='Team 2 — 场景分析：视觉推理 + 检测调度 + 车载信息面板',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scene_analyzer = scene_analyzer.scene_analyzer:main',
            'info_display_panel = scene_analyzer.info_display_panel:main',
        ],
    },
)
