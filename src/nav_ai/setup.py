from setuptools import setup

package_name = 'nav_ai'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ai_planner.launch.py']),
        ('share/' + package_name + '/policy', ['policy/model.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='AI local planner node for Nav2 (topic-based cmd_vel modulator)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ai_planner = nav_ai.ai_planner_node:main',
        ],
    },
)
