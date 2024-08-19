from setuptools import find_packages, setup

package_name = 'eai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/eai.launch.py']),
        ('share/' + package_name, ['launch/rs_launch.py']),
        ('share/' + package_name, ['config/camera_config.yaml']),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arpit',
    maintainer_email='carpit680@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = eai.main:main',
            'gradio_node = eai.gradio_node:main'
        ],
    },
)
