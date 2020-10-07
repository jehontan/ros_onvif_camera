  
from glob import glob
from setuptools import setup

package_name = 'onvif_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'test_control', 'geometry_helper'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Je Hon Tan',
    author_email='jehontan@gmail.com',
    maintainer='Je Hon Tan',
    maintainer_email='jehontan@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS node for controlling ONVIF cameras.',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'onvif_camera = onvif_camera.onvif_camera:main',
            'test_control = test_control.test_control:main'
        ],
    },
)