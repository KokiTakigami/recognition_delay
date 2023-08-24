from setuptools import setup

package_name = 'recognition_delay'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tacky',
    maintainer_email='koki.takigami@tier4.jp',
    description='Reproduce recognition delay',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recognition_delay = recognition_delay.recognition_delay:main',
            'test_marker = recognition_delay.test_marker_pub:main'
        ],
    },
)
