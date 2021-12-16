from setuptools import setup

package_name = 'recording'

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
    maintainer='toranosuke',
    maintainer_email='toranosuke11041999@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recording = recording.recording:main',
            'get_index = recording.get_index:main',
            'sub_recording = recording.sub_recording:main',
            'sub_test_recording = recording.sub_test_recording:main'
        ],
    },
    py_modules=['recording.get_index', 'recording.recording',
                'recording.sub_recording', 'recording.sub_test_recording']
)
