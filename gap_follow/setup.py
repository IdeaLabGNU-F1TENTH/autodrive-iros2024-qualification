from setuptools import setup, find_packages

package_name = 'gap_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Myeongjun Kim, Juyoung Kim, SunWoong Moon, Sujin Park, Ji-hong Park, Gyuhyeok Lee',
    maintainer_email='gnu_kim98@gnu.ac.kr, wndudwkd003@gnu.ac.kr, 219161113@gnu.ac.kr, park_11412@gnu.ac.kr, hong_0002@gnu.ac.kr, gyuhyeok_lee0527@gnu.ac.kr',
    description='autodrive gap_follow',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_node = scripts.reactive_node:main',
        ],
    },
)
