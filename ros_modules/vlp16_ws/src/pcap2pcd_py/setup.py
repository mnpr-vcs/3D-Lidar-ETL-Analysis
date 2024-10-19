from setuptools import setup

package_name = 'pcap2pcd_py'

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
    maintainer='mnpr_captn',
    maintainer_email='mnpr_captn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "reader_node = pcap2pcd_py.pcap_reader:main"
            , "test_pub = pcap2pcd_py.test_pub:main"
            , "test_sub = pcap2pcd_py.test_sub:main"
            , "sum_server = pcap2pcd_py.sum_server:main"
            , "sum_client = pcap2pcd_py.sum_client:main"
            , "hw_status_pub = pcap2pcd_py.hw_status_pub:main"
        ],
    },
)
