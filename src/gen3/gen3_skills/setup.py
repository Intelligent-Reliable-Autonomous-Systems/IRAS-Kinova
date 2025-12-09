from setuptools import find_packages, setup

package_name = 'gen3_skills'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Will Solow',
    maintainer_email='soloww@oregonstate.edu',
    description='Library of parameterized skills for Kinova Gen3 Arm',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ["skills_manager = gen3_skills.skills_manager:main"
        ],
    },
)
