from pathlib import Path
from glob import glob
from setuptools import setup

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
data=[('share/ament_index/resource_index/packages',['resource/orchard_bringup']),('share/orchard_bringup',['package.xml'])]
for sub in ('models','assets','config'):
    files=glob(f'../../{sub}/*')
    data.append((f'share/orchard_bringup/{sub}',files))
data.append(('share/orchard_bringup/python/orchard_drone',glob('../../orchard_drone/*.py')))
data.append(('share/orchard_bringup/launch',glob('launch/*.launch.py')))
setup(name='orchard_bringup',version='0.1.0',packages=['orchard_bringup'],
      package_dir={'':'.'},
      data_files=data,install_requires=['setuptools'],zip_safe=False,
      entry_points={'console_scripts':['orchard_sim=orchard_bringup.simulation_node:main']})
