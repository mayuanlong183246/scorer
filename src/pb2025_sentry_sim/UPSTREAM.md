# Upstream provenance

Navigation integration reads the upstream launch, bridge, robot controller and MecanumDrive2 contracts. Imported sources retain their original licenses.

| Repository | Revision |
| --- | --- |
| https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator | 61c49f5f0a9fd1ab9d867fb30c49520f5d5a0ee7 |
| https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_interfaces | 424f50c831b3e74fa72446969362cdabb2e22cd1 |
| https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_core | 8209ec517dfbb7ecac613cf0c9534d67ff918b7d |
| https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_gazebo | 7443ff06c345752d0e23172c479d99b27207e2c7 |
| https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_gz_resources | b5c759f08844dfda19c79aa870866ace8d4c7b3a |
| https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description | a0541dddcbfe376f369a6345b532a10923ad9149 |

Downloaded GitHub API tarballs because git transport was unavailable. Resources and robot description were filled with missing assets only; existing local model files were preserved. Existing sdformat_tools is used. xmacro 1.2.1 is installed locally in `.sim_python`.

The navigation map is derived from the pinned rmuc_2025 collision STL (body offset 14.5,8,0.2). `scripts/make_field_map.py MESH OUTPUT_DIR` regenerates it. This conservative planar map blocks ramps; it is not a mapping/localization benchmark.
