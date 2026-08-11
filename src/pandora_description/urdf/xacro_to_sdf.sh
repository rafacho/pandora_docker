#!/usr/bin/env bash
echo "Cleaning sdf and urdf"
rm -rf pandora.urdf pandora.sdf
rosrun xacro xacro -o pandora.urdf pandora.xacro
gz sdf -p pandora.urdf > pandora.sdf
echo "Generated SDF"