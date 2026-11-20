# Third-party robot descriptions

The URDF files in this directory are not covered by the MIT license of the
rest of the repository. They are xacro-expanded copies of robot descriptions
published by the ROS-Industrial project and keep their upstream licenses.
Mesh files are not included; the `package://` mesh paths are only used for
visualisation and are not needed by TwinBench.

| File | Upstream package | Repository | Authors (package.xml) | License |
|------|------------------|------------|-----------------------|---------|
| `ur5e.urdf`, `ur10e.urdf` | `ur_description` | https://github.com/ros-industrial/universal_robot (noetic-devel) | Wim Meeussen, Kelsey Hawkins, Mathias Ludtke, Felix Messmer | BSD (as declared in `ur_description/package.xml`) |
| `staubli_tx60.urdf` | `staubli_tx60_support` | https://github.com/ros-industrial/staubli_experimental (kinetic-devel) | Nils Melchert (Leibniz University Hannover) | Apache License 2.0, see `LICENSE-Apache-2.0.txt` |

These descriptions are community-maintained, not vendor-validated.

