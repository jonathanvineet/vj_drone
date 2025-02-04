VJ Drone

Overview

VJ Drone is a ROS2-based quadrotor control project that integrates PX4 with Gazebo for simulation. The project provides a Python script to control the drone, making it accessible for developers and researchers working with aerial robotics.

Features

Simulates a quadrotor using PX4 and Gazebo

Python-based control for autonomous flight

Easy setup and integration with ROS2

Prerequisites

Before setting up the project, ensure you have the following dependencies installed:

System Requirements

Ubuntu 22.04 (recommended)

At least 20GB of free disk space

A computer with a decent GPU (for Gazebo rendering)

Software Dependencies

Run the following commands to install required packages:

sudo apt update && sudo apt upgrade -y
sudo apt install git python3 python3-pip python3-venv \
    protobuf-compiler libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libeigen3-dev libopencv-dev libyaml-cpp-dev python3-jinja2 python3-numpy \
    python3-toml python3-pandas python3-sympy python3-setuptools python3-scipy \
    python3-matplotlib gazebo11 libgazebo11-dev

Setup

Clone the Repository

git clone https://github.com/jonathanvineet/vj_drone.git
cd vj_drone

Install Python Dependencies

pip install -r requirements.txt

Setting Up PX4 and Gazebo

Navigate to your PX4 installation directory:

cd ~/PX4-Autopilot

Run the simulation with Gazebo:

make px4_sitl gazebo

Running the Drone Control Script

With the simulation running, execute the Python script to make the drone hover:

python3 drone_hover.py

Contributing

Feel free to fork this repository and submit pull requests. Contributions are welcome!

License

This project is licensed under the MIT License.

Contact

For any questions or issues, open an issue on GitHub or contact me at jonathanvineet@example.com.
