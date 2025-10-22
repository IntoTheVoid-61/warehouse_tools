# warehouse_tools (ROS 2 Package)

This ROS 2 package is part of the **Warehouse-Pathfinder** ecosystem and provides graphical tools for **warehouse environment creation and pickup scenario definition**.  
It builds directly on the original `CreateWarehouse.py` module from the base **Warehouse-Pathfinder** repository, extending it into a **ROS 2-integrated system** with parameterized configuration and launch support.

---

## Overview

The `warehouse_tools` package provides GUI utilities and ROS 2 nodes for creating:
- **Empty warehouse environments**, and  
- **Pickup/Delivery scenarios** within these environments.

These tools integrate with ROS 2’s node and launch system, allowing easy configuration and reuse across other packages in the Warehouse-Pathfinder suite.

---

## Relation to the Original Repository

This package originates from the **[`Warehouse-Pathfinder`](https://github.com/IntoTheVoid-61/Warehouse-Pathfinder)** project.

In the original repository:
- `CreateWarehouse.py` was a **standalone Python class** handling map generation, geometry storage, and saving to disk.
- All operations were launched manually or through scripts.

In this ROS 2 package:
- The same `CreateWarehouse` logic has been **encapsulated into ROS 2 nodes**.
- Launch files and YAML configuration files are used to manage parameters and execution context.
- The package supports modular interaction between warehouse generation, scenario creation, and future simulation or navigation nodes.

---

## Package Structure

<p>warehouse_tools/
<p>├── warehouse_tools/
<p>│ ├── init.py
<p>│ ├── create_warehouse.py # Core logic (adapted from original Warehouse-Pathfinder)
<p>│ ├── create_empty_warehouse.py # ROS 2 node for warehouse creation GUI
<p>│ └── create_pickup_scenario.py # ROS 2 node for pickup scenario GUI
<p>│
<p>├── config/
<p>│ ├──  create_empty_params.yaml # Default YAML config for warehouse creation
<p>│ └──  create_pickup_params.yaml # Default YAML config for pickup scenario creation
<p>│
<p>├── launch/
<p>│ ├── create_warehouse.launch.py # Launch file for creating warehouse GUI
<p>│ └── create_warehouse.launch.py # Launch file for creating pickup scenario GUI
<p>│
<p>├── package.xml
<p>├── setup.py
<p>└── README.md

---

## Installation

### 1. Clone into your ROS 2 workspace
    cd ~/ros2_ws/src
    git clone https://github.com/IntoTheVoid-61/warehouse_tools.git

### 2. Build package
    cd ~/ros2_ws
    colcon build --packages-select warehouse_tools
    source install/setup.bash

### 3. Install dependecies (inside venv)
    pip install pygame pyyaml

---

## Usage

### Create warehouse model
    ros2 launch warehouse_tools create_warehouse.launch.py

This will allow you to create a warehouse with configurable **parameters**:
 - Number of blocks
 - Number of aisles
 - Number of locations per aisle

<p>To save the warehouse press SPACE_BAR key.
<p>The model of the warehouse will be saved in parent_dir/warehouse_name/warehouse_name_empty 
It will include:

- **png** image of the warehouse,
- **.txt** with the matrix representation of the warehouse.

### Create pickup scenario
    ros2 launch warehouse_tools create_pickup_scenario.launch.py

<p>This will load the model warehouse, where you can click to select desired pickup locations.

<p>To save the scenario press SPACE_BAR key.
<p>It will create an pickup_scenario folder inside the parent_dir/warehouse_name/warehouse_name/scenarios folder
It will include:

- **png** image of the warehouse,
- **.txt** file with the matrix representation of the warehouse (with pickup locations).

---

## Configuration (YAML)
<p>Parameters for the warehouse creation node are set via YAML files under the config/ folder.

---

# Author

<p> Ziga Breznikar
<p> email: ziga.breznikar@student.um.si




    




