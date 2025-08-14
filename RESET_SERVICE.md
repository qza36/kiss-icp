# KISS-ICP Reset Service

## Overview
The KISS-ICP package now includes a reset service that allows resetting the map and odometry to the initial state.

## Service Details
- **Service Name**: `/kiss/reset`
- **Service Type**: `std_srvs/srv/Empty`
- **Description**: Resets the local map and pose estimation to initial state

## Usage

### Command Line
```bash
ros2 service call /kiss/reset std_srvs/srv/Empty
```

### Python
```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class ResetClient(Node):
    def __init__(self):
        super().__init__('reset_client')
        self.client = self.create_client(Empty, '/kiss/reset')
        
    def call_reset(self):
        request = Empty.Request()
        future = self.client.call_async(request)
        return future

# Usage
rclpy.init()
client = ResetClient()
future = client.call_reset()
rclpy.spin_until_future_complete(client, future)
if future.result() is not None:
    print("Reset successful!")
else:
    print("Reset failed!")
```

### C++
```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

auto node = rclcpp::Node::make_shared("reset_client");
auto client = node->create_client<std_srvs::srv::Empty>("/kiss/reset");

auto request = std::make_shared<std_srvs::srv::Empty::Request>();
auto result = client->async_send_request(request);
// Handle result as needed
```

## When to Use
- When you want to start mapping from a new location
- When the odometry has drifted significantly 
- After moving the robot to a completely new environment
- For testing and debugging purposes

## Effects
After calling the reset service:
1. The local map (VoxelHashMap) is cleared
2. The pose is reset to identity (origin)
3. The motion delta is reset to identity
4. The adaptive threshold is reinitialized
5. Future frames will start building a new map from scratch