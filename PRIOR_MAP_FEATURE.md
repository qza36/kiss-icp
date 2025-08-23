# KISS-ICP Prior Map Initialization Feature

This document describes the new prior PCD map initialization functionality added to KISS-ICP.

## Overview

The prior map initialization feature allows KISS-ICP to initialize its pose by aligning the first frame of sensor data with a pre-existing point cloud map in PCD format. This is useful for localization in known environments.

## Usage

### Command Line Interface

You can specify a prior map when running the KISS-ICP pipeline:

```bash
kiss_icp_pipeline --visualize --prior-map /path/to/prior_map.pcd <data-dir>
```

### Python API

```python
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP

# Configure with prior map
config = KISSConfig()
config.initialization.use_prior_map = True
config.initialization.prior_map_path = "/path/to/prior_map.pcd"

# Create odometry instance
odometry = KissICP(config=config)

# Process frames - first frame will be aligned with prior map
frame, source = odometry.register_frame(point_cloud, timestamps)
```

### Configuration File

You can also specify the prior map in a YAML configuration file:

```yaml
initialization:
  use_prior_map: true
  prior_map_path: "/path/to/prior_map.pcd"
```

Then use it:

```bash
kiss_icp_pipeline --config config.yaml --visualize <data-dir>
```

## How it Works

1. **Loading**: When `use_prior_map` is enabled and a valid `prior_map_path` is provided, the prior map is loaded at initialization
2. **First Frame**: When processing the first frame, instead of using the default identity pose, the system aligns the first frame with the prior map using ICP registration
3. **Subsequent Frames**: After initialization, the system continues with normal operation using the standard motion model

## Supported Formats

- ASCII PCD files (.pcd)
- The system uses the existing KISS-ICP point cloud loading infrastructure, so other formats supported by the `GenericDataset` should work

## Implementation Details

### Python Implementation
- New `InitializationConfig` class in `kiss_icp/config/config.py`
- Prior map loading and initialization logic in `kiss_icp/kiss_icp.py`
- Command line option added to `kiss_icp/tools/cmd.py`
- Pipeline updated to accept configuration objects in `kiss_icp/pipeline.py`

### C++ Implementation
- Extended `KISSConfig` struct in `cpp/kiss_icp/pipeline/KissICP.hpp`
- Prior map loading and initialization methods in `cpp/kiss_icp/pipeline/KissICP.cpp`
- Simple ASCII PCD loader for C++ (can be extended for binary format)

## Benefits

- **Improved Localization**: Better initial pose estimation in known environments
- **Reduced Drift**: Starting with a good initial pose reduces accumulation of errors
- **Faster Convergence**: ICP registration can converge faster with better initialization
- **Map Consistency**: Ensures trajectories are consistent with existing maps

## Notes

- The feature is completely optional and backward compatible
- If the prior map file cannot be loaded, the system falls back to default initialization
- The prior map is only used for the first frame; subsequent frames use normal odometry
- Large prior maps may increase initialization time but shouldn't affect runtime performance