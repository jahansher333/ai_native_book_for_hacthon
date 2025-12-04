---
id: synthetic-data
title: Synthetic Data Generation
sidebar_label: Synthetic Data
sidebar_position: 3
description: Generate millions of training images with domain randomization
keywords: [synthetic-data, domain-randomization, replicator, computer-vision, data-augmentation]
---

# Synthetic Data Generation

## Why Synthetic Data?

Real-world data collection is **expensive, slow, and limited**:
- Hiring annotators: $50-200 per 1,000 images
- Data collection: Weeks/months of robot operation
- Edge cases: Hard to capture (fire, floods, rare objects)

**Synthetic data solves this**: Generate millions of labeled images in hours with perfect annotations (segmentation masks, bounding boxes, depth, normals).

---

## Isaac Sim Replicator

**Replicator** is Isaac Sim's synthetic data generation framework.

### Key Features

- **Domain Randomization**: Vary textures, lighting, object poses
- **Parallel Rendering**: 1000s of images/hour (GPU-accelerated)
- **Perfect Labels**: Semantic segmentation, instance masks, bounding boxes
- **PyTorch Integration**: Direct tensor output (no file I/O)

---

## Basic Data Generation

### Capture Single Image

```python
import omni.replicator.core as rep

# Setup camera
camera = rep.create.camera(position=(5, 5, 5), look_at=(0, 0, 0))

# Attach writer (save to disk)
render_product = rep.create.render_product(camera, resolution=(1280, 720))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="output", rgb=True, distance_to_camera=True)
writer.attach([render_product])

# Capture frame
rep.orchestrator.step()
```

**Output**: `output/rgb_0000.png`, `output/distance_to_camera_0000.npy`

---

## Domain Randomization

### Randomize Lighting

```python
import omni.replicator.core as rep

def randomize_lights():
    lights = rep.get.prims(path_pattern="*/Light")

    with lights:
        rep.modify.attribute("intensity", rep.distribution.uniform(500, 2000))
        rep.modify.attribute("color", rep.distribution.uniform((0.8, 0.8, 0.8), (1.0, 1.0, 1.0)))

    return lights.node

# Register randomizer
rep.randomizer.register(randomize_lights)

# Trigger on every frame
with rep.trigger.on_frame():
    rep.randomizer.randomize_lights()
```

---

### Randomize Object Poses

```python
def randomize_objects():
    objects = rep.get.prims(semantics=[("class", "obstacle")])

    with objects:
        rep.modify.pose(
            position=rep.distribution.uniform((-5, -5, 0), (5, 5, 2)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

    return objects.node

rep.randomizer.register(randomize_objects)

with rep.trigger.on_frame():
    rep.randomizer.randomize_objects()
```

---

### Randomize Textures

```python
def randomize_materials():
    # Get all objects with "prop" semantic
    props = rep.get.prims(semantics=[("class", "prop")])

    # Define material library
    materials = [
        "omniverse://localhost/NVIDIA/Materials/2023_2/Base/Wood/Wood_American_Cherry.mdl",
        "omniverse://localhost/NVIDIA/Materials/2023_2/Base/Metal/Aluminum_Anodized.mdl",
        "omniverse://localhost/NVIDIA/Materials/2023_2/Base/Plastic/Plastic_Black.mdl"
    ]

    with props:
        rep.randomizer.materials(materials)

    return props.node

rep.randomizer.register(randomize_materials)
```

---

## Complete Data Generation Script

```python
import omni.replicator.core as rep
import numpy as np

# Setup scene
def setup_scene():
    # Add ground
    ground = rep.create.plane(scale=(50, 50, 1), position=(0, 0, 0))
    rep.physics.collider(ground)

    # Add 20 random obstacles
    obstacles = rep.create.from_usd(
        "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Props/Blocks/block_instanceable.usd",
        count=20
    )

    # Setup camera
    camera = rep.create.camera(position=(8, 8, 4), look_at=(0, 0, 0))

    return camera, obstacles

# Randomizers
def randomize_scene(obstacles):
    # Randomize obstacle positions
    with obstacles:
        rep.modify.pose(
            position=rep.distribution.uniform((-10, -10, 0.5), (10, 10, 2)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

    # Randomize lighting
    lights = rep.get.prims(path_pattern="*/Light")
    with lights:
        rep.modify.attribute("intensity", rep.distribution.uniform(500, 3000))

# Main generation loop
def generate_dataset(num_frames=1000):
    camera, obstacles = setup_scene()

    # Attach writer
    render_product = rep.create.render_product(camera, resolution=(1280, 720))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir="synthetic_data",
        rgb=True,
        distance_to_camera=True,
        semantic_segmentation=True,
        bounding_box_2d_tight=True
    )
    writer.attach([render_product])

    # Generate frames
    with rep.trigger.on_frame(num_frames=num_frames):
        randomize_scene(obstacles)

    # Run orchestrator
    rep.orchestrator.run()

# Execute
generate_dataset(num_frames=5000)
```

---

## Output Formats

### RGB Images

**Path**: `synthetic_data/rgb/rgb_0000.png`
**Format**: PNG (1280×720×3)

### Depth Maps

**Path**: `synthetic_data/distance_to_camera/distance_to_camera_0000.npy`
**Format**: NumPy array (1280×720, float32, meters)

### Semantic Segmentation

**Path**: `synthetic_data/semantic_segmentation/semantic_segmentation_0000.npy`
**Format**: NumPy array (1280×720, uint32, class IDs)

### Bounding Boxes

**Path**: `synthetic_data/bounding_box_2d_tight/bounding_box_2d_tight_0000.npy`
**Format**: NumPy structured array `[('semanticId', '<u4'), ('x_min', '<i4'), ('y_min', '<i4'), ('x_max', '<i4'), ('y_max', '<i4')]`

---

## Training with Synthetic Data

### PyTorch Dataset

```python
import torch
from torch.utils.data import Dataset
from PIL import Image
import numpy as np

class SyntheticDataset(Dataset):
    def __init__(self, data_dir, transform=None):
        self.data_dir = data_dir
        self.transform = transform

        # Count frames
        self.num_frames = len(list((Path(data_dir) / "rgb").glob("*.png")))

    def __len__(self):
        return self.num_frames

    def __getitem__(self, idx):
        # Load RGB
        rgb_path = Path(self.data_dir) / "rgb" / f"rgb_{idx:04d}.png"
        rgb = Image.open(rgb_path).convert('RGB')

        # Load depth
        depth_path = Path(self.data_dir) / "distance_to_camera" / f"distance_to_camera_{idx:04d}.npy"
        depth = np.load(depth_path)

        if self.transform:
            rgb = self.transform(rgb)

        return {
            'rgb': rgb,
            'depth': torch.from_numpy(depth).float()
        }
```

---

## Hands-On Lab: Pick-and-Place Dataset

**Goal**: Generate 10,000 images of objects on a table for pick-and-place training.

### Requirements

1. Table with 5-15 random objects
2. Randomize object types (cube, cylinder, sphere)
3. Randomize materials (wood, metal, plastic)
4. Randomize lighting (intensity, color)
5. Export RGB, depth, segmentation masks

### Starter Code

```python
import omni.replicator.core as rep

def setup_pick_place_scene():
    # TODO: Create table
    # TODO: Scatter objects (5-15 random count)
    # TODO: Setup camera above table
    pass

def randomize_pick_place():
    # TODO: Randomize object poses
    # TODO: Randomize materials
    # TODO: Randomize lighting
    pass

# Generate 10,000 frames
generate_dataset(num_frames=10000)
```

---

## Key Takeaways

✅ **Synthetic data** eliminates manual annotation
✅ **Domain randomization** improves model generalization
✅ **Replicator** generates 1000s of images/hour
✅ **Perfect labels** (segmentation, depth, bounding boxes)

---

## Next Steps

Learn Visual SLAM in **[Chapter 3: VSLAM](/docs/isaac/vslam)**.

<div style={{textAlign: 'center', marginTop: '3rem', padding: '2rem', backgroundColor: 'var(--ifm-color-emphasis-100)', borderRadius: '8px'}}>
  <h2>📚 Continue Learning</h2>
  <div style={{marginTop: '2rem'}}>
    <a className="button button--primary button--lg" href="/docs/isaac/vslam">
      Next: Visual SLAM →
    </a>
  </div>
</div>
