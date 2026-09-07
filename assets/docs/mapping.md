# Mapping

`slam.launch.py` brings up the hardware, the EKF and `slam_toolbox` in
`online_async` mode. It starts no teleop node, so drive the robot manually:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`run_slam.sh` wraps the launch file with bag recording and an automatic map save on
exit:

```bash
./run_slam.sh --scenario S6 --speed 0.10 --run 01
./run_slam.sh --no-save-map --no-bag        # dry run
./run_slam.sh --help
```

## Saving a map manually

Confirm the map topic is alive first:

```bash
ros2 topic hz /map
```

### Preferred: raise the timeout and match the QoS

`slam_toolbox` publishes `/map` with transient-local durability, and large maps
exceed the map saver's default timeout. Set both:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/mbrobot_ws/src/mobile_robot/maps/map_e6 \
  --ros-args \
    -p save_map_timeout:=60000 \
    -p map_subscribe_transient_local:=true \
    -p free_thresh_default:=0.25 \
    -p occupied_thresh_default:=0.65
```

### Alternative: the slam_toolbox service

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '$HOME/mbrobot_ws/src/mobile_robot/maps/map_e6'}"
```

## After saving

`map_saver_cli` writes a `.pgm` and a `.yaml`. Keep the `image:` key in the `.yaml`
relative to the file itself so the map stays portable across machines:

```yaml
image: map_e6.pgm
```

The `floor` launch argument selects `maps/map_<floor>.yaml` together with
`config/checkpoints_<floor>.yaml`, so a new map needs a matching checkpoint file
before `navigator` can use it.

`scripts/tools/save_checkpoints.py` captures poses from `/amcl_pose` on each ENTER
keypress and writes them as JSON. `navigator` reads the YAML schema shown in the
existing `config/checkpoints_*.yaml` files, so the captured output has to be
converted before use.

Re-run `colcon build` after adding maps or checkpoints — the launch files read them
from `install/share/`, not from the source tree.
