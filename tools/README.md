# Local Tool Entrypoints

Run these commands from the workspace root:

```bash
./tools/Simulator.sh
./tools/Library.sh
```

`Simulator.sh` starts the offline Regional simulator with the complete tactical
roster. Its default page is `http://127.0.0.1:9011/tactical`; all arguments are
passed through to `scripts/python/start.py`.

`Library.sh` serves the local documentation website at `http://127.0.0.1:1037/`.
Its Documentation, Graph, and Split modes are live views of `docs/**/*.md`:
files create pages/nodes and their Markdown or Obsidian links create edges. All
arguments are passed through to `scripts/understand_graph_dashboard.py`.

The other retained tool directories are focused utilities, not additional
simulation systems: `Behaviortree/` is the offline BT XML viewer, `maps/` is
the field-map and point-authoring toolchain, and `PnPcamera/` is the camera
intrinsic-calibration utility.
