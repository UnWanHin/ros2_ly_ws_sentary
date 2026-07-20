# Local Tool Entrypoints

Run these commands from the workspace root:

```bash
./tools/Simulator.sh
./tools/Library.sh
```

`Simulator.sh` starts the offline Regional simulator with the complete tactical
roster. Its default page is `http://127.0.0.1:9011/tactical`; all arguments are
passed through to `scripts/python/start.py`.

`Library.sh` serves the read-only project knowledge graph at
`http://127.0.0.1:1037/`; all arguments are passed through to
`scripts/understand_graph_dashboard.py`.

The other retained tool directories are focused utilities, not additional
simulation systems: `Behaviortree/` is the offline BT XML viewer, `maps/` is
the field-map and point-authoring toolchain, and `PnPcamera/` is the camera
intrinsic-calibration utility.
