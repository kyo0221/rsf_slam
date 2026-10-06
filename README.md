点群rosbagから2D地図を生成・保存します。

```bash
ros2 launch rsf_slam mapping.launch.py bag:=/path/to/bag output:=/path/to/map
```

GNSS 理想切替(rsf_bag_visualizer の simulate_switch)の軌跡で地図を作る場合は `georef` を指定します。地図は georef の地図座標系で出力されます。

```bash
S=$(ros2 pkg prefix rsf_slam)/share/rsf_slam/config
ros2 launch rsf_slam mapping.launch.py bag:=/path/to/bag output:=/path/to/map rate:=2.0 \
  params_file:=$S/slam_params_gnss.yaml georef:=$S/tsukuba_georef.yaml
```

2 枚の地図を継ぎ合わせるには `merge_maps.py` を使います。1 枚目の既知セルを優先し、その周囲 `--margin` m 以内にある 2 枚目の障害物は除きます。出力範囲は両方を含む大きさで、格子は 1 枚目に揃えます。

```bash
ros2 run rsf_slam merge_maps.py kakunin.yaml gnss_full.yaml merged --margin 5.0
```

slam_toolbox を使わず、GNSS 理想切替の軌跡で点群を直接描く場合は `render_gnss_map.py` を使います。セルごとに障害物として当たった回数とレーザーが通過した回数を数え、その比で占有・自由を決めます。高さの帯の外の点(地面や建物上部)も通過として使うので自由空間が広く描かれ、壁は自由空間に面した表面だけが線として残ります。

```bash
S=$(ros2 pkg prefix rsf_slam)/share/rsf_slam/config
ros2 run rsf_slam render_gnss_map.py /path/to/bag $S/tsukuba_georef.yaml /path/to/map --window 10 --min-move 3
```
