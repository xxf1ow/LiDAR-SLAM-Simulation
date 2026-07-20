#!/usr/bin/env bash
# mapping 建图收尾:存当前 LIO-SAM 地图 + 转 2D 占据栅格给 nav2。
#
# 用法:建图跑够后(终端里 lio_sam 还在跑),另开终端:
#   cd ~/xxsim/core && source install/setup.bash && bash mapping/save_map.sh
#
# 前提:
#   ① lio_sam 在跑(service /lio_sam/save_map 在线);
#   ② 已 source install/setup.bash(lio_sam msgs + robot_navigation 可用);
#   ③ open3d 已装(本机 user site ~/.local/lib/python3.10)。
#
# 流程(LIO-SAM 存盘前 rm -r 目标目录重建 —— 见 mapOptmization.cpp:188/414,
#       故必须存到专门子目录 ~/result/loam/,绝不能直接存 ~/result/):
#   1) service /lio_sam/save_map 存到 ~/result/loam/(rm -r loam/ 重建,安全)
#   2) cp GlobalMap.pcd → ~/result/GlobalMap.pcd(gicp_localization 默认读这里)
#   3) pcd_to_occupancy 转 ~/result/factory_map.yaml + .pgm(nav2 map_server 读)
set -euo pipefail

LOAM_DIR="${HOME}/result/loam"          # LIO-SAM 存盘目录(rm -r 重建,专门子目录)
PCD="${HOME}/result/GlobalMap.pcd"      # gicp 读的先验图
YAML="${HOME}/result/factory_map.yaml"  # nav2 读的占据栅格

echo "[1/3] service /lio_sam/save_map → ${LOAM_DIR}/"
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
    "{resolution: 0.2, destination: '/result/loam'}"

[ -f "${LOAM_DIR}/GlobalMap.pcd" ] || {
    echo "!! ${LOAM_DIR}/GlobalMap.pcd 未生成 —— service 失败? lio_sam 在跑吗?"; exit 1; }

echo "[2/3] cp GlobalMap.pcd → ${PCD}"
cp "${LOAM_DIR}/GlobalMap.pcd" "${PCD}"

echo "[3/3] pcd_to_occupancy → ${YAML}"
ros2 run robot_navigation pcd_to_occupancy \
    --pcd "${PCD}" --out "${YAML}" \
    --resolution 0.05 --z-min 0.1 --z-max 2.0 --min-pts 2

echo "[done] 先验图 + 占据栅格就绪:"
echo "  ${PCD}            (gicp_localization 读)"
echo "  ${YAML}           (nav2 map_server 读)"
echo "  ${YAML%.yaml}.pgm"
echo "  存盘原始(trajectory/transformations 等): ${LOAM_DIR}/"
