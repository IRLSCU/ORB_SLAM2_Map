# ORB_SLAM2_Map
ORB_SLAM2 tool to create map

# ORB-SLAM 编译
- 相关脚本三个:
  - build.sh -> 构建基于linux的orb
  - build_ros.sh -> 构建基于ros的orb, 注意设置环境变量ROS_PACKAGE_PATH
  - build_all.sh -> 一个便捷的完备脚本用于一次性构建 linux的orb和ros的orb, **建议直接运行这个**
# ORB-SLAM 运行
- 流程
  - 启动摄像头(脚本:run_1_cam.sh)
  - orb轨迹记录模式(脚本: run_2_orb.sh, 选项为不加载地图, 不保存地图, 使用ctrl c结束任务, 结束后提示地图保存,选择保存)
  - orb重定位模式(脚本: run_2_orb.sh, 选项为加载地图, 不保存地图, 重定位模式, 使用ctrl c结束任务, 结束后提示地图保存,选择不保存)
  - 位姿记录(脚本: record_pose.sh, 功能为使用rosbag recore 记录topic名为cur_pose的回话)
  - 位姿博让(脚本: play.sh 此只是示例, 一定要修改bag包的名字)
