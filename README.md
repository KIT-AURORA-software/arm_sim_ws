# レポジトリ説明
<br>
アームのシミュレーションを行うワーキングスペース

## 使用方法
1. レポジトリをクローンする。
```
git clone https://github.com/KIT-AURORA-software/arm_sim_ws.git
```
2. ワーキングスペースに移動しビルドを行う
```
cd ~/arm_sim_ws
colcon build
source install/setup.bash
```
3. 起動する
```
ros2 launch aurora_arm launch_gazebo.launach.py
```
gazeboにアームがうつったらOK
