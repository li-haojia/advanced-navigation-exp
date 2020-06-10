# 机器人高级导航实验 作业
## 首先给设计这个实验的学长和老师点个赞  真的超级棒
1. 根据gazebo中机器人的位置 自动初始化机器人的初始位置
    * 请求gazebo服务器
    * 发布pose estamination的话题
    * 请求清除带代价地图

2. 在终端中输入一个目标点，并监控执行状态
    * 输入错误处理
    * 利用move_base/goal发布目标
    * 利用move_base/result监控状态

3. 读取landmarks.txt 中的坐标点信息，并依次执行 
    * 文件注释处理以及数据读取
    * 利用move_base/goal发布目标
    * 利用move_base/status监控状态  

    PS:其实应该用action_lib 但是我比较懒