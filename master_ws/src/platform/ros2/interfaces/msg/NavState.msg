# 无人机导航状态(Controller 发布)
std_msgs/Header header
string mode                 # STANDBY / TRANSIT / ORBIT / FOLLOW
float64 lat
float64 lon
float32 alt_m
float64 heading_deg
bool arrived                # 是否到达指定点(转场完成判定)
