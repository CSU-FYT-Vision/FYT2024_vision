# rm_upstart

看门狗程序及自启动服务

## 看门狗

每个节点都会以1s一次的频率发布heartbeat话题，看门狗程序(rm_watch_dog.sh)通过ros2 topic echo监听这些话题，如果有一个节点在TIMEOUT(10s)内都没有更新心跳数据，看门狗程序将杀死所有节点并重新启动程序