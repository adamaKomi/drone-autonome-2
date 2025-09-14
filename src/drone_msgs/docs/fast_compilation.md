# Compiler facilement le package pour drone_navigation

```
rm -rf build/drone_msgs/ install/drone_msgs/ log/
```
```
colcon build --packages-select drone_msgs --parallel-workers 4 --cmake-args -DBUILD_TESTING=OFF
```