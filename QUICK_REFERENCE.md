# 🛠️ DEVELOPMENT QUICK REFERENCE

## **Essential Commands**

### **Build System**
```bash
# Build all packages
cd ~/ros2_ws
colcon build

# Build specific packages
colcon build --packages-select drone_msgs drone_mission

# Build with debug info
colcon build --packages-select drone_msgs --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Source environment
source install/setup.bash
```

### **Launch System**
```bash
# Launch complete system
ros2 launch system_full.launch.py

# Launch with configuration
ros2 launch system_full.launch.py config_file:=/path/to/custom_config.yaml

# Launch in simulation mode
ros2 launch system_full.launch.py simulation:=true
```

### **Testing Commands**
```bash
# Run all tests
python3 mvp_test_suite.py

# Run specific test categories
python3 mvp_test_suite.py --category unit
python3 mvp_test_suite.py --category integration
python3 mvp_test_suite.py --category system

# Run with verbose output
python3 mvp_test_suite.py --category all --verbose
```

### **System Status & Debugging**
```bash
# Check system status
ros2 node list
ros2 topic list
ros2 service list

# Monitor system coordinator
ros2 topic echo /system_status

# Monitor mission status
ros2 topic echo /mission_status

# Check drone status
ros2 topic echo /drone_status

# View logs
ros2 run rqt_console rqt_console
```

### **Development Workflow**
```bash
# 1. Make changes to code
# 2. Build affected packages
colcon build --packages-select [package_name]

# 3. Source environment
source install/setup.bash

# 4. Test changes
python3 mvp_test_suite.py --category unit

# 5. Launch system for integration testing
ros2 launch system_full.launch.py

# 6. Run integration tests
python3 mvp_test_suite.py --category integration
```

## **Troubleshooting**

### **Build Issues**
```bash
# Clean build
rm -rf build install log
colcon build

# Check for missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build with verbose output
colcon build --event-handlers console_direct+
```

### **Runtime Issues**
```bash
# Check ROS 2 setup
echo $ROS_DISTRO
echo $AMENT_PREFIX_PATH

# Check for missing topics/services
ros2 topic list | grep [topic_name]
ros2 service list | grep [service_name]

# Check node health
ros2 node info [node_name]
```

### **Performance Monitoring**
```bash
# Monitor CPU/Memory usage
htop

# Monitor ROS 2 performance
ros2 run rqt_top rqt_top

# Network monitoring
ros2 run rqt_graph rqt_graph
```

## **Key File Locations**

### **Configuration**
- Main config: `/home/adama133/ros2_ws/config/drone_config.yaml`
- Launch files: `/home/adama133/ros2_ws/launch/`
- Test config: `/home/adama133/ros2_ws/config/test_config.yaml`

### **Source Code**
- Custom messages: `/home/adama133/ros2_ws/src/drone_msgs/`
- Enhanced mission: `/home/adama133/ros2_ws/src/drone_mission/enhanced_mission_node.py`
- System coordinator: `/home/adama133/ros2_ws/src/drone_system/system_coordinator.py`
- Test suite: `/home/adama133/ros2_ws/mvp_test_suite.py`

### **Documentation**
- MVP Plan: `/home/adama133/ros2_ws/docs/MVP_PLAN_DETAILED.md`
- Week 1 Guide: `/home/adama133/ros2_ws/docs/WEEK1_IMPLEMENTATION_GUIDE.md`
- Architecture: `/home/adama133/ros2_ws/docs/SYSTEM_ARCHITECTURE.md`

## **Next Actions Checklist**

- [ ] Build all packages: `colcon build`
- [ ] Source environment: `source install/setup.bash`
- [ ] Run unit tests: `python3 mvp_test_suite.py --category unit`
- [ ] Launch system: `ros2 launch system_full.launch.py`
- [ ] Run integration tests: `python3 mvp_test_suite.py --category integration`
- [ ] Check system status: `ros2 topic echo /system_status`
- [ ] Validate mission functionality: Test basic mission execution
- [ ] Review logs for any issues
- [ ] Update existing packages for message integration
- [ ] Run comprehensive system tests
