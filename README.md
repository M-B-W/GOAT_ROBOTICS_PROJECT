# Butler Bot – Autonomous Food Delivery Robot

## 1. Project Overview

The Butler Bot project simulates a smart, autonomous delivery robot built using ROS 2 Humble and Python. It is designed for environments such as cafes or office spaces to transport items between a kitchen and predefined tables. This project supports navigation, order handling, confirmation, cancellation, and interactive GUI operation.

## 2. Technologies Used

- ROS 2 Humble
- Python 3
- Tkinter (for GUI)
- Nav2 (ROS Navigation Stack)
- RViz2
- Gazebo Simulation
- Nav2 Simple Commander API

## 3. Project Features

- Autonomous navigation to/from home, kitchen, and tables using Nav2
- Real-time task management with visual feedback
- Task confirmations and cancellations through a GUI
- Multi-table delivery with confirmation logic
- Dynamic table skipping and task abortion via topic-based control
- GUI console logging and task status display

## 4. Tasks Breakdown

1. **Task 1**: Basic delivery from home to kitchen and then to a selected table.
2. **Task 2**: Delivery with timeout confirmation. Robot waits for a confirmation message and returns home if not received.
3. **Task 3**: After delivery to kitchen, if no confirmation at the table, robot returns to kitchen before going home.
4. **Task 4**: Task cancellation handling. Robot aborts delivery mid-way and navigates accordingly.
5. **Task 5**: Delivery to multiple tables sequentially with no confirmation or cancellation.
6. **Task 6**: Delivery with confirmation. If not received, robot skips that table and moves to the next.
7. **Task 7**: Dynamic skipping of canceled orders using a cancel_topic listener. Robot finishes valid deliveries and returns.

## 5. GUI Features

- Button controls to launch Task 1 to Task 7
- Dynamic table selection and comma-separated entry fields for multi-table tasks
- Confirm and Cancel buttons for Kitchen and Tables (table1, table2, table3)
- Stop Task button to immediately terminate a running node
- Reverse Table order functionality for Tasks 5, 6, and 7
- Refresh button to clear the console log view
- Console section showing real-time ROS output and robot status logs

## 6. Setup Instructions

1. Install ROS 2 Humble and source the ROS environment.
2. Clone the project repository and navigate to the workspace:
   ```bash
   git clone https://github.com/your-repo/butler-bot.git
   cd butler-bot
   ```
3. Build the workspace and source it:
   ```bash
   colcon build --packages-select butler_bot
   source install/setup.bash
   ```
4. Launch the simulation (optional):
   ```bash
   ros2 launch butler_bot_sim simulation.launch.py
   ```
5. Run the GUI:
   ```bash
   python3 src/gui/butler_gui.py
   ```

## 7. ROS Topics Used

| Topic             | Message Type           | Purpose                                      |
|------------------|------------------------|----------------------------------------------|
| /confirm_stage   | std_msgs/msg/String    | Confirmation for kitchen or table delivery   |
| /cancel_task     | std_msgs/msg/String    | Cancel a specific task for a table or kitchen|
| /goal_pose       | geometry_msgs/PoseStamped | Send goals for navigation                 |

## 8. Future Enhancements

- Add speech feedback for robot delivery status
- Implement QR code detection for table recognition
- Add dynamic SLAM and multi-robot tasking
- Cloud-based task logging and dashboard integration

## 9. Contributors

This project was developed by Rabina and team during a two-month internship at SREC using ROS 2 Humble. Full delivery simulation was tested in Gazebo and managed via a custom Python GUI.
