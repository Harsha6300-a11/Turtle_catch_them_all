# Turtle_catch_them_all

Built a ROS 2 Jazzy project “Turtle Catch Them All” using Turtlesim.
Designed custom messages and services to spawn, track, and catch turtles in real time, integrating publisher–subscriber communication and service calls for autonomous control.

# 🐢 Turtle Catch Them All (ROS 2 Project)

This project is built using **ROS 2 Jazzy** and the **Turtlesim** package.  
The goal is to spawn turtles randomly in the simulator, control one main turtle, and **catch them all** using custom ROS2 services and messages.  

---

## 🚀 Features
- Random turtle spawning in the Turtlesim environment  
- Controller logic for moving the main turtle towards spawned turtles  
- **Custom service (`CatchTurtle.srv`)** to remove turtles once caught  
- **Custom messages (`Turtle.msg`, `TurtleArray.msg`)** to manage active turtles  

---

## 📂 Project Structure
```

Turtle\_catch\_them\_all/
├── src/
│   ├── my\_robot\_interfaces/   # Custom messages & services
│   │   ├── msg/               # Turtle.msg, TurtleArray.msg
│   │   ├── srv/               # CatchTurtle.srv
│   ├── turtle\_catch/          # Nodes: turtle\_controller, turtle\_spawner
├── README.md
├── LICENSE
├── .gitignore
└── package.xml

````

---

## 🛠 Requirements
- Ubuntu 24.04  
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)  
- Python 3.12  
- turtlesim package  

---

## ⚙️ Installation
```bash
# Clone the repository
git clone https://github.com/Harsha6300-a11/Turtle_catch_them_all.git
cd Turtle_catch_them_all

# Build the packages
colcon build --packages-select my_robot_interfaces turtle_catch

# Source the setup
source install/setup.bash
````

---

## ▶️ Run the Project

1. Start **Turtlesim**

   ```bash
   ros2 run turtlesim turtlesim_node
   ```

2. Run the **Turtle Spawner**

   ```bash
   ros2 run turtle_catch turtle_spawner
   ```

3. Run the **Turtle Controller**

   ```bash
   ros2 run turtle_catch turtle_controller
   ```

---

## 🔎 How It Works

1. **Turtle Spawner Node**

   * Randomly spawns turtles in the Turtlesim environment.
   * Publishes all active turtles as a `TurtleArray` message.

2. **Turtle Controller Node**

   * Subscribes to the `alive_turtles` topic (list of active turtles).
   * Chooses one turtle to catch and computes velocity commands.
   * Publishes movement commands to `/turtle1/cmd_vel` to move the main turtle.
   * Once a turtle is close enough, it calls the `CatchTurtle` service.

3. **Custom Service – CatchTurtle.srv**

   * Takes the name of a turtle to catch.
   * Removes that turtle from the simulator using the `kill` service.
   * Updates the active turtles list.

4. **Messages**

   * `Turtle.msg`: Represents one turtle (name, x, y, theta).
   * `TurtleArray.msg`: Holds a list of currently alive turtles.

---

## 📜 License

This project is licensed under the MIT License.
You are free to use, modify, and distribute it.

---

## 🙌 Acknowledgments

* ROS 2 Jazzy documentation
* Turtlesim package (ROS classic demo)

```
