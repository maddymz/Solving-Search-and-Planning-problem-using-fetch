### Dependencies
- Fast Downward Planner ([Installation](http://www.fast-downward.org/HomePage))
- fetch gazebo demo ros package (run `sudo apt-get install ros-indigo-fetch-gazebo-demo`)

### Running
Search
1. Change paths on line 5 and line 6 in gen_maze.py
2. Run `roscore`
3. Run `rosrun search server.py -d <dimensions> -n <obstacles> -s <seed>`
4. Run `roslaunch search simulation.launch`
5. Run `roslaunch search demo.launch`
6. Run `rosrun search move_fetch.py`
7. Run `rosrun search search_algorithm.py -a <search_algo>`

The robot will navigate itself in the maze and reach the top right corner

Planning
1. Change paths on line 14 in server.py
2. Change paths on lines 18-22 in sort_cubes.py
3. Copy `demo_cube` folder to the `.gazebo` in `home` directory
4. Run `roscore`
5. Run `rosrun planning server.py -c <cubes> -s <seed>`
6. Run `roslaunch planning simulation.launch`
7. Run `roslaunch planning demo.launch`
8. Run `rosrun planning move_fetch.py`
9. Run `rosrun planning sort_cubes.py`

The robot will navigate to the cubes pick them up and place them in the top right corner of the maze


