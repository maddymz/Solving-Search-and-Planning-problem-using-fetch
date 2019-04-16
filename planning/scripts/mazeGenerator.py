from collections import defaultdict
import numpy as np
import argparse
import random
import pprint
import json

def copy_empty_world(root_path):
    f_in = open(root_path+'/worlds/empty_world.sdf', 'r')
    f_out = open(root_path+'/worlds/maze.sdf', 'w')
    for line in f_in:
        f_out.write(line)
    f_in.close()
    return f_out

def add_walls_description(f_out):
    for i in range(1, 5):
        f_out.write('<model name=\'wall{}\'>\n'.format(i))
        f_out.write('<static>1</static>\n<link name=\'link\'>\n<pose>0 0 0.42 0 -0 0</pose>\n<collision name=\'collision\'>\n<geometry>\n<box>\n<size>7.5 0.2 2.8</size>\n</box>\n')
        f_out.write('</geometry>\n<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<ode/>\n</friction>\n</surface>\n</collision>\n')
        f_out.write('<visual name=\'visual\'>\n<cast_shadows>0</cast_shadows>\n<geometry>\n<box>\n<size>7.5 0.2 2.8</size>\n</box>\n</geometry>\n<material>\n<script>\n')
        f_out.write('<uri>model://grey_wall/materials/scripts</uri>\n<uri>model://grey_wall/materials/textures</uri>\n<name>vrc/grey_wall</name>\n</script>\n</material>\n</visual>\n<self_collide>0</self_collide>\n')
        f_out.write('<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n<pose>-0.779308 4.01849 0 0 -0 0</pose>\n</model>\n')

def add_walls(f_out, length):
    scale = (length+2)/7.5
    wall_dimensions = [(-1, length/2, -1.55905, scale, 1), (length/2, length+1, 0, scale, 1), (length+1, length/2, -1.55905, scale, 1), (length/2, -1, 0, scale, 1)]
    for i in range(4):
        f_out.write('<model name=\'wall{}\'>\n'.format(i+1))
        f_out.write('<pose>{} {} 0 0 -0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
        f_out.write('<link name=\'link\'>\n')
        f_out.write('<pose>{} {} 0.42 -0 0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
        f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n<wrench>0 0 0 0 -0 0</wrench>\n</link>\n</model>\n')
    return wall_dimensions

def add_cube_descriptions(f_out, coords):
    print(coords)
    for z, i in enumerate(coords):
        x, y = i
        print("cube description")
        print(z+1, x, y)
        f_out.write("<model name='cube_{0}'>\n".format(z+1))
        f_out.write('''<static>0</static>
            <link name='link'>
                <inertial>
                    <mass>0.05</mass>
                    <inertia>
                        <ixx>0.001</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.001</iyy>
                        <iyz>0</iyz>
                        <izz>0.001</izz>
                    </inertia>
                </inertial>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>0.063462 0.063462 0.063462</size>
                        </box>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>30</mu>
                                <mu2>30</mu2>
                            </ode>
                        </friction>
                        <contact>
                            <ode>
                                <kp>1e+06</kp>
                                <kd>100</kd>
                                <max_vel>1</max_vel>
                                <min_depth>0.002</min_depth>
                            </ode>
                        </contact>
                        <bounce/>
                    </surface>
                    <max_contacts>10</max_contacts>
                </collision>
                <visual name='visual'>
                    <geometry>
                        <box>
                            <size>0.063462 0.063462 0.063462</size>
                        </box>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/Blue</name>
                        </script>
                    </material>
                </visual>
                <velocity_decay>
                    <linear>0</linear>
                    <angular>0</angular>
                </velocity_decay>
                <self_collide>0</self_collide>
                <kinematic>0</kinematic>
                <gravity>1</gravity>
            </link>
            <pose>0 1 0 0 -0 0</pose>
            </model>''')
    f_out.write('<gui fullscreen=\'0\'>\n<camera name=\'user_camera\'>\n<pose>5 -5 2 0 0.275643 2.35619</pose>\n<view_controller>orbit</view_controller>\n</camera>\n</gui>\n')


def add_cube(f_out, x, y, index):
    f_out.write("<model name='cube_{0}'>\n".format(index))
    f_out.write("<pose>{0} {1} 0.029231 0 0 0</pose>\n</model>".format(x, y))


def cube_dict_generator(cubes, cubeCounter, location, coord1):
    cubes["cube_"+str(cubeCounter)]["loc"] = location
    cubes["cube_"+str(cubeCounter)]["load_loc"] = []
    cubes["cube_"+str(cubeCounter)]["load_loc"].append(coord1)
    # cubes["cube_"+str(cubeCounter)]["load_loc"].append(cord2)

def add_bloced_edges(x, y):
    blocked_list = []

    x_dec = 0.5
    y_dec = 0.5
    blocked_list.append((x-x_dec, y))
    blocked_list.append((x+x_dec, y))
    blocked_list.append((x, y-y_dec))
    blocked_list.append((x, y+y_dec))
    return blocked_list


def generate_blocked_edges(grid_dimension, total_cubes, seed, root_path, myscale=0.5):
    object_dict = {}
    cubes = {}
    np.random.seed(seed)
    blocked_edges = set()
    list_of_list_of_coords = []
    f_out = copy_empty_world(root_path)
    wall_dimensions = add_walls(f_out, grid_dimension*myscale)
    cubeCounter = 1
    
    count = 1
    coords = []
    while(count <= total_cubes):
        cubes["cube_"+str(cubeCounter)] = {}

        x = myscale*np.random.randint(1, (grid_dimension+1)//2)
        y = myscale*np.random.randint(1, (grid_dimension+1))
        cube_x = x + (random.randint(70,90)/100.0)
        cube_y = y

        if((cube_x <= grid_dimension//2) and ((x, y, x+0.5, y) not in blocked_edges) and ((x+0.5, y, x+1, y) not in blocked_edges)):

            blocked_edges.add((x, y, x+0.5, y))
            blocked_edges.add((x+0.5, y, x+1, y))
            blocked_edges.add((x+1, y, x+1.5, y))
            blocked_edges.add((x+0.5, y-0.5, x+0.5, y))
            blocked_edges.add((x+0.5, y, x+0.5, y+0.5))

            offset = 0.5
            coords.append((cube_x, cube_y))
            cube_dict_generator(cubes, cubeCounter, (cube_x, cube_y), (x, y))
            add_cube(f_out, cube_x, cube_y, cubeCounter)
            count += 1
            cubeCounter += 1        
    
    f_out.write('</state>')

    add_walls_description(f_out)
    add_cube_descriptions(f_out, coords)
        
    f_out.write('</world>\n</sdf>')
    f_out.close()

    object_dict["cubes"] = cubes
    place_locations = [[wall_dimensions[2][0]-1, wall_dimensions[1][1]-2], [wall_dimensions[2][0]-2, wall_dimensions[1][1]-1]]
    object_dict["place_loc"] = place_locations

    with open(root_path + '/cubes.json', 'w') as fp:
        json.dump(object_dict, fp)

    mazeInfo = [(0,grid_dimension, "EAST", myscale), blocked_edges]
    print blocked_edges
    return object_dict, mazeInfo


if __name__ == "__main__":
    pass
