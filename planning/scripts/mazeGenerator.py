from collections import defaultdict
import numpy as np
import argparse
import random
import pprint
import json


subject_list = ["Operating Systems Architecture","Artificial Intelligence","Information Security",
                "Computer Networks I","Computer Graphics","Cloud Computing",
                "The Software Process","Reasoning About Programs","Mobile Computing","Advanced Database Systems",
                "Web Information Systems","Cloud Computing","Computer Systems Principles and Programming","Human Computer Interaction",
                "Design Computing","Relational Database Systems","Algorithms and Data Structures","Programming in the Large",
                "Introduction to Computer Systems"] 

color_list = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.1725490196078432, 0.5562), (1.0, 0.9254901960784314, 0.8980392156862745), 
                (0.6340, 0.9490196078431372, 0.6780392156862745), (1.0, 0.5764705882352941, 0.3980392156862745), (1.0, 1.0, 0.8980392156862745), 
                (0.9764705882352941, 1.0, 0.8980392156862745), (0.9490196078431372, 1.0, 0.8980392156862745), (0.9254901960784314, 1.0, 0.8980392156862745), 
                (0.8980392156862745, 1.0, 0.8980392156862745), (0.8980392156862745, 1.0, 0.9254901960784314), (0.8980392156862745, 0.9490196078431372, 1.0), 
                (0.8980392156862745, 0.9254901960784314, 1.0), (0.8980392156862745, 0.8980392156862745, 1.0), (1.0, 0.8980392156862745, 0.8980392156862745), 
                (1.0, 0.8980392156862745, 1.0), (1.0, 0.9019607843137255, 0.9490196078431372), (1.0, 0.8980392156862745, 0.9254901960784314)]

def copy_empty_world(root_path):
    f_in = open(root_path+'/worlds/new_empty_world.sdf', 'r')
    f_out = open(root_path+'/worlds/new_maze.sdf', 'w')
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
        # f_out.write('<scale>{} {} 0.03</scale>\n'.format(wall_dimensions[i][3], wall_dimensions[i][4]))
        f_out.write('<link name=\'link\'>\n')
        f_out.write('<pose>{} {} 0.42 -0 0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
        f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n<wrench>0 0 0 0 -0 0</wrench>\n</link>\n</model>\n')

def add_book_description(f_out, coords, color, bookCounter):
    for z, i in enumerate(coords):
        x, y = i
        f_out.write("<model name='cube_{0}'>\n".format(bookCounter+1))
        f_out.write('''<static>0</static>
            <link name='link'>
                <inertial>
                    <mass>0.25</mass>
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
                            <size>0.06 0.06 0.06</size>
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
                            <size>0.06 0.06 0.06</size>
                        </box>
                    </geometry>
                    <material>
                        <script>
                            <uri>file://media/materials/scripts/gazebo.material</uri>
                            <name>Gazebo/Red</name>
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
        # f_out.write("<link name='cover'>\n<pose>0 -0.000108 0.015405 0 -0 0</pose>\n<self_collide>0</self_collide>\n<kinematic>0</kinematic>\n")
        # f_out.write("<gravity>1</gravity>\n<visual name='visual'>\n<geometry>\n<box>\n<size>0.245 0.16 0.03</size>\n</box>\n</geometry>\n<material>\n<script>\n")
        # f_out.write("<uri>model://book_1/materials/scripts/book_{0}.material</uri>\n<uri>model://book_1/materials/textures/cover{0}.jpg</uri>\n<name>book_{0}</name>\n</script>\n</material>\n<cast_shadows>1</cast_shadows>\n<transparency>0</transparency>\n</visual>\n<collision name='collision'>\n<laser_retro>0</laser_retro>\n<max_contacts>10</max_contacts>\n<geometry>\n<box>\n<size>0.245 0.16 0.03</size>\n</box>\n</geometry>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<ode><mu>1000</mu><mu2>1000</mu2></ode>\n</friction>\n</surface>\n</collision>\n<inertial>\n<inertia>\n<ixx>0.00058</ixx>\n<ixy>0</ixy>\n<ixz>0</ixz>\n<iyy>0.00058</iyy>\n<iyz>0</iyz>\n<izz>0.00019</izz>\n</inertia>\n<mass>0.05</mass>\n</inertial>\n</link>\n<link name='page'>\n<pose>0 0.000608 0.015405 0 -0 0</pose>\n<visual name='visual'>\n<pose>0 0 0 0 -0 0</pose>\n<geometry>\n<box>\n<size>0.24502 0.15862 0.028</size>\n</box>\n</geometry>\n<material>\n<ambient>1 1 1 1</ambient>\n<diffuse>1 1 1 1</diffuse>\n<specular>0.01 0.01 0.01 1</specular>\n<emissive>0 0 0 1</emissive>\n<shader type='vertex'>\n<normal_map>__default__</normal_map>\n</shader>\n</material>\n<cast_shadows>1</cast_shadows>\n<transparency>0</transparency>\n</visual>\n<collision name='collision'>\n<laser_retro>0</laser_retro>\n<max_contacts>10</max_contacts>\n<pose>0 0 0 0 -0 0</pose>\n<geometry>\n<box>\n<size>0.245 0.16 0.03</size>\n</box>\n</geometry>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<ode><mu>1000</mu>\n<mu2>1000</mu2>\n</ode>\n</friction>\n</surface>\n</collision>\n<self_collide>0</self_collide>\n<inertial>\n<inertia>\n<ixx>0.00058</ixx>\n<ixy>0</ixy>\n<ixz>0</ixz>\n<iyy>0.00058</iyy>\n<iyz>0</iyz>\n<izz>0.00019</izz>\n</inertia>\n<mass>0.05</mass>\n</inertial>\n<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n<static>0</static>\n<allow_auto_disable>1</allow_auto_disable>\n<pose>0.830691 0.858956 0 0 -0 0</pose>\n</model>".format(color, x, y))
        bookCounter += 1
    f_out.write('<gui fullscreen=\'0\'>\n<camera name=\'user_camera\'>\n<pose>5 -5 2 0 0.275643 2.35619</pose>\n<view_controller>orbit</view_controller>\n</camera>\n</gui>\n')

#change in dimenssion of the book is handled in this function.
def add_book(f_out, x, y, book_size_scale, bookCounter):
    f_out.write("<model name='cube_{0}'>\n".format(bookCounter))
    f_out.write("<pose>{0} {1} 0.029231 0 0 0</pose>\n</model>".format(x, y))
    # f_out.write("<scale>{0} {0} 1</scale>\n".format(book_size_scale))
    # f_out.write("<link name='cover'>\n")
    # f_out.write("<pose>{0} {1} 0.015 -1e-06 1e-06 0</pose>\n".format(x, y-0.000108))
    # f_out.write("<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0.017626 0.011511 -0.205341 -0.7674 1.17508 -0</acceleration>\n<wrench>0.017626 0.011511 -0.205341 0 -0 0</wrench>\n</link>\n<link name='page'>\n")
    # f_out.write("<pose>{0} {1} 0.015 0 1e-06 0</pose>\n".format(x, y+0.000608))
    # f_out.write("<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 -9.8 0 -0 0</acceleration>\n<wrench>0 0 -9.8 0 -0 0</wrench>\n</link>\n</model>")
    

def add_trolly_description(f_out, coords):
    subject_count = -1
    for z, i in enumerate(coords):
        x, y = i
        if z % 2 == 0:
            scale = 0.5
            subject_count += 1
        else:
            scale = 0.3
        f_out.write("<model name='trolly_{0}\'>\n".format(z+1))
        f_out.write("<link name='link'>\n<pose>0 0 0 0 -0 0</pose>\n<inertial>\n<mass>1</mass>\n")
        f_out.write("<pose>0 0 0 0 -0 0</pose>\n<inertia>\n<ixx>1</ixx>\n<ixy>0</ixy>\n<ixz>0</ixz>\n<iyy>1</iyy>\n<iyz>0</iyz>\n<izz>1</izz>\n</inertia>\n</inertial>\n")
        f_out.write("<self_collide>0</self_collide>\n<kinematic>0</kinematic>\n<gravity>1</gravity>\n<visual name='visual'>\n<geometry>\n")
        f_out.write("<mesh>\n<uri>model://bookcart/meshes/bookcart.dae</uri>\n</mesh>\n</geometry>\n<pose>0 0 0 0 -0 0</pose>\n")
        f_out.write("<cast_shadows>1</cast_shadows>\n<transparency>0</transparency>\n<material>\n<ambient>{0} {1} {2} 0</ambient>\n<diffuse>{0} {1} {2} 0</diffuse>\n<specular>0 0 0 1</specular>\n<emissive>0 0 0 1</emissive>\n".format(color_list[subject_count][0], color_list[subject_count][1], color_list[subject_count][2]))
        f_out.write("<script>\n<name>ModelPreview_1::link::visual_MATERIAL_</name>\n<uri>__default__</uri>\n</script>\n<shader type='vertex'>\n<normal_map>__default__</normal_map>\n</shader>\n</material>\n</visual>\n<collision name='collision_0'>\n<laser_retro>0</laser_retro>\n<max_contacts>10</max_contacts>\n<pose>0 0 0 0 -0 0</pose>\n<geometry>\n<mesh>\n<uri>model://bookcart/meshes/bookcart.dae</uri>\n</mesh>\n</geometry>\n<surface>\n<friction>\n<ode>\n<mu>1</mu>\n<mu2>1</mu2>\n<fdir1>0 0 0</fdir1>\n<slip1>0</slip1>\n<slip2>0</slip2>\n</ode>\n</friction>\n<bounce>\n<restitution_coefficient>0</restitution_coefficient>\n<threshold>1e+06</threshold>\n</bounce>\n<contact>\n<collide_without_contact>0</collide_without_contact>\n<collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n<ode>\n")
        f_out.write("<soft_cfm>0</soft_cfm>\n<soft_erp>0.2</soft_erp>\n<kp>1e+13</kp>\n<kd>1</kd>\n<max_vel>0.01</max_vel>\n<min_depth>0</min_depth>\n</ode>\n<bullet>\n<split_impulse>1</split_impulse>\n<split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n<soft_cfm>0</soft_cfm>\n<soft_erp>0.2</soft_erp>\n<kp>1e+13</kp>\n<kd>1</kd>\n</bullet>\n</contact>\n</surface>\n</collision>\n</link>\n<static>1</static>\n<allow_auto_disable>1</allow_auto_disable>\n<pose>1.06777 -0.068202 0 0 -0 0</pose>\n</model>")

def add_trolly(f_out, x, y, scale, trollies_count):
    f_out.write("<model name='trolly_{0}'>".format(trollies_count))
    f_out.write("<pose>{0} {1} 0 0 -0 0</pose>".format(x, y))
    # f_out.write("<scale>{0} {0} 0.35</scale>".format(scale))
    f_out.write("<link name='link_20'>")
    f_out.write("<pose>{0} {1} 0 0 -0 0</pose>".format(x, y))
    f_out.write("<velocity>0 0 0 0 -0 0</velocity>")
    f_out.write("<acceleration>0 0 0 0 -0 0</acceleration>")
    f_out.write("<wrench>0 0 0 0 -0 0</wrench>")
    f_out.write("</link>")
    f_out.write("</model>")




def book_dict_generator(books, bookCounter, size, location, coord1, cord2, book_index):
    print(bookCounter)
    books["book_"+str(bookCounter)]["size"] = size
    # Total book count of a subject including small and large
    books["book_"+str(bookCounter)]["subject"] = subject_list[book_index]
    books["book_"+str(bookCounter)]["loc"]= location
    books["book_"+str(bookCounter)]["load_loc"] = []
    books["book_"+str(bookCounter)]["load_loc"].append(coord1)
    books["book_"+str(bookCounter)]["load_loc"].append(cord2)

def trolly_dict_generator(books, trollies_counter, size, access_loc_list, location, subject_count_index):
    books["trolly_"+str(trollies_counter)]["size"] = size
    books["trolly_"+str(trollies_counter)]["subject"] = subject_list[subject_count_index]
    books["trolly_"+str(trollies_counter)]["loc"]= location
    books["trolly_"+str(trollies_counter)]["load_loc"] = access_loc_list




def add_bloced_edges(x, y):
    blocked_list = []

    x_dec = 0.5
    y_dec = 0.5
    blocked_list.append((x-x_dec, y))
    blocked_list.append((x+x_dec, y))
    blocked_list.append((x, y-y_dec))
    blocked_list.append((x, y+y_dec))

    return blocked_list


def generate_blocked_edges(grid_dimension, list_of_number_of_books, seed,  number_of_trollies, root_path, myscale=0.5):
    object_dict = {}
    books = {}
    np.random.seed(seed)
    blocked_edges = set()
    list_of_list_of_coords = []
    f_out = copy_empty_world(root_path)
    add_walls(f_out, grid_dimension*myscale)
    book_size_scale = 1
    bookCounter = 1
    subject_count = -1
    for book_index, book_count in enumerate(list_of_number_of_books):
        if book_index % 2 == 0:
            subject_count += 1
        book_size_scale = 1
        size = "large"
        if book_index%2 ==0:
            book_size_scale = 0.5
            size = "small"


        n_obstacles = book_count
        count = 1
        coords = []
        while(count <= n_obstacles):
            books["book_"+str(bookCounter)] = {}
            x = myscale*np.random.randint(1, (grid_dimension+1)//2)
            y = myscale*np.random.randint(1, (grid_dimension+1))
            flag = np.random.randint(0, 2)
            if(flag == 0 and ((x+myscale) <= grid_dimension*myscale//2) and ((x, y, x+myscale, y) not in blocked_edges)):
                blocked_edges.add((x, y, x+myscale, y))
                # offset = np.random.uniform(0, 0.05*0)
                offset = 0.5
                coords.append((x+myscale/2+offset, y))
                book_dict_generator(books, bookCounter, size, (x+myscale/2+offset, y), (x, y), (x+myscale, y),  subject_count)
                add_book(f_out, x+myscale/2+offset, y, book_size_scale, bookCounter)
                count += 1
            
            elif(flag == 1 and ((y+myscale) <= grid_dimension*myscale//2) and ((x, y, x, y+myscale) not in blocked_edges)):
                blocked_edges.add((x, y, x, y+myscale))
                # offset = np.random.uniform(0, 0.05*myscale)
                offset = 0.5
                coords.append((x, y+myscale/2-offset))
                book_dict_generator(books, bookCounter, size, (x, y+myscale/2-offset), (x, y), (x, y+myscale),  subject_count)
                add_book(f_out, x, y+myscale/2-offset, book_size_scale, bookCounter)
                count += 1
            else:
                bookCounter -= 1
            bookCounter += 1
        list_of_list_of_coords.append(coords)


    ntrollies = number_of_trollies
    trolliesCoords = []
    trollies_count = 1
    x = grid_dimension//4+ 1
    y = 1
    subject_count = -1
    trollies = {}
    while(trollies_count <= ntrollies):

        
        scale = 0.4
        size = "large"
        if trollies_count%2 ==0:
            scale = 0.3
            size = "small"
        trollies["trolly_"+str(trollies_count)] = {}

        # To get subject name
        if (trollies_count-1) % 2 == 0:
            subject_count += 1

        trolliesCoords.append((x,y))
        blocked_list = add_bloced_edges(x, y)
        for blocked_edge in blocked_list:
            if x > blocked_edge[0] or (x == blocked_edge[0] and y > blocked_edge[0]):
                blocked_edges.add((blocked_edge[0], blocked_edge[1], x, y))
            else :
                blocked_edges.add(( x, y, blocked_edge[0], blocked_edge[1]))
        trolly_dict_generator(trollies, trollies_count, size, blocked_list, (x, y), subject_count)
        add_trolly(f_out, x, y,scale, trollies_count)
        trollies_count += 1
        if (trollies_count-1) % 3 == 0:
            x += 2
            y = 1
        else:
            y += 2

    
    f_out.write('</state>')

    add_trolly_description(f_out, trolliesCoords)
    add_walls_description(f_out)
    #color list will decide color of the book. R, G, B, X(need to check)
    
    for book_index, book_count in enumerate(list_of_number_of_books):
        bookCounter = book_index * book_count
        add_book_description(f_out, list_of_list_of_coords[book_index], book_index+1, bookCounter)
        
    f_out.write('</world>\n</sdf>')
    f_out.close()

    object_dict["books"] = books
    object_dict["bins"] = trollies
    with open(root_path + '/books.json', 'w') as fp:
        json.dump(object_dict, fp)

    mazeInfo = [(0,grid_dimension, "EAST", myscale), blocked_edges]
    print grid_dimension
    return object_dict, mazeInfo


if __name__ == "__main__":
    #robot height = 0.4m 
    #trolly height = 1m

    
    subject_count = 6
    book_sizes = 2
    book_count_of_each_size = 5
    book_count_of_each_subject = book_count_of_each_size * book_sizes
    book_count_list = [book_count_of_each_size] * subject_count * book_sizes
    number_of_trollies = 1 #subject_count * 2
    grid_size = max((((book_count_of_each_subject * subject_count) / 4) // 1 ) + 1, ((number_of_trollies/4)*7), 10)

    root_path = "/home/ketan/catkin_ws/src/search"

    books, mazeInfo = generate_blocked_edges(grid_size, book_count_list, 2,  number_of_trollies, root_path,0.5)

    # pprint.pprint(books)
    print(mazeInfo)
