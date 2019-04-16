import pickle
def write_objects(fhandle,object_dict):
	cube_list = object_dict["cubes"].keys()
	cube_loc_list = [cube_name + "_iloc" for cube_name in cube_list]
	cube_list_str = " ".join(cube for cube in cube_list) + " - cube"
	cube_loc_str = " ".join(cube_loc for cube_loc in cube_loc_list)  + " - location"
	fhandle.write("(:objects" + "\n")
	fhandle.write("fetch - robot" + "\n")
	fhandle.write(cube_list_str + "\n")
	fhandle.write("place_area_location - location\n")
	fhandle.write("fetch_init_loc - location" + "\n")
	fhandle.write(cube_loc_str + "\n")
	fhandle.write(")" + "\n")
	return cube_list, cube_loc_list

def write_init_state(fhandle,object_dict,cube_list,cube_loc_list):
	fhandle.write("(:init"+"\n")
	for i in range(len(cube_list)):
		fhandle.write("(Cube_At {} {})".format(cube_list[i],cube_loc_list[i]) + "\n")
	fhandle.write("(Robot_At fetch fetch_init_loc)" + "\n")
	fhandle.write("(Empty_Basket fetch)"  + "\n")
	fhandle.write(")" + "\n")

def write_pddl(path,object_dict):
	fhandle = open(path,"w")
	fhandle.write("(define (problem p01)\n")
	fhandle.write("(:domain cubeWorld)\n")
	cube_list, cube_loc_list = write_objects(fhandle,object_dict)
	write_init_state(fhandle,object_dict,cube_list,cube_loc_list)
	goal = ''
	for cube in cube_list:
		goal += '(' + 'Cube_At' + ' ' + cube + ' ' +  'place_area_location' + ')' + ' '
	fhandle.write("(:goal " + '('+ 'and' + ' ' + goal + ')' + " )" + "\n")
	fhandle.write(")")
	fhandle.close()


if __name__ == "__main__":
	pass
