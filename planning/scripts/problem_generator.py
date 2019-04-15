import pickle
def write_objects(fhandle,object_dict):
	book_list = object_dict["books"].keys()
	book_loc_list = [book_name + "_iloc" for book_name in book_list]
	bins_list = object_dict["bins"].keys()
	bins_loc_list = [bins_name + "_iloc" for bins_name in bins_list]
	book_list_str = " ".join(book for book in book_list) + " - book"
	book_loc_str = " ".join(book_loc for book_loc in book_loc_list)  + " - location"
	bin_list_str = " ".join(bin_name for bin_name in bins_list) + " - bin"
	bin_loc_str = " ".join(bin_loc for bin_loc in bins_loc_list) + " - location"
	subject_set = set()
	for book in book_list:
		subject_set.add(object_dict["books"][book]["subject"].replace(" ","_"))
	subject_str = " ".join(sub_name for sub_name in subject_set) + " - subject"
	fhandle.write("(:objects" + "\n")
	fhandle.write("tbot3 - robot" + "\n")
	fhandle.write("tbot3_init_loc - location" + "\n")
	fhandle.write(book_list_str + "\n")
	fhandle.write(bin_list_str + "\n")
	fhandle.write(book_loc_str + "\n")
	fhandle.write(bin_loc_str + "\n")
	fhandle.write(subject_str + "\n")
	fhandle.write("small large - size" + "\n")
	fhandle.write(")" + "\n")
	return book_list,book_loc_list,bins_list,bins_loc_list

def write_init_state(fhandle,object_dict,book_list,book_loc_list,bins_list,bins_loc_list):
	fhandle.write("(:init"+"\n")
	for i in range(len(book_list)):
		fhandle.write("(Book_At {} {})".format(book_list[i],book_loc_list[i]) + "\n")
	for i in range(len(bins_list)):
		fhandle.write("(Bin_At {} {})".format(bins_list[i],bins_loc_list[i]) + "\n")
	for book in book_list:
		fhandle.write("(Book_Subject {} {})".format(book,object_dict["books"][book]["subject"].replace(" ","_")) + "\n")
		fhandle.write("(Book_Size {} {})".format(book,object_dict["books"][book]["size"]) + "\n")
	for bin_name in bins_list:
		fhandle.write("(Bin_Subject {} {})".format(bin_name,object_dict["bins"][bin_name]["subject"].replace(" ","_")) + "\n")
		fhandle.write("(Bin_Size {} {})".format(bin_name,object_dict["bins"][bin_name]["size"]) + "\n")
	fhandle.write("(Robot_At tbot3 tbot3_init_loc)" + "\n")
	fhandle.write("(Empty_Basket tbot3)"  + "\n")
	fhandle.write(")" + "\n")





def write_pddl(path,object_dict):
	fhandle = open(path,"w")
	fhandle.write("(define (problem p01)\n")
	fhandle.write("(:domain bookWorld)\n")
	book_list,book_loc_list,bins_list,bins_loc_list = write_objects(fhandle,object_dict)
	write_init_state(fhandle,object_dict,book_list,book_loc_list,bins_list,bins_loc_list)
	fhandle.write("(:goal ENTER YOUR GOAL FORMULA HERE )" + "\n")
	fhandle.write(")")
	fhandle.close()


if __name__ == "__main__":
	# # object_dict = {"books" : { "b1" : { "size" : "large","subject" : "s1" , "loc" : (1,2)},\
	# 							"b2" : { "size" : "small","subject" : "s2" , "loc" : (2,2)}\
	# 						},\
	# 				"bins" : {"bin1" : { "size" : "large","subject" : "s1" , "loc" : (3,2)},\
	# 							"bin2" : { "size" : "small","subject" : "s2" , "loc" : (4,2)}\
	# 						}\
	# 			}
	object_dict = pickle.load(open("/home/naman/catkin_ws/src/planning/object_dict.p","rb"))
	fhandle = open("temp_problem.pddl","w")
	write_pddl(fhandle,object_dict)
