
def set_list(list): # by value
	list = ["A", "B", "C"]
	return list

my_list = ["X","Y"]

new_list = set_list(my_list)

print(new_list)


def add(list): # by reference
	list.append("D")
	return list

my_list = ["E"]

new_list = add(my_list)

print(new_list)

