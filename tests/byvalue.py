
def set_list(list):
	list = ["A", "B", "C"]
	return list

def add(list):
	list.append("D")
	return list

my_list = ["E"]
add(my_list)
set_list(my_list)
print(my_list)


def set_list(list):
	list = 5
	return list

def add(list):
	list += 1
	return list

my_list = 2
add(my_list)
set_list(my_list)
print(my_list)
