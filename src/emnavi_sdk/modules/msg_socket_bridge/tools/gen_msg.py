base_type = {
    "string" , "float64","float32","int64","int32","uint32","time"
}
msg_dict = {}

# Head 是 std_msgs

# Find Msg
def read_install_msg_file(pkg_name, msg_name):
    msg_item = []
    ros_version = "noetic"
    try:
        with open("/opt/ros/{}/share/{}/msg/{}.msg".format(ros_version, pkg_name, msg_name), 'r') as file:
            # 逐行读取文件内容
            lines = file.readlines()
            for line in lines:
                cleaned_line = line.strip()
                if(line.startswith('#') is False and cleaned_line):
                    print(line.strip())  # strip() 方法用于去除行尾的换行符
                    msg_item.append([cleaned_line.split(" ")[0],cleaned_line.split(" ")[1]])
    except FileNotFoundError:
        print(f"Error: The file '{pkg_name}' '{msg_name}'does not exist.")
        return False,[]
    return True,msg_item

def recursion_find_type_2(msg_class_name,parent_class_name=["std_msgs"]):
    global msg_dict
    has_parent=" "
    if( msg_class_name not in msg_dict):
        if( msg_class_name not in base_type):
            new_name = msg_class_name.split("/")
            if(len(new_name)>1):
                has_parent = new_name[0]
                flag,msg_item_temp = read_install_msg_file(msg_class_name.split("/")[0],msg_class_name.split("/")[1])
            else:
                print(f"parent_class_name '{parent_class_name}' ")
                # flag,msg_item_temp = read_install_msg_file(parent_class_name.split("/")[0],msg_class_name)
                # if(flag is False):
                for j in parent_class_name:
                    flag,msg_item_temp = read_install_msg_file(j,msg_class_name)
                    if(flag is True):
                        break
            msg_dict[new_name[0]] = msg_item_temp
            for i in msg_item_temp:
                    if(has_parent == " "):
                        recursion_find_type_2(i[0],parent_class_name)
                    else:
                        recursion_find_type_2(i[0],[has_parent] + parent_class_name)

                    
    
    
recursion_find_type_2("nav_msgs/Odometry")
print(msg_dict)

# flag,msg_item = read_install_msg_file("nav_msgs","Odometry")
# print(msg_item)
# for i in msg_item:
#     print(i)
#     if(i[0] not in msg_dict):
#         if(i[0] not in base_type):
#             print("not have")

# print(msg_dict)
# for i in msg_item:
#     print(i)
#     if(i[0] in msg_dict):
#         print("have")
# Read Msg


# Create Msg
