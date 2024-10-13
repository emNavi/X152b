import xml.etree.ElementTree as ET

xml_content = '''
<dependencies>
    <build_depend>roscpp</build_depend>
    <build_depend>std_msgs</build_depend>
    <!-- <build_depend>traj_utils</build_depend> -->
    <build_depend>message_runtime</build_depend>
</dependencies>
'''

# 将字符串解析为 XML 树
root = ET.fromstring(xml_content)

# 查找并提取 <build_depend> 标签的文本内容
build_depends = []
for build_depend_elem in root.findall('.//build_depend'):
    build_depends.append(build_depend_elem.text)

# 输出提取的字段值
print("Build Depends:")
for dep in build_depends:
    print(dep)
