import os
 
path="/home/nvidia/2022-09-26-16-24-25/pcd/"
path_result = path + "/result"
 
f=os.listdir(path)

try:
    os.mkdir(path_result)
except FileExistsError:
    pass
 
for i in f:
    print("i is",i)
    pre=i.split(".")
    print(pre[1])
    p=pre[0]
    q= int(p) + 1664176215
    name_pre=path+str(i)
    name_now=path + "result/" + str(q) + "." + str(pre[1]) + ".pcd"

    print(name_now)
    print(name_pre)
 
    os.rename(name_pre,name_now)
    print(name_pre,"is changed to",name_now)
