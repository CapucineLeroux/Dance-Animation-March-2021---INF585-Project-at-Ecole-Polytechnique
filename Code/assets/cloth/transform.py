f = open('initial_positions_30by30.txt','r')
s = f.read().split()
print(s)

f1 = open('initial_positions_15by15.txt','w')
n = 30
for i in range (0,n,2) :
    for j in range (0,n,2):
        f1.write(s[3*(n*i+j)]+' '+s[3*(n*i+j)+1]+' '+s[3*(n*i+j)+2]+' ')

f1.close()
