    
step = 100
half = 8

num = []

last1 = 1100
last2 = 1100

for i in range(half):
    if i % 2 == 0:
        num.append(last1)
        last1 += step
    else:
        last2 -= step
        num.append(last2) 
        
print(num)