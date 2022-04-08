##########

def CTE(q):
    T = []
    for i in range(q,5):
        T.append(i+2**2) 
    #print("CTE is %f" %T[i])
    return T


if _name_ == '_main_':
    p = CTE(1)
    print(p)


##########
def CTE(q):
    T = []
    for i in range(5):
        T.append(i+2**2) 
    #print("CTE is %f" %T[i])
    return T


if _name_ == '_main_':
    p = CTE(1)
    print(p)