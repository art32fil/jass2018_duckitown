
def extract_info(dec_digit):
    digit = bin(dec_digit)
    array = []
    two_last = digit[-2:]
    if int(two_last, 2) == 0: array.append(0)
    elif int(two_last, 2) == 1: array.append(1)
    elif int(two_last, 2) == 2: array.append(2)
    else: array.append(3)
    cross = digit[-5:-2]
    if int(cross, 2) == 0: array.append(0)
    elif int(cross, 2) == 1: array.append(1)
    elif int(cross, 2) == 2: array.append(2)
    elif int(cross, 2) == 3: array.append(3)
    else: array.append(4)
    num = digit[:-5]
    array.append(int(num, 2))
    array.reverse()
    return array


print(extract_info(143))
