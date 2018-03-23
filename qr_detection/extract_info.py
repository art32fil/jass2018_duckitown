#!/usr/bin/env python
def extract_info(dec_digit):
    digit = bin(dec_digit)
    array = []
    two_last = digit[-2:]
    if int(two_last, 2) == 0: array.append("up")
    elif int(two_last, 2) == 1: array.append("left")
    elif int(two_last, 2) == 2: array.append("down")
    else: array.append("right")
    cross = digit[-5:-2]
    if int(cross, 2) == 0: array.append("X")
    elif int(cross, 2) == 1: array.append("T-up")
    elif int(cross, 2) == 2: array.append("T-left")
    elif int(cross, 2) == 3: array.append("T-down")
    else: array.append("T-right")
    num = digit[:-5]
    array.append(int(num, 2))
    array.reverse()
    return array

a,b,c = extract_info(143)
print(str(a)+b+c)
