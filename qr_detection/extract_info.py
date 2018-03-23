#!/usr/bin/env python
def extract_info(dec_digit):
    digit = bin(dec_digit)
    digit_len = len(digit)-2
    cnt_zero = 16 - digit_len
    zeros = ''
    for i in range(cnt_zero):
        zeros += '0'
    t = digit.split('0b')
    digit = '0b' + zeros + t[1]
    print(digit)
    array = []
    two_last = digit[-2:]
    if int(two_last, 2) == 0: array.append("up")
    elif int(two_last, 2) == 1: array.append("right")
    elif int(two_last, 2) == 2: array.append("down")
    else: array.append("left")
    cross = digit[-5:-2]
    if int(cross, 2) == 0: array.append("X")
    elif int(cross, 2) == 1: array.append("T-down")
    elif int(cross, 2) == 2: array.append("T-left")
    elif int(cross, 2) == 3: array.append("T-up")
    else: array.append("T-right")
    num = digit[:-5]
    array.append(int(num, 2))
    array.reverse()
    return array

print(extract_info(2))
