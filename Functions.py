
import random

def get_line_intersection(p1, p2, p3, p4):
    try:
        s1 = (p2[0] - p1[0], p2[1] - p1[1])
        s2 = (p4[0] - p3[0], p4[1] - p3[1])

        s = (-s1[1] * (p1[0] - p3[0]) + s1[0] * (p1[1] - p3[1])) / (-s2[0] * s1[1] + s1[0] * s2[1]);
        t = ( s2[0] * (p1[1] - p3[1]) - s2[1] * (p1[0] - p3[0])) / (-s2[0] * s1[1] + s1[0] * s2[1]);

        if (s >= 0 and s <= 1 and t >= 0 and t <= 1):

            i_x = p1[0] + (t * s1[0])
            i_y = p1[1] + (t * s1[1])
            return i_x, i_y # Collision detected
        else:
            return None, None # No collision
    except:
        return None, None

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def truncate(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    s = '{}'.format(f)
    if 'e' in s or 'E' in s:
        return '{0:.{1}f}'.format(f, n)
    i, p, d = s.partition('.')
    return '.'.join([i, (d+'0'*n)[:n]])

def invert_bools(bool_list):
    print bool_list
    if not all(isinstance(item, int) for item in bool_list):
        raise AssertionError('bool list must contain only ints')
        
    if max(bool_list) > 1:
        raise AssertionError('bool list must have maximum value of 1')

    if bool_list.count(1) != 1:
        raise AssertionError('bool list must contain a single 1')

    new_list = bool_list[:]
    index_list = []
    for i in range(0,len(new_list)):
        if new_list[i] == 1:
            new_list[i] = 0
        elif new_list[i] == 0:
            index_list.append(i)
            
    index = random.choice(index_list)
    new_list[index] = 1
    return new_list
            
def make_binary(start_list):

    new_list = start_list[:]
    index_list = []
    minVal = min(new_list)
    maxVal = minVal
    maxIndex = 0
    for i in range(0,len(new_list)):
        if new_list[i] > maxVal:
            maxIndex = i
    for i in range(0,len(new_list)):
        if i == maxIndex:
            new_list[i] = 1
        else:
            new_list[i] = 0
    return new_list
            

