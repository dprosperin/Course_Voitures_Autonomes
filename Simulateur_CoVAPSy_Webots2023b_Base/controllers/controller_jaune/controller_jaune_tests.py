def get_mapped_angle(angle_unmapped):
    if angle_unmapped >= 0 and angle_unmapped <= 90:
        return 90 - angle_unmapped
    elif angle_unmapped >= 270 and angle_unmapped <= 359:
        return 450 - angle_unmapped
    else:
        return angle_unmapped

def get_unmapped_angle(angle_mapped):
    if angle_mapped >= 0 and angle_mapped <= 90:
        return 90 - angle_mapped
    elif angle_mapped >= 91 and angle_mapped <= 180:
        return 450 - angle_mapped

def get_angle_cap(angle_mapped):
    if angle_mapped >= 0 and angle_mapped <= 180:
        return map(angle_mapped, 0, 180, 16, -16)
    else:
        return 0

def map(x, in_min, in_max, out_min, out_max):
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

######### TESTS ###################
test_unmapped_angles = [i for i in range(90, -1, -1)] + [i for i in range(359, 269, -1)]

for i in range(0, 181):
    assert i == get_mapped_angle(test_unmapped_angles[i])

for i in range(0, 181):
    assert get_unmapped_angle(i) == test_unmapped_angles[i]