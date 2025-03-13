


def set_direction_degre(angle_degre):
    if angle_degre > 16:
        angle_degre = 16
    elif angle_degre < -16:
        angle_degre = -16   
    angle = -angle_degre * 3.14/180

    print(angle)

set_direction_degre(-16)