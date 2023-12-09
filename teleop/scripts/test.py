def map_angle(angle, from_range: list, to_range: list):

    #assert angle >= from_range[0] and angle <= from_range[1], "Angle must be in from_range"
            
    mapped_angle = (angle - from_range[0]) * (to_range[1] - to_range[0]) / (from_range[1] - from_range[0]) + to_range[0]
    return mapped_angle

print(map_angle(-0.5, [1, 2], [2, 4]))