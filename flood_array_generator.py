import numpy as np

np.set_printoptions(threshold=np.inf)  # Disable truncation

def generate_pattern(cells):
    """Generates an n x n array following the quadrant-based pattern logic."""
    n = cells*2 + 1
    array = np.zeros((n, n), dtype=int)
    center = cells  # Find the center of the grid

    for i in range(0,cells,2):
        for j in range(0,cells,2):
            value = (j+i)/2
            array[center + j + 1, center - i -1] = value
            array[center + j + 1, center + i + 1] = value
            array[center - j -1, center - i -1] = value
            array[center - j -1, center + i + 1] = value
    for i in range(0,cells,2):
        array[0 , center - i -1] = 1
        array[0 , center + i +1] = 1
        array[n-1, center - i -1] = 1
        array[n-1, center + i +1] = 1
    for j in range(0,cells,2):
        array[center + j+1 , 0] = 1
        array[center + j+1 , n-1 ] = 1
        array[center - j-1, 0 ] = 1
        array[center - j-1, n-1 ] = 1
    
    array[1, 1] = 100

    return array

# Generate a 33x33 array
array_33x33 = generate_pattern(4)
print(array_33x33)


