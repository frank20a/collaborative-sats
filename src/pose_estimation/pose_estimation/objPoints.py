from numpy import array, float32


# Marker side dimension in meters
m = 0.09
# Target box dimensions in meters
x = 0.10
y = 0.10
z = 0.20
# objPoints for the target, ids from 0 to 5 ascending
marker_cube_1 = [
    # Marker 0 (Top)f
    array([
        [  m/2,  m/2, z/2 ],
        [  m/2, -m/2, z/2 ],
        [ -m/2, -m/2, z/2 ],
        [ -m/2,  m/2, z/2 ],
    ], dtype=float32),
    
    # Marker 1 (Right)f
    array([
        [ -m/2, -y/2,  m/2 ],
        [  m/2, -y/2,  m/2 ],
        [  m/2, -y/2, -m/2 ],
        [ -m/2, -y/2, -m/2 ],
    ], dtype=float32),
    
    # Marker 2 (Back)f
    array([
        [ x/2, -m/2,  m/2 ],
        [ x/2,  m/2,  m/2 ],
        [ x/2,  m/2, -m/2 ],
        [ x/2, -m/2, -m/2 ],
    ], dtype=float32),

    # Marker 3 (Front) f
    array([
        [ -x/2,  m/2,  m/2 ],
        [ -x/2, -m/2,  m/2 ],
        [ -x/2, -m/2, -m/2 ],
        [ -x/2,  m/2, -m/2 ],
    ], dtype=float32),

    
    # Marker 4 (Bottom)f
    array([
        [  m/2,  m/2, -z/2 ],
        [ -m/2,  m/2, -z/2 ],
        [ -m/2, -m/2, -z/2 ],
        [  m/2, -m/2, -z/2 ],
    ], dtype=float32),

    # Marker 5 (Left) f
    array([
        [  m/2,  y/2,  m/2 ],
        [ -m/2,  y/2,  m/2 ],
        [ -m/2,  y/2, -m/2 ],
        [  m/2,  y/2, -m/2 ],
    ], dtype=float32),
]

# =================================================================
# Dictionary that contains all the models from above
models = {
    'marker_cube_1': marker_cube_1,
}