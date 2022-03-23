# Notes about modelling visuals for Gazebo

## Using OGRE materials

Add the following line to your `~/.bashrc` file
`export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:~/dev-ws/models:${GAZEBO_RESOURCE_PATH}`

Create a `/media` folder inside `/models` with the following structure
```
/media
    /scripts
        mat_1.material
        mat_2.material
        ...
    /textures
        text_1.png
        text_2.jpg
        ...
    /meshes
        mesh_1.dae
        mesh_2.obj
        ...
```