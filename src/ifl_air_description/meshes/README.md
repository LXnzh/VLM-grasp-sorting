
## Instruction to create mesh files

The process to create meshfiles is described [here](https://gitlab.kit.edu/kit/ifl/gruppen/air/air_wiki/-/wikis/ROS/Create-URDF-file-for-Custom-Robot-(Mesh-files-conversion)) in the AIR Wiki.



## Old instruction (requires subscription, or is not efficient)

To create meshes you can use Autodesk Fusion 360 and the Add-In [Simlab](https://apps.autodesk.com/FUSION/de/Detail/Index?id=8111537807083848754&appLang=en&os=Win64) to generate .dae files, which can be used in the URDF files. 

The texture may not be correctly taken over in the .dae file. But the .dae files can be openend with a text editor and you can search for the word diffuse to change the color.


Alternative Add-In is [Collada Writer](https://apps.autodesk.com/FUSION/de/Detail/Index?id=934783265519220722&appLang=en&os=Win64) to export dae files. It will take over also colors and materials, but export all bodies (also not visible ones), and the file size is very large (5x compared to simlab). Large file size is due to representing all values with > 10 digits precision, even integers. Similar filesize as simlab can be achieves by limiting the precision to 6 digitis and replacing all ".000000" with "" in the DAE file. 