# Cloth Simulation
<p>
<img src="http://www.w3.org/1999/xhtml" width="800px" />
</p>
For this project I made a real-time cloth simulator that realistically simulates the movement of cloth. My simulator uses a point mass and spring system that defines the structure and behavior of the cloth by using springs with various physical constraints that connect between point masses to form a wireframe grid. I used Verlet integration to simulate the movement of cloth over time by computing each point mass's next position based on the point mass's current position, velocity, acceleration, applied forces. I also included the handling of collisions between the cloth and spheres and planes by testing for intersections and updating each point mass's position to lie on the surface of the other object. Additionally, I implemented real-time handling of self-collisions so the cloth can realistically fold onto itself by using spatial hashing to ensure fast search times for finding collisions between point masses. Lastly, I created a few GLSL shader programs in order to direct raytracing computations to the GPU rather than the CPU so that my cloth simulation could be interactive and in real-time. These customized shader programs include support for diffuse shading, Blinn-Phong shading, texture mapping, bump mapping, displacement mapping, and environment-mapped reflections. 

For more information on this project, please visit my personal website:
https://cal-cs184-student.github.io/p4-clothsim-sp20-vaustin6203/
