# CustomVehiclePhysics
 UPBGE Vehicle Physics

Warning: this physics have some bugs

Example 1 countains:
-simple demo of vehicle physics.
-simple 3d model of vehicle.
-simple test map.

Example 2 countains:
-Demo of vehicle physics
-Nissan GTR r35 3d model
-Post processing like bloom, ambient occlusion, FXAA.
-Test map with a little road.
-Shader like a "PBR" on vehicle and roads.

You can customize the vehicle physisc editing wheel json file, changing class vehicle variables and wheel variables in your game component or if you prefer, inherit the vehicle class to change behavior.

To understand the structure of vehicle objects see Example 1, see objects physics panel, object parents, objects properties and object components. By default I created 2 components, VehicleComponent, to import the physics makes to work and PlayerControllerComponent, to set player inputs to the Vehicle class.

Main bugs:
-Deltatime problem, the car flicks with frametime variations, I think it needs substep but, I don't know how to implement.
-Tire friction at low velocity is not accurate.

if you want to improve the project, you can.
