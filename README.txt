Codes done by Guoyao Shen for MEAM620 project 1-3.

(1) In the trajectory generator, the "main idea" is stop-and-go, means that the velocity and acceleration of each end of a piece of the trajectory is 0, and here use 5th interpolation to guatantee boundary acceleration can be 0.
(2) The file "detectcollision" is a file got from MEAM520, which is used to check collision given the vector.
(3) The generator also have same direction detect, meams that orientation within some error would be treated as same direction so that can delete some unnecessary points.
(4) For challenging map, please see "mymap.txt".
(5) For result of the "mamap.txt", please see "mamapresult.png". It's done under the condition of:
	map = load_map('maps/mymap.txt', 1.0, 1.0, 0.25);
	start = [10.0, 2.0, 10.0];
	stop = [10.0, 90.0, 40.0];
	and average velocity setting at 5, same direction error at 4, points distance at 20 in 	deleting points refinement in trajectory generator.