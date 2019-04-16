(define (problem p01)
(:domain cubeWorld)
(:objects
fetch - robot
cube_1 cube_2 - cube
place_area_location - location
fetch_init_loc - location
cube_1_iloc cube_2_iloc - location
)
(:init
(Cube_At cube_1 cube_1_iloc)
(Cube_At cube_2 cube_2_iloc)
(Robot_At fetch fetch_init_loc)
(Empty_Basket fetch)
)
(:goal (and (Cube_At cube_1 place_area_location) (Cube_At cube_2 place_area_location) ) )
)