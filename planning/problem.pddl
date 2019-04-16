(define (problem p01)
(:domain cubeWorld)
(:objects
fetch - robot
cube_1 cube_2 cube_3 cube_4 cube_5 - cube
place_area_location - location
fetch_init_loc - location
cube_1_iloc cube_2_iloc cube_3_iloc cube_4_iloc cube_5_iloc - location
)
(:init
(Cube_At cube_1 cube_1_iloc)
(Cube_At cube_2 cube_2_iloc)
(Cube_At cube_3 cube_3_iloc)
(Cube_At cube_4 cube_4_iloc)
(Cube_At cube_5 cube_5_iloc)
(Robot_At fetch fetch_init_loc)
(Empty_Basket fetch)
)
(:goal (and (Cube_At cube_1 place_area_location) (Cube_At cube_2 place_area_location) (Cube_At cube_3 place_area_location) (Cube_At cube_4 place_area_location) (Cube_At cube_5 place_area_location) ) )
)