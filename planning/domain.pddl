(define (domain cubeWorld)

    (:requirements
        :equality
        :typing
        :strips
    )

    (:types
        robot
        cube
        location
    )


    (:predicates
        (Cube_At ?a - cube ?b - location)
        (Robot_At ?a - robot ?b - location)
        (Empty_Basket ?f - robot)
    )


    (:action pick
        :parameters ( ?cube - cube ?bot - robot ?loccube - location)
        :precondition (and 
            (Robot_At ?bot ?loccube)
            (Cube_At ?cube ?loccube)
            (Empty_Basket ?bot)
        )
        :effect (and
            (not (Cube_At ?cube ?loccube))
            (Robot_At ?bot ?loccube)
            (not (Empty_Basket ?bot))
        )
    )

    (:action place
        :parameters (?cube - cube ?bot - robot ?area_load_loc - location)
        :precondition (and
            (Robot_At ?bot ?area_load_loc)
            (not (Empty_Basket ?bot))
        )
        :effect (and
            (Robot_At ?bot ?area_load_loc)
            (Cube_At ?cube ?area_load_loc)
            (Empty_Basket ?bot)
        )
    )
    
    (:action move
        :parameters (?bot - robot ?oldloc - location ?newloc - location)
        :precondition (and
            (Robot_At ?bot ?oldloc)
        )
        :effect (and
            (Robot_At ?bot ?newloc)
            (not (Robot_At ?bot ?oldloc))
        )
    )
)
