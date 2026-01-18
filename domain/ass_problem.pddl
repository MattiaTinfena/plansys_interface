(define 
    (problem ass_problem)
    (:domain ass_domain)
    (:objects
        robot1 - robot
        base p1 p2 p3 p4 - point
        m1 m2 m3 m4 - marker
    )

    (:init
        (= (num_marker) 4)
        (robot_at robot1 base)
        (robot_free robot1)
        (detecting robot1)

        (= (num_id_detected) 0)
        (= (num_photo_taken) 0)

        (= (marker_id m1) -1)
        (= (marker_id m2) -1)
        (= (marker_id m3) -1)
        (= (marker_id m4) -1)

        (is_base base)

        (marker_at m1 p1)
        (marker_at m2 p2)
        (marker_at m3 p3)
        (marker_at m4 p4)

        (unvisited m1)
        (unvisited m2)
        (unvisited m3)
        (unvisited m4)

        (photo_untaken m1)
        (photo_untaken m2)
        (photo_untaken m3)
        (photo_untaken m4)

        (is_first m3)
        (is_next m4 m3)
        (is_next m1 m4)
        (is_next m2 m1)
    )

    (:goal
        ( and
            (visited m1)
            (visited m2)
            (visited m3)
            (visited m4)
            (acquiring_imgs robot1)
            ; (photo_taken m1)            
            ; (photo_taken m2)      
            ; (photo_taken m3)      
            ; (photo_taken m4)  
            ; (robot_at robot1 base)    
        )
    )
)