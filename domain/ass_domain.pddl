(define (domain ass_domain)
    (:requirements :strips :typing :durative-actions :fluents)

    (:types
        robot
        point
        marker
    )

    (:predicates
        (robot_at ?r - robot ?p - point)
        (marker_at ?m - marker ?p - point)
        (is_next ?m1 ?m2 - marker)
        (is_first ?m - marker)
        (visited ?m - marker)
        (unvisited ?m - marker)
        (photo_taken ?m - marker)
        (photo_untaken ?m - marker)
        (robot_free ?r - robot)
        (detecting ?r - robot)
        (acquiring_imgs ?r - robot)
        (is_base ?p - point)

    )

    (:functions
        (state ?s - robot)
        (num_id_detected)
        (num_photo_taken)
        (num_marker)
        (marker_id ?m - marker)
    )

    (:durative-action detect_id
        :parameters (?r - robot ?p1 ?p2 - point ?m1 - marker)
        :duration ( = ?duration 5)
        :condition (and
            (at start(detecting ?r))
            (over all(marker_at ?m1 ?p1))
            (at start(unvisited ?m1))
            (at start(robot_at ?r ?p2))
            (at start (robot_free ?r))
            ; (at start(is_first ?m1))

        )
        :effect (and
            (at start(not(robot_free ?r)))
            (at end(robot_free ?r))
            (at end(robot_at ?r ?p1))
            (at end(not(robot_at ?r ?p2)))
            (at end(not(unvisited ?m1)))
            (at end(visited ?m1))
            (at end(increase (num_id_detected) 1))
        )
    )

    (:durative-action change_to_acquire_state
        :parameters (?r - robot ?m1 - marker)
        :duration ( = ?duration 3)
        :condition (and 
            (at start(detecting ?r))
            (at start(robot_free ?r))
            (at start(visited ?m1))
        )
        :effect(and
            (at end(not(detecting ?r)))
            (at end(acquiring_imgs ?r))
            (at start(not(robot_free ?r)))
            (at end(robot_free ?r))


        )
    )

    (:durative-action capture_first_img
        :parameters (?r - robot ?p1 ?p2 - point ?m1 - marker)
        :duration ( = ?duration 5)
        :condition (and
            (at start(photo_untaken ?m1))
            (at start(is_first ?m1))
            (at start(acquiring_imgs ?r))
            (over all(marker_at ?m1 ?p1))
            (at start(robot_at ?r ?p2))
            (at start(robot_free ?r))
        )
        :effect (and
            (at start(not(robot_free ?r)))
            (at end(robot_free ?r))
            (at end(robot_at ?r ?p1))
            (at end(not (robot_at ?r ?p2)))
            (at end(photo_taken ?m1))
            (at end(not(photo_untaken ?m1)))  
            (at end(increase (num_photo_taken) 1))  
        )
    )

    (:durative-action capture_other_imgs
        :parameters (?r - robot ?p1 ?p2 - point ?m1 ?m2 - marker)
        :duration ( = ?duration 5)
        :condition (and
            (at start(photo_untaken ?m1))
            (at start(is_next ?m1 ?m2))
            (at start(photo_taken ?m2))
            (at start(acquiring_imgs ?r))
            (over all(marker_at ?m1 ?p1))
            (at start(robot_at ?r ?p2))
            (at start(robot_free ?r))
        )
        :effect (and
            (at start(not(robot_free ?r)))
            (at end(robot_free ?r))
            (at end(robot_at ?r ?p1))
            (at end(not (robot_at ?r ?p2)))
            (at end(photo_taken ?m1))
            (at end(not(photo_untaken ?m1)))  
            (at end(increase (num_photo_taken) 1))  
        )
    )

    (:durative-action return_to_base
        :parameters (?r - robot ?base ?p - point)
        :duration (= ?duration 7)
        :condition (and 
            (at start(robot_free ?r))
            (at start(is_base ?base))
            (at start(robot_at ?r ?p))
            ; (at start(= (num_photo_taken) (num_marker)))
        )
        :effect (and 
            (at start(not(robot_free ?r)))
            (at end(robot_free ?r))
            (at end(robot_at ?r ?base))
            (at end(not(robot_at ?r ?p)))
        )
    )
)