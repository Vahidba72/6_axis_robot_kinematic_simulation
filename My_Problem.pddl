(define (problem My_Problem) (:domain My_Domain)
(:objects

    robot1 - robot
    gripper1 - gripper
    can1 - object
    cube1 - object
    table1 - waypoint
    table2 - waypoint
    shelf1 - waypoint
    home - waypoint
)

(:init
    (robot_at robot1 home)
    (robot_has_gripper robot1 gripper1)
    (gripper_empty gripper1)

    (object_at can1 table1)
    (object_at cube1 table1)

    (object_graspable can1)
    (object_graspable cube1)

    (location_accessible home)
    (location_accessible table1)
    (location_accessible table2)
    (location_accessible shelf1)

    (location_clear table2)
    (location_clear shelf1)

    (adjacent home table1)
    (adjacent table1 home)
    (adjacent table1 table2)
    (adjacent table2 table1)
    (adjacent table2 shelf1)
    (adjacent shelf1 table2)
    (adjacent home shelf1)
    (adjacent shelf1 home)

   

)

(:goal (and
    
    (object_at can1 table2)
    (object_at cube1 shelf1)

    (robot_at robot1 home)
    (gripper_empty gripper1)
    
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)