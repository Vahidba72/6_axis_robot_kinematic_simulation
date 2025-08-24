(define (domain My_Domain)
  ;remove requirements that are not needed
  (:requirements :strips :typing :fluents :disjunctive-preconditions)
  
  (:types
    robot
    waypoint
    object
    gripper
  )
  
  ; un-comment following line if constants are needed
  ;(:constants )
  
  (:predicates
    (robot_at ?r - robot ?w - waypoint)
    (gripper_empty ?g - gripper)
    (gripper_holding ?g - gripper ?o - object)
    (robot_has_gripper ?r - robot ?g - gripper)
    (object_at ?o - object ?w - waypoint)
    (object_graspable ?o - object)
    (location_clear ?w - waypoint)
    (location_accessible ?w - waypoint)
    (adjacent ?w1 - waypoint ?w2 - waypoint)
    (gripper_open ?g - gripper)  ; Added this predicate since it's used in effects
  )
  
  ;(:functions ;todo: define numeric functions here
  ;)
  
  ;define actions here
  ; Action to move the robot end effector from one waypoint to another
  (:action go_to
    :parameters (?r - robot ?from ?to - waypoint)
    :precondition (and
      (robot_at ?r ?from)
      (location_accessible ?to)
      (adjacent ?from ?to)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
    )
  )
  
  (:action pick_up
    :parameters (?r - robot ?g - gripper ?o - object ?w - waypoint)
    :precondition (and
      (robot_at ?r ?w)
      (robot_has_gripper ?r ?g)
      (gripper_empty ?g)
      (object_at ?o ?w)
      (object_graspable ?o)
    )
    :effect (and
      (not (gripper_empty ?g))
      (gripper_holding ?g ?o)
      (not (object_at ?o ?w))
      (location_clear ?w)
      (gripper_open ?g)
    )
  )
  
  (:action place
    :parameters (?r - robot ?g - gripper ?o - object ?w - waypoint)
    :precondition (and
      (robot_at ?r ?w)
      (robot_has_gripper ?r ?g)
      (gripper_holding ?g ?o)
      (location_clear ?w)
    )
    :effect (and
      (not (gripper_holding ?g ?o))
      (gripper_empty ?g)
      (object_at ?o ?w)
      (not (location_clear ?w))
    )
  )
)