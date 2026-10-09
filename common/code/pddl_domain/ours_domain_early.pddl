(define (domain tamp)

(:requirements :strips :fluents :durative-actions :typing :conditional-effects :adl :continuous-effects :duration-inequalities :universal-preconditions :timed-initial-literals)

(:types waypoint robot job jobtype - object
        normal_way col_way - waypoint
        single_task - jobtype
)

(:predicates (at ?r - robot ?w - waypoint)
             (connected ?w1 ?w2 - waypoint)
             (located ?j - job ?w - waypoint)
             (vertex_free ?w - waypoint)
             (reserved ?w - col_way)
             (work_free ?j - job)
             (finish ?j - job ?t - jobtype)
             (isJobOfType ?j - job ?t - jobtype)
             (can_perform ?r - robot ?t - jobtype)
             (before ?j - job ?t1 ?t2 - jobtype)
             (robot_way_token ?r - robot)
             (robot_free ?r - robot)
)

(:functions (distance ?from ?to - waypoint)
            (task_duration ?r - robot ?t - jobtype)
            (velocity ?r - robot)
)

(:durative-action normal_navigate
    :parameters (?r - robot ?from ?to - normal_way)
    :duration (= ?duration (/ (distance ?from ?to) (velocity ?r)))
    :condition (and (at start (at ?r ?from))
                    (over all (vertex_free ?to))
                    (over all (connected ?from ?to)))
    :effect (and (at start (not (at ?r ?from)))
                 (at start (not (robot_way_token ?r)))
                 (at start (vertex_free ?from))
                 (at end (not (vertex_free ?to)))
                 (at end (robot_way_token ?r))
    		  (at end (at ?r ?to)))
)

(:durative-action col_navigate
    :parameters (?r - robot ?from ?to - col_way)
    :duration (= ?duration (/ (distance ?from ?to) (velocity ?r)))
    :condition (and (at start (at ?r ?from))
                    (at start (reserved ?to))
                    (at start (reserved ?from))
                    (over all (vertex_free ?to))
                    (over all (connected ?from ?to)))
    :effect (and (at start (not (at ?r ?from)))
                 (at start (vertex_free ?from))
                 (at end (not (vertex_free ?to)))
                 (at start (not (reserved ?to)))
                 (at end (reserved ?to))
                 (at start (not (reserved ?from)))
                 (at end (reserved ?from))
                 (at end (at ?r ?to)))
)

(:durative-action normal_to_col_navigate
    :parameters (?r - robot ?from - normal_way ?to - col_way)
    :duration (= ?duration (/ (distance ?from ?to) (velocity ?r)))
    :condition (and (at start (at ?r ?from))
                    (at start (reserved ?to))
                    (over all (vertex_free ?to))
                    (over all (connected ?from ?to)))
    :effect (and (at start (not (at ?r ?from)))
                 (at start (vertex_free ?from))
                 (at end (not (vertex_free ?to)))
                 (at start (not (reserved ?to)))
                 (at end (reserved ?to))
                 (at end (at ?r ?to)))
)

(:durative-action col_to_normal_navigate
    :parameters (?r - robot ?from - col_way ?to - normal_way)
    :duration (= ?duration (/ (distance ?from ?to) (velocity ?r)))
    :condition (and (at start (at ?r ?from))
                    (at start (reserved ?from))
                    (over all (vertex_free ?to))
                    (over all (connected ?from ?to)))
    :effect (and (at start (not (at ?r ?from)))
                 (at start (vertex_free ?from))
                 (at end (not (vertex_free ?to)))
                 (at start (not (reserved ?from)))
                 (at end (reserved ?from))
                 (at end (at ?r ?to)))
)

(:durative-action do_task_single
    :parameters (?r - robot ?w - waypoint ?j - job ?s - single_task)
    :duration (= ?duration (task_duration ?r ?s))
    :condition (and 
                    (at start (work_free ?j))
                    (over all (isJobOfType ?j ?s))
                    (at start (can_perform ?r ?s))
                    (over all (at ?r ?w))
                    (over all (located ?j ?w))
                    (at start (forall (?x - jobtype) (imply (before ?j ?x ?s) (finish ?j ?x)))))
    :effect (and (at start (not (work_free ?j)))
                 (at end (finish ?j ?s))
                 (at end (work_free ?j)))
)


)


