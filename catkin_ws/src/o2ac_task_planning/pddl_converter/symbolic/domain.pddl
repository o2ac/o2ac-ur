(define (domain osx-assembly)
  (:requirements :adl)
  (:types
    assembly tool - object  ; The type "assembly" includes individual parts (= atomic assemblies)
    robot - robot
    mating screw insert - mating  ; Thy type mating defines how a part is combined with an other
  )

  (:predicates 
    ; General object predicates
    (available ?x - object) ; True if object is in tray or tool in holder

    ; Assembly/part properties
    (complete ?a - assembly)
    (part-of ?part ?whole - assembly)  ; True if part belongs into the whole

    ; Part states
    (incorporated ?part ?whole - assembly)  ; "Incorporated" parts are currently part of the assembly (they are "assembled", or "mounted")
    (object-is-stable ?o - assembly)  ; This is true if a screw has been fastened, or the object is horizontal (e.g. bearing, peg...) (this might not work for all parts)
    (object-is-placed ?o - assembly)  ; True when the object is at the goal/target location and not in the tray

    ; Assembly hierarchy
    (assemble-order ?part1 ?part2 ?whole - assembly)  ; Part 2 needs to come after Part 1

    ; Mating properties
    (requires-screws ?a - assembly) ; True if the part needs to be fastened with screws
    (has-this-mating ?part - assembly ?mating - mating) ; The part has mating
    (screw-requires-tool ?tool - tool ?screw - screw) ; This screw requires the tool: 'tool'   Screw size fits this tool
    (mating-satisfied ?mating - mating) ; True if the mating is satisfied

    ; Robot & tool properties
    (robot-carries-an-object ?r - robot) ; This could probably be evaluated with a (for all) clause on robot-carries-this-object instead, but having this is probably easier.
    (robot-carries-this-object ?r - robot ?o - object)  ; Can be a tool or part
    (tool-carries-screw ?t - tool ); True when tool has picked a screw
  )


  ; === Actions for pick/place/release    

  (:action pick
    :parameters (?robot - robot ?part - object ?helper_robot - robot)
    :precondition(
      and
        (available ?part)
        (not (robot-carries-an-object ?robot))
        (not (robot-carries-an-object ?helper_robot))
        (not(= ?robot ?helper_robot))
    )
    :effect(
      and
        (robot-carries-an-object ?robot)
        (robot-carries-this-object ?robot ?part)
        (not (available ?part))  ; This implies that only one of each part exists in the scene
    )
  )
           
  (:action place
    :parameters (?robot - robot ?part ?prev ?whole - assembly)
    :precondition(
      and
        (robot-carries-this-object ?robot ?part)
        (part-of ?part ?whole)
        (assemble-order ?prev ?part ?whole)
        (incorporated ?prev ?whole)
    )
    :effect(
      and
        (object-is-placed ?part)
        (when (not (requires-screws ?part))
          (object-is-stable ?part)
        )
    )
  )
    
  (:action release
    :parameters (?robot - robot ?part ?whole - assembly)
    :precondition(
      and
        (robot-carries-this-object ?robot ?part)
        (object-is-stable ?part)
        (part-of ?part ?whole)
        (forall (?prev - assembly) ; = all previous parts have been assembled
          (imply (assemble-order ?prev ?part ?whole)
            (incorporated ?prev ?whole)
          )
        )
    )
    :effect(
      and 
        (not (robot-carries-this-object ?robot ?part))
        (not (robot-carries-an-object ?robot))
        (when (not (requires-screws ?part)) (incorporated ?part ?whole))
    )
  )


  ; === Tool-related actions (equip/pick-screw/fasten)

  (:action equip
    :parameters (?robot - robot ?tool - tool)
    :precondition(
      and
        (available ?tool)
        (not (robot-carries-an-object ?robot))
    )
    :effect(
      and
        (robot-carries-an-object ?robot)
        (robot-carries-this-object ?robot ?tool)
        (not (available ?tool))
    )
  )

  (:action unequip
    :parameters (?robot - robot ?tool - tool)
    :precondition (robot-carries-this-object ?robot ?tool)
    :effect(
      and 
        (not (robot-carries-this-object ?robot ?tool))
        (not (robot-carries-an-object ?robot))
        (available ?tool)
    )
  )
    
  (:action pick-screw
      :parameters (?robot - robot ?tool - tool)
      :precondition(
        and
          (robot-carries-this-object ?robot ?tool)
          (not (tool-carries-screw ?tool))
      )
      :effect (tool-carries-screw ?tool)
  )
  
  (:action fasten-with-screw
    :parameters (?robot - robot ?tool - tool ?part - assembly ?screw - screw ?whole - assembly)
    :precondition(
      and
        (robot-carries-this-object ?robot ?tool)
        (tool-carries-screw ?tool)
        (screw-requires-tool ?tool ?screw)
        (has-this-mating ?part ?screw)
        (not (mating-satisfied ?screw))
        (object-is-placed ?part)
        (requires-screws ?part)
        (part-of ?part ?whole)
    )
    :effect(
      and
        (not (tool-carries-screw ?tool))
        (object-is-stable ?part)
        (mating-satisfied ?screw)
        (when
          (forall (?s - screw)
            (
              imply(
                and
                  (has-this-mating ?part ?s)
                  (not(= ?s ?screw))
              )
              (mating-satisfied ?s)
            )
          )
          (incorporated ?part ?whole)
        )
    )
  )


  ; Ending action (can only be accessed if all part of the assembly are incorporated and released)

  (:action finished
    :parameters (?whole - assembly)
    :precondition(
      forall (?p - assembly ?r - robot)
        (imply (part-of ?p ?whole)
          (
            and
              (incorporated ?p ?whole)
              (not (robot-carries-this-object ?r ?p))
          )
        )
    )
    :effect(
      and
        (complete ?whole)
        (available ?whole)
    )
  )
)
