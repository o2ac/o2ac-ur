(define (problem pick-place-test)
  (:domain osx-assembly)
  (:objects
    base panel_bearing panel_motor sub-assembly-panel_bearing - assembly
    a_bot b_bot - robot
  )
  (:init
    ; Robots are free
    (not (robot-carries-an-object a_bot))
    (not (robot-carries-an-object b_bot))

    ; Parts init
    (not (available base))
      (not (requires-screws base))
      (incorporated base sub-assembly-panel_bearing)

    (available panel_bearing)
    (available panel_motor)

    ; Assembly init

    (part-of base sub-assembly-panel_bearing)
    (part-of panel_bearing sub-assembly-panel_bearing)
    (part-of panel_motor sub-assembly-panel_bearing)
        
    (assemble-order base panel_bearing sub-assembly-panel_bearing)
    (assemble-order base panel_motor sub-assembly-panel_bearing)
        
  )
          
  (:goal (and (object-is-placed panel_bearing) (object-is-placed panel_motor)))
)
   