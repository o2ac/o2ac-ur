(define (problem osx-assembly)
  (:domain osx-assembly)
  (:objects
    base panel_bearing sub-assembly-panel_bearing - assembly
    screw_tool_m3 screw_tool_m4 - tool
    a_bot b_bot - robot
    panel_bearing/bottom_screw_hole_1 panel_bearing/bottom_screw_hole_2 - screw
  )
  (:init
    ; Robots are free
    (not (robot-carries-an-object a_bot))
    (not (robot-carries-an-object b_bot))

    ; Tools are free and available
    (not (tool-carries-screw screw_tool_m3))
    (not (tool-carries-screw screw_tool_m4))
        
    (available screw_tool_m3)
    (available screw_tool_m4)

    ; Parts init
    (not (available base))
      (not (requires-screws base))
      (incorporated base sub-assembly-panel_bearing)

    (available panel_bearing)
      (requires-screws panel_bearing)
        (has-this-mating panel_bearing panel_bearing/bottom_screw_hole_1)
          (screw-requires-tool screw_tool_m4 panel_bearing/bottom_screw_hole_1)
        (has-this-mating panel_bearing panel_bearing/bottom_screw_hole_2)
          (screw-requires-tool screw_tool_m4 panel_bearing/bottom_screw_hole_2)

    ; Assembly init

    (part-of base sub-assembly-panel_bearing)
    (part-of panel_bearing sub-assembly-panel_bearing)
        
    (assemble-order base panel_bearing sub-assembly-panel_bearing)
        
  )
          
  (:goal (complete sub-assembly-panel_bearing))
)
   